import copy
import math
import numpy as np
import opendbc.custom.loger as trace1
import cereal.messaging as messaging

from typing import  List, Tuple
from cereal import car, log
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import CAR, Buttons
from opendbc.custom.params_json import read_json_file
from openpilot.common.params import Params

LaneChangeState = log.LaneChangeState


class CarStateCustom:
  # ----------------------------
  # Tunables / Guard constants
  # ----------------------------
  INIT_DELAY_FRAMES = 500           # 초기 5s 동안 강제 disengage
  RESUME_LONGPRESS_FRAMES = 55      # 버튼 길게 누름 임계
  TARGET_MIN_KPH = 30               # SCC 세트 최소 속도

  # ===== 튜닝 상수 =====
  LANE_PROB_TH      = 0.50          # 차선 인식 확률 임계값
  LANE_HOLD_TICKS   = 50            # 차선 가시 타이머 초기값
  WAIT_SYNC_TICKS   = 200           # 좌/우 깜빡이 동시 입력 시 대기

  # Curve penalty (kph)
  CURVE_Y_ABS_RANGE: Tuple[float, float] = (10.0, 60.0)   # |y| 편차 맵 구간
  CURVE_PENALTY_RANGE: Tuple[float, float] = (0.0, 10.0)  # 감속량 맵




  def __init__(self, CP, CS):
    self.CS = CS
    self.CP = CP

    # Messaging / Params
    self.sm = messaging.SubMaster(
      ['longitudinalPlan', 'modelV2', 'pandaStates', 'uICustom'],
      ignore_avg_freq=['uICustom']
    )
    self.params = Params()

    # 상태/타이머
    self.frame = 0
    self.timer_init = self.INIT_DELAY_FRAMES


    # ACC/크루즈 상태
    self.acc_active = 0
    self.old_acc_active = 0
    self.prev_acc_active = 0
    self.oldCruiseStateEnabled = False
    self.controlsAllowed = 0

    # 버튼/세트 속도
    self.cruise_buttons_old = 0
    self.prev_cruise_btn = 0
    self.cruise_buttons_time = 0
    self.cruise_set_speed_kph = 0.0
    self.VSetDis = 0.0
    self.lead_distance = 0.0
    self.gapSet = 0
    self._gas_pressed_prev = False
    self._cencel_button = True

    # 차량/주행 상태
    self.clu_Vanz = 0.0
    self.is_highway = False
    self.steeringAngle = 0.0
    self.modelxDistance = 0.0
    self.modelyDistance = 0.0
    self.model_v2 = None
    self.brakePos = 0.0
    self.clu_Main = 0
    self.mainMode_ACC = False


    # 외부/플래너 파생
    self.speed_plan_kph = 0.0
    self.cruise_set_mode = 0
    self.cruiseGap = 0
    self.control_mode = 0
    self.Cruise_Limit_Status = 0  # 속도 제한(ISA/크루즈 제한) 기능의 활성/유효 여부. 0=비활성, 1=활성 같은 토글.
    self.Cruise_Limit_Target = 0  # 제한 속도 목표값(보통 km/h). 표지판 인식/맵 기반 제한 속도가 여기 들어올 수 있음.


    # 좌/우 차선변경 헬퍼(유지)
    self.leftLaneTime = self.LANE_HOLD_TICKS
    self.rightLaneTime = self.LANE_HOLD_TICKS
    self.lanechange_wait = 100


    # Custom menu (robust defaults)
    self.autoLaneChange, self.menu_debug, self.curveSpeedLimit, self.autoEngage = self._load_custom_params()



    # 지원 차량 목록
    self.cars = self._get_supported_cars(CP)


  # ----------------------------
  # Config / Params
  # ----------------------------
  def _load_custom_params(self) -> Tuple[int, int, float, int]:
    """Safely load custom menu params with robust fallbacks."""
    try:
      m_jsonobj = read_json_file("CustomParam")
      autoLaneChange = int(m_jsonobj.get("ParamAutoLaneChange", 0))
      menu_debug = int(m_jsonobj.get("ParamDebug", 0))
      curveSpeedLimit = float(m_jsonobj.get("ParamCurveSpeedLimit", 40))
      autoEngage = int(m_jsonobj.get("ParamAutoEngage", 0))
      return autoLaneChange, menu_debug, curveSpeedLimit, autoEngage
    except Exception:
      return 0, 0, 40.0, 0

  # ----------------------------
  # Misc
  # ----------------------------
  @staticmethod
  def _safe_vl_map(cp, msg: str):
    """메시지 맵 복사 (없을 경우 빈 dict)"""
    try:
      return cp.vl[msg]
    except Exception:
      return {}

  # ----------------------------
  # Helpers: CAN value access
  # ----------------------------
  @staticmethod
  def _vl(cp, msg: str, sig: str, default=0):
    """안전한 단일 신호 읽기"""
    try:
      return cp.vl[msg][sig]
    except Exception:
      return default

  @staticmethod
  def _vla(cp, msg: str, sig: str, default=0):
    """안전한 배열형 신호 읽기 (vl_all)"""
    try:
      return cp.vl_all[msg][sig]
    except Exception:
      return default

  @staticmethod
  def _finite_or(v, fb=0.0):
    try:
      return v if math.isfinite(v) else fb
    except Exception:
      return fb

  @staticmethod
  def curvature_to_steering_angle(curvature, wheelbase_m=2.7):
      """
      curvature : float, 모델 출력 desiredCurvature [1/m]
      wheelbase_m : 차량 휠베이스 [m] (기본값 2.7m 예시)

      return: steering angle [deg]
      """
      if not math.isfinite(curvature):
          return 0.0

      angle_rad = math.atan(wheelbase_m * curvature)
      angle_deg = math.degrees(angle_rad)
      angle_deg *= 16.0
      return angle_deg


  # ----------------------------
  # Model-based helpers
  # ----------------------------
  @staticmethod
  def _max_model_distance(model_v2):
    """modelV2 position의 마지막 포인트를 가져와 범위를 클램프"""
    if model_v2 is None or getattr(model_v2, "position", None) is None:
      return None, None

    xs = getattr(model_v2.position, "x", []) or []
    ys = getattr(model_v2.position, "y", []) or []

    last_x = xs[-1] if xs else None
    last_y = ys[-1] if ys else None

    x = float(np.clip(last_x, 10, 500)) if last_x is not None else None
    y = float(np.clip(last_y, -60, 60)) if last_y is not None else None
    return x, y

  # ----------------------------
  # Public-ish helpers
  # ----------------------------
  def _get_supported_cars(self, CP):
    cars = []
    for _, member in CAR.__members__.items():
      cars.append(member.value)
    return cars


  def _update_controls_allowed(self):
    """pandaStates 기반 제어 허용 플래그 업데이트"""
    self.controlsAllowed = 1 if any(ps.controlsAllowed for ps in self.sm['pandaStates']) else 0


  # ----------------------------
  # SubMaster ingestion
  # ----------------------------
  def _update_from_submaster(self):
    """SubMaster에서 보조 신호 갱신. sm.update(0)는 여기서만 호출합니다."""
    self.sm.update(0)

    # Planner 기반 속도 (kph)
    speeds = getattr(self.sm['longitudinalPlan'], 'speeds', [])
    if len(speeds):
      self.speed_plan_kph = float(speeds[-1]) * CV.MS_TO_KPH

    # UI Custom
    if self.sm.updated.get("uICustom", False):
      ui_comm = self.sm['uICustom'].community
      cm = int(getattr(ui_comm, 'cruiseMode', self.cruise_set_mode))
      cg = int(getattr(ui_comm, 'cruiseGap', self.cruiseGap))
      csl = float(getattr(ui_comm, 'curveSpeedLimit', self.curveSpeedLimit))
      if self.cruise_set_mode != cm: self.cruise_set_mode = cm
      if self.cruiseGap != cg:       self.cruiseGap = cg
      if getattr(self, 'curveSpeedLimit', None) != csl: self.curveSpeedLimit = csl


    # 모델 곡률/이격
    mdl = self.sm['modelV2']
    self.model_v2 = mdl
    self.laneChangeState = getattr(mdl.meta, 'laneChangeState', None)
    self.laneLineProbs = list(getattr(mdl, 'laneLineProbs', []))

    try:
      desired_curv = float(mdl.action.desiredCurvature)
      if not math.isfinite(desired_curv): desired_curv = 0.0
    except Exception:
      desired_curv = 0.0

    # 곡률 → 조향각(도)
    wb = self._finite_or(getattr(self.CP, 'wheelbase', 2.7), 2.7)
    self.steeringAngle = self.curvature_to_steering_angle( desired_curv, wb )

    x_d, y_d = self._max_model_distance(mdl)
    # None 대비 기본값 0.0
    self.modelxDistance = x_d if x_d is not None else 0.0  # 앞차와의 거리
    self.modelyDistance = y_d if y_d is not None else 0.0  # 곡률

    if self.clu_Vanz > self.curveSpeedLimit:
      # 곡률 기반 속도 페널티(간단한 예: 곡률 y 편차 10~60 → 0~10 kph 감속)
      y_abs = abs(self.modelyDistance)
      xp = list(self.CURVE_Y_ABS_RANGE)
      fp = list(self.CURVE_PENALTY_RANGE)
      spd_penalty = float(np.interp(y_abs, xp, fp, left=fp[0], right=fp[1]))
      self.speed_plan_kph = max(0.0, self.speed_plan_kph - spd_penalty)

  # ----------------------------
  # Longitudinal mode & cruise speed
  # ----------------------------
  def _update_longitudinal_mode_and_speed(self, ret, cp, cp_cruise):
    """OP Long on/off 분기 + cruiseState.speed 관리."""
    if self.CP.openpilotLongitudinalControl:
      # OP long ON: vehicle ACC signals are informational only
      self.mainMode_ACC = (self._vl(cp, "TCS13", "ACCEnable", 1) == 0)
      self.acc_active = (self._vl(cp, "TCS13", "ACC_REQ", 0) == 1)
      self.lead_distance = 0.0
      # ret.cruiseState.speed is managed by OP
    else:
      # Stock SCC path
      self.mainMode_ACC = (self._vl(cp_cruise, "SCC11", "MainMode_ACC", 0) == 1)
      self.acc_active = (self._vl(cp_cruise, "SCC12", "ACCMode", 0) != 0)
      self.lead_distance = self._vl(cp_cruise, "SCC11", "ACC_ObjDist", 0.0)
      self.gapSet = self._vl(cp_cruise, "SCC11", "TauGapSet", 0)
      self.VSetDis = self._vl(cp_cruise, "SCC11", "VSetDis", 0.0)  # kph

      if self.acc_active:
        ret.cruiseState.speed = self._cruise_speed_button_logic() * CV.KPH_TO_MS
      else:
        ret.cruiseState.speed = 0.0

      # Only when stock main ACC is OFF, allow menu toggle via buttons
      if not self.mainMode_ACC:
        self._cruise_control_mode_toggle()

  def _cruise_control_mode_toggle(self):
    """RES/SET 버튼으로 메뉴 모드 토글(기존 로직 유지)"""
    cruise_buttons = getattr(self.CS, "prev_cruise_buttons", 0)
    if cruise_buttons == self.cruise_buttons_old:
      return


    self.cruise_buttons_old = cruise_buttons
    if cruise_buttons == Buttons.RES_ACCEL:
      self.control_mode +=   1
    elif cruise_buttons == Buttons.SET_DECEL:
      self.control_mode -=  1

    if self.control_mode < 0 or self.control_mode > 5:
      self.control_mode = 0


  # ----------------------------
  # Cruise set-speed via buttons
  # ----------------------------
  def _update_button_press_timer(self, cruise_buttons: int):
    """RES/SET 길게 누름 카운팅"""
    if cruise_buttons in (Buttons.RES_ACCEL, Buttons.SET_DECEL):
      self.cruise_buttons_time += 1
    else:
      self.cruise_buttons_time = 0

  def _handle_longpress_set_vset(self) -> bool:
    """길게 누름이면 클러스터 세트속도로 고정 (True 반환 시 처리 완료)"""
    if self.cruise_buttons_time >= self.RESUME_LONGPRESS_FRAMES:
      self.cruise_set_speed_kph = self.VSetDis
      return True
    return False

  def _sync_on_gas_release(self, gas_now: bool) -> None:
    """Gas pedal released: sync set-speed to current if difference large."""
    if not gas_now and self._gas_pressed_prev:
      diff = float(self.clu_Vanz) - float(self.cruise_set_speed_kph)
      if diff >= 5.0:
        self.cruise_set_speed_kph = max(float(self.clu_Vanz), float(self.TARGET_MIN_KPH))
    self._gas_pressed_prev = gas_now

  def _cruise_speed_button_logic(self) -> float:
    """버튼 입력 기반의 세트 속도 관리(원 로직 유지, 가드 추가)"""
    # ACC 활성/비활성 전환 감지
    if self.prev_acc_active != self.acc_active:
      self.old_acc_active = self.prev_acc_active
      self.prev_acc_active = self.acc_active
      self._cencel_button = False
      self.cruise_set_speed_kph = self.VSetDis

    _gas_now = bool(getattr(self.CS.out, "gasPressed", False))
    if not self.acc_active:
      self._gas_pressed_prev = _gas_now
      return float(self.cruise_set_speed_kph)


    # === 가속 페달 해제 시 현재 속도로 동기화 ===
    self._sync_on_gas_release(_gas_now)


    cruise_buttons = getattr(self.CS, "prev_cruise_buttons", 0)
    self._update_button_press_timer(cruise_buttons)

    # 길게 누름 처리
    if self._handle_longpress_set_vset():
      return float(self.cruise_set_speed_kph)

    # 같은 버튼 반복 입력 → 무시
    if self.prev_cruise_btn == cruise_buttons:
      return float(self.cruise_set_speed_kph)
    self.prev_cruise_btn = cruise_buttons

    set_speed_kph = self.cruise_set_speed_kph

    if cruise_buttons == Buttons.RES_ACCEL:
      set_speed_kph = self.VSetDis + 1
    elif cruise_buttons == Buttons.SET_DECEL:
      self._cencel_button = False
      # 가속 페달 중이거나 방금 ACC가 꺼져 있었다면 현재 속도로 세팅
      if _gas_now or (not self.old_acc_active):
        set_speed_kph = self.clu_Vanz
      else:
        set_speed_kph = self.VSetDis - 1

    if set_speed_kph < self.TARGET_MIN_KPH:
      set_speed_kph = self.TARGET_MIN_KPH

    self.cruise_set_speed_kph = float(set_speed_kph)
    return float(set_speed_kph)



  # ----------------------------
  # Engage management (LFA/ACC)
  # ----------------------------
  def lfa_engage(self, ret):
    self._update_controls_allowed()

    # 초기 쿨다운
    if self.timer_init > 0:
      self.timer_init -= 1
      ret.cruiseState.enabled = False
      return

    # OP Long on이면 여기서 추가 처리 없음 (원 코드 의도 유지)
    if self.CP.openpilotLongitudinalControl:
      return

    # 안전 조건 불만족 시 타이머 재가동 & 이전 상태 해제
    unsafe = (
      ret.parkingBrake or
      ret.doorOpen or
      ret.seatbeltUnlatched or
      ret.gearShifter != car.CarState.GearShifter.drive
    )
    if unsafe:
      self.oldCruiseStateEnabled = False
      self.timer_init = self.INIT_DELAY_FRAMES  # 주석과 동작 일치
      return

    # 메인 ACC 사용불가 → 표시만 on
    if not ret.cruiseState.available:
      self.oldCruiseStateEnabled = True
      return

    # 이전에 enable 표시를 켰다면 유지
    if self.oldCruiseStateEnabled:
      ret.cruiseState.enabled = True

  # ----------------------------
  # Auto lane change
  # ----------------------------
  def _lane_visibility_bits(self, probs: List[float]) -> Tuple[int, int]:
    """
    Returns (left_bits, right_bits).
    bit1: outer, bit2: inner (원 코드 호환을 위해 동일 비트 사용)
    """
    left_vis, right_vis = 0, 0
    if probs and len(probs) >= 4:
      p0, p1, p2, p3 = map(float, probs[:4])
      if p3 > self.LANE_PROB_TH: right_vis |= 2
      if p2 > self.LANE_PROB_TH: right_vis |= 1
      if p1 > self.LANE_PROB_TH: left_vis  |= 1
      if p0 > self.LANE_PROB_TH: left_vis  |= 2
    return left_vis, right_vis


  def _auto_lane_change(self, ret):
    if self.autoLaneChange == 0:
      return
    """
    자동 차선변경 보조 로직
    - 튜닝 상수 상단 집약
    - 가드절 정리
    - lane 가시성/타이머/토크 가압 분리
    - 예외/누락 필드 방어
    """
    # ===== 차선 가시성 계산 =====
    left_vis, right_vis = self._lane_visibility_bits(self.laneLineProbs)


    # “안쪽” 차선이 보이면 타이머 리셋
    if (left_vis  & 2): self.leftLaneTime  = self.LANE_HOLD_TICKS
    if (right_vis & 2): self.rightLaneTime = self.LANE_HOLD_TICKS

    # ===== 타이머 갱신 =====
    self.lanechange_wait = max(0, self.lanechange_wait - 1)
    self.leftLaneTime    = max(0, self.leftLaneTime - 1)
    self.rightLaneTime   = max(0, self.rightLaneTime - 1)

    leftBlinker  = bool(getattr(ret, "leftBlinker", False))
    rightBlinker = bool(getattr(ret, "rightBlinker", False))

    # 안전한 토크 임계값
    steer_th = self.CS.params.STEER_THRESHOLD

    # ===== 상태 머신 =====
    if self.laneChangeState == LaneChangeState.off:
      # 양쪽 깜빡이 동시: 동기화 대기 진입
      if leftBlinker and rightBlinker:
        self.lanechange_wait = self.WAIT_SYNC_TICKS
        # 기존 코드 유지: 즉시 해제 (불필요하면 주석처리)
        ret.leftBlinker = False
        ret.rightBlinker = False
      # 최소 대기 보장
      elif self.lanechange_wait < self.autoLaneChange:
        self.lanechange_wait = self.autoLaneChange
      return

    if self.laneChangeState == LaneChangeState.preLaneChange:
      # 토크 가압 조건: 해당 방향 차선이 일정 시간 보였고(wait==0)
      if leftBlinker and self.leftLaneTime > 0 and self.lanechange_wait <= 0:
        ret.steeringTorque =  steer_th
        ret.steeringPressed = True
      elif rightBlinker and self.rightLaneTime > 0 and self.lanechange_wait <= 0:
        ret.steeringTorque = -steer_th
        ret.steeringPressed = True
      return

    return


  # ----------------------------
  # Vehicle state aggregation
  # ----------------------------
  def _update_vehicle_general_states(self, ret, cp, cp_cam) -> None:
    self.brakePos   = float(self._vl(cp, "E_EMS11", "Brake_Pedal_Pos", 0.0))
    ret.engineRpmDEPRECATED = float(self._vl(cp, "E_EMS11", "N", 0.0)) # opkr
    self.Cruise_Limit_Status = int(self._vl(cp, "E_EMS11", "Cruise_Limit_Status", 0))
    self.Cruise_Limit_Target = int(self._vl(cp, "E_EMS11", "Cruise_Limit_Target", 0))
    self.is_highway = bool(self._vl(cp_cam, "LFAHDA_MFC", "HDA_Icon_State", 0))
    self.clu_Vanz   = float(self._vl(cp, "CLU11", "CF_Clu_Vanz", 0.0))  # kph
    self.clu_Main   = int(self._vl(cp, "CLU11", "CF_Clu_CruiseSwMain", 0))



  # ----------------------------
  # Debug / telemetry
  # ----------------------------
  def _fill_tpms(self, tpms_msg, unit, fl, fr, rl, rr):
    """
    unit: 0:psi(입력값이 psi), 1:kPa, 2:bar
    내부 표시는 psi 기준으로 통일 (필요시 UI에서 단위 라벨만 바꾸세요)
    """
    # kPa -> psi = * 0.1450377377
    # bar -> psi = * 14.5037738
    if unit == 1: # kPa -> psi
      factor = 0.1450377377
    elif unit == 2:  # bar -> psi
      factor = 14.5037738
    else:
      factor = 1.0  # 이미 psi
    tpms_msg.unit = unit
    tpms_msg.fl = fl * factor
    tpms_msg.fr = fr * factor
    tpms_msg.rl = rl * factor
    tpms_msg.rr = rr * factor

  def _send_debug(self, ret, cp):

    carSCustom = car.CarState.CarSCustom.new_message()
    carSCustom.supportedCars = self.cars
    carSCustom.breakPos = float(self.brakePos)
    carSCustom.leadDistance = float(self.lead_distance)
    carSCustom.gapSet = int(self.gapSet)
    carSCustom.electGearStep = self._vl(cp, "ELECT_GEAR", "Elect_Gear_Step", 0)

    self._fill_tpms(
      carSCustom.tpms,
      self._vl(cp, "TPMS11", "UNIT", 0),
      self._vl(cp, "TPMS11", "PRESSURE_FL", 0.0),
      self._vl(cp, "TPMS11", "PRESSURE_FR", 0.0),
      self._vl(cp, "TPMS11", "PRESSURE_RL", 0.0),
      self._vl(cp, "TPMS11", "PRESSURE_RR", 0.0),
    )

    cruise_buttons = getattr(self.CS, "prev_cruise_buttons", 0)
    if cruise_buttons in (Buttons.CANCEL, Buttons.RES_ACCEL, Buttons.SET_DECEL):
      carSCustom.touched += 1
      if cruise_buttons == Buttons.CANCEL:
        self._cencel_button = True
    elif self.acc_active and bool(getattr(self.CS.out, "gasPressed", False)):
      carSCustom.touched += 1

    ret.carSCustom = carSCustom

    if self.menu_debug == 0:
      return
    # 로그 (원 포맷 유지)
    trace1.printf1('MD={:.0f},CA={:.0f}, CB={}'.format(self.control_mode, self.controlsAllowed, self._cencel_button))
    trace1.printf2('SA={:5.2f} , {:.0f}'.format(self.steeringAngle, int(self.mainMode_ACC) ))
    trace1.printf3('IG={:.0f},{:.0f},{:.0f} CL={:.0f},{:.0f}'.format(
      self._vl(cp, "E_EMS11", "IG_Reactive_Stat", 0),
      self._vl(cp, "E_EMS11", "Gear_Change", 0),
      self._vl(cp, "E_EMS11", "Engine_Run", 0),
      self._vl(cp, "E_EMS11", "Cruise_Limit_Status", 0),
      self._vl(cp, "E_EMS11", "Cruise_Limit_Target", 0),

    ))


  # ----------------------------
  # Public API
  # ----------------------------
  def set_cruise_speed(self, set_speed_kph: float):
    """CruiseButtonCtrl에서 호출하는 setter (존재 가드용)"""
    self.cruise_set_speed_kph = float(set_speed_kph)


  # ----------------------------
  # Update (entry point from CarState)
  # ----------------------------
  def update(self, ret, CS, cp, cp_cruise, cp_cam):
    # SubMaster는 여기서만 업데이트
    self._update_from_submaster()


    # Longitudinal split (OP long on/off)
    self._update_longitudinal_mode_and_speed(ret, cp, cp_cruise)


    # 원본 프레임 보관
    self.lfahda = copy.copy(self._safe_vl_map(cp_cam, "LFAHDA_MFC"))
    self.mdps12 = copy.copy(self._safe_vl_map(cp, "MDPS12"))

    # 기타 값 갱신
    self._update_vehicle_general_states(ret, cp, cp_cam)



    # LFA/Engage 관리
    self.lfa_engage(ret)
    self._auto_lane_change(ret)

    # 디버그 전송
    self._send_debug(ret, cp)
