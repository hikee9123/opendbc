from enum import Enum
from typing import Callable, Optional, Dict
import numpy as np
from opendbc.car.hyundai.values import Buttons


class State(Enum):
  IDLE = "IDLE"               # 대기: 목표-설정속도 차이를 보고 가/감속으로 분기
  ACCEL = "ACCEL"             # 가속 버튼 누르는 상태
  DECEL = "DECEL"             # 감속 버튼 누르는 상태
  HOLD_NONE = "HOLD_NONE"     # 버튼 해제 유지
  STANDSTILL = "STANDSTILL"   # 정지 감지: 리드 이격 증가 모니터링
  RESUME = "RESUME"           # 정지 후 재출발: RES_ACCEL 누르기


class CruiseButtonCtrl:
  # ----------------------------
  # Tunables / Guard constants
  # ----------------------------
  MIN_SET_SPEED_KPH = 30.0
  SETPOINT_MAX_KPH = 210.0

  WAIT_PRESS_FRAMES = 5
  WAIT_NONE_FRAMES = 6
  STANDSTILL_RESUME_PRESS = 5
  IDLE_COOLDOWN_FRAMES = 100

  ACC_SAFETY_INIT = 200
  ACC_SAFETY_INIT_INACTIVE = 300

  EPS_KPH = 0.5
  LEAD_NOISE_EPS = 0.1
  VSET_MIN, VSET_MAX = 0.0, 250.0

  # 4) 전이 히스테리시스 상수 추가 및 적용
  DELTA_HYST_KPH = 0.2  # 클래스 상단 상수들 옆에 추가


  # ----------------------------
  # Lifecycle
  # ----------------------------
  def __init__(self, CP):
    self.CP = CP

    # 속도/목표
    self.target_speed: float = 0.0
    self.set_point: float = 0.0
    self.VSetDis: float = 30.0
    self._external_target_kph: Optional[float] = None

    # 버튼/상태/카운터
    self.btn_cnt: int = 0
    self._state: State = State.IDLE
    self._case_map: Dict[State, Callable[..., Optional[Buttons]]] = {
      State.IDLE:       self._case_idle,
      State.ACCEL:      self._case_acc,
      State.DECEL:      self._case_dec,
      State.HOLD_NONE:  self._case_none,
      State.STANDSTILL: self._case_standstill,
      State.RESUME:     self._case_resume,
    }


    # 타이머/보조
    self.waittime_press: int = self.WAIT_PRESS_FRAMES
    self.waittime_none: int = self.WAIT_NONE_FRAMES
    self.idle_cooldown_timer: int = 0
    self.wait_accsafety: int = 0
    self.last_lead_distance: float = 0.0

    self.initialized  = False
    self._prev_acc_active = False


  # ----------------------------
  # State handlers
  # ----------------------------
  def _case_idle(self, CS) -> Optional[Buttons]:
    """대기: 목표-설정속도 차이를 보고 가/감속/유지/정지 전이 결정"""
    self.btn_cnt = 0
    self.target_speed = float(np.clip(self.set_point, self.MIN_SET_SPEED_KPH, self.SETPOINT_MAX_KPH))
    self._refresh_vset(CS)

    delta = self.target_speed - self.VSetDis
    standstill = bool(CS.out.cruiseState.standstill)

    if not self._is_acc_on(CS):
      return self._goto(State.HOLD_NONE)

    if standstill:
      self.last_lead_distance = 0.0
      return self._goto(State.STANDSTILL)

    if CS.out.gasPressed:
      # 운전자 가속 → 현재속도로 재설정 후 유지
      try:
        clu_v = float(CS.customCS.clu_Vanz)
      except (AttributeError, TypeError, ValueError):
        clu_v = self.VSetDis
      if clu_v - self.VSetDis > (5.0 + self.EPS_KPH):
        if hasattr(CS.customCS, "set_cruise_speed"):
          CS.customCS.set_cruise_speed(clu_v)
          return self._goto(State.DECEL)
      return None

    if delta >= (1.0 + self.EPS_KPH + self.DELTA_HYST_KPH):
      return self._goto(State.ACCEL)
    if delta <= (-1.0 - self.EPS_KPH - self.DELTA_HYST_KPH):
      return self._goto(State.DECEL)
    return None

  def _case_acc(self, CS) -> Optional[Buttons]:
    """가속 버튼 누르기"""
    self.btn_cnt += 1
    if self._reached_target() or self.btn_cnt > self.waittime_press:
      self.btn_cnt = 0
      return self._goto(State.HOLD_NONE)
    return Buttons.RES_ACCEL

  def _case_dec(self, CS) -> Optional[Buttons]:
    """감속 버튼 누르기"""
    self.btn_cnt += 1
    if self._reached_target() or self.btn_cnt > self.waittime_press:
      self.btn_cnt = 0
      return self._goto(State.HOLD_NONE)
    return Buttons.SET_DECEL

  def _case_none(self, CS) -> Optional[Buttons]:
    """버튼 해제 유지"""
    self.btn_cnt += 1
    if self.btn_cnt > self.waittime_none:
      return self._goto(State.IDLE)
    return None

  def _case_standstill(self, CS) -> Optional[Buttons]:
    """정지 상태: 선행 이격 증가 감지 시 RESUME 준비"""
    if not bool(CS.out.cruiseState.standstill):
      return self._goto(State.IDLE)

    lead = getattr(CS.customCS, "lead_distance", None)
    if lead is None or lead <= 5.0:
      self.last_lead_distance = 0.0
      return None

    if self.last_lead_distance == 0.0:
      self.last_lead_distance = float(lead)
      return None

    if lead > self.last_lead_distance + self.LEAD_NOISE_EPS:
      self.btn_cnt = 0
      return self._goto(State.RESUME)
    return None

  def _case_resume(self, CS) -> Optional[Buttons]:
    """정지 후 재출발: RES_ACCEL N프레임"""
    if (not self._is_acc_on(CS)) or (self._last_button(CS) != Buttons.NONE) or CS.out.brakePressed:
      self.btn_cnt = 0
      return self._goto(State.IDLE)

    self.btn_cnt += 1
    if self.btn_cnt > self.STANDSTILL_RESUME_PRESS:
      self.btn_cnt = 0
      return self._goto(State.HOLD_NONE)
    return Buttons.RES_ACCEL

  def _try_enable_acc(self, CS) -> Optional[Buttons]:
    """
    ACC가 꺼진 상태에서 가속 페달 입력으로 ACC를 켜는 동작을 처리.
    차량마다 ACC on 버튼은 다르지만, 일반적으로 RES_ACCEL을 사용.
    """
    if CS.customCS.autoEngage <= 30:
      return None
    elif CS.customCS._cencel_button:
      return None
    elif CS.customCS.cruiseGap != CS.customCS.gapSet:
      return None
    elif CS.customCS.clu_Vanz <= CS.customCS.autoEngage: # self.MIN_SET_SPEED_KPH:
      return None

    acc_on = self._is_acc_on(CS)
    gas = bool(CS.out.gasPressed)

    # ACC가 꺼져 있고, 가속페달이 눌린 경우 → ACC 켜기
    if (not acc_on) and gas:
      # 상태 머신 초기화
      self.reset()
      self._goto(State.IDLE)

      # ACC on 버튼 신호 반환 (보통 SET_DECEL) 현재의 속도로 설정됨
      return Buttons.SET_DECEL

    return None

  # ----------------------------
  # Internal helpers
  # ----------------------------
  def _goto(self, state: State) -> None:
    """상태 전이"""
    self._state = state
    return None

  def state_name(self) -> str:
    """디버깅용 현재 상태명"""
    return self._state.value

  def _prepare_set_point(self, CS, target_kph: float) -> None:
    """외부 목표를 현재 루프에 반영(작은 노이즈 무시)"""
    tgt = round(float(target_kph), 1)
    if abs(tgt - self.set_point) < 0.1:
      # 0.1kph 미만 변화는 무시해 버튼 토글을 줄임
      self._refresh_vset(CS)
      return
    self.set_point = max(self.MIN_SET_SPEED_KPH, tgt)
    self._refresh_vset(CS)


  def _refresh_vset(self, CS) -> None:
    """차량의 현재 설정속도(VSetDis) 갱신 및 클램프"""
    try:
      vset = float(getattr(CS.customCS, "VSetDis", self.VSetDis))
    except Exception:
      vset = self.VSetDis
    self.VSetDis = float(np.clip(vset, self.VSET_MIN, self.VSET_MAX))


  def _reached_target(self) -> bool:
    """목표-설정속도 도달 판정(데드밴드)"""
    return abs(self.target_speed - self.VSetDis) <= self.EPS_KPH

  def _last_button(self, CS) -> Buttons:
    """가장 최근 크루즈 버튼 반환(없으면 NONE)"""
    try:
      return CS.cruise_buttons[-1]
    except (AttributeError, IndexError, TypeError):
      return Buttons.NONE

  def _cooldown_tick(self) -> bool:
    """쿨다운 중이면 1틱 감소하고 False, 아니면 True"""
    if self.idle_cooldown_timer > 0:
      self.idle_cooldown_timer -= 1
      return False
    return True

  def _button_idle_ok(self, CS) -> bool:
    """버튼 입력 가능한 안전 상태인지 확인. cruise_set_mode==0이면 FSM 비활성."""
    if CS.customCS.cruise_set_mode == 0:
      return False
    if (not self._is_acc_on(CS)) or (self._last_button(CS) != Buttons.NONE) or CS.out.brakePressed:
      self.idle_cooldown_timer = self.IDLE_COOLDOWN_FRAMES
      return False
    return self._cooldown_tick()

  @staticmethod
  def _is_acc_on(CS) -> bool:
    return bool(getattr(CS.customCS, "acc_active", False))



  # ----------------------------
  # Public API
  # ----------------------------
  def set_target_speed(self, kph: Optional[float]) -> None:
    """외부(맵/커브/longPlan 등)에서 계산한 목표속도 주입"""
    if kph is None:
      self._external_target_kph = None
      return
    k = round(float(kph), 1)  # 0.1 단위 반올림 → 버튼 토글 노이즈 감소
    self._external_target_kph = float(np.clip(k, self.MIN_SET_SPEED_KPH, self.SETPOINT_MAX_KPH))

  def prime(self, CS) -> None:
    """크루즈 진입 직후 초기화 (초기 튐 방지)"""
    self._refresh_vset(CS)
    self.set_point = float(np.clip(self.VSetDis, self.MIN_SET_SPEED_KPH, self.SETPOINT_MAX_KPH))
    self.btn_cnt = 0
    self._goto(State.IDLE)
    # 필요 시 진입 직후 쿨다운을 주고 싶다면 다음 줄의 주석을 해제
    # self.idle_cooldown_timer = self.IDLE_COOLDOWN_FRAMES

  def reset(self) -> None:
    """상태/타이머 리셋 (차선 변경 등 이벤트에서 호출)"""
    self.btn_cnt = 0
    self.idle_cooldown_timer = 0
    self.wait_accsafety = 0
    self.last_lead_distance = 0.0
    self._goto(State.IDLE)


  def update(self, c, CS, frame) -> Optional[Buttons]:
    """
    버튼 신호만 반환 (차량에 버튼 시뮬레이션)
    - 목표속도는 set_target_speed()로 외부 주입
    - openpilotLongitudinalControl 활성 시 버튼 조작 불필요
    """
    # openpilot long 활성 시 버튼 불필요
    if self.CP.openpilotLongitudinalControl:
      return None

    if not self.initialized:
      self.prime(CS)
      self.initialized = True
      return None


    # ACC 꺼짐
    if not self._is_acc_on(CS):
      # 가속페달로 ACC 활성화 시도
      btn = self._try_enable_acc(CS)
      if btn is not None:
        return btn
      self.wait_accsafety = self.ACC_SAFETY_INIT_INACTIVE
      return None

    # 버튼 입력 가능 상태가 아니면 리턴
    if not self._button_idle_ok(CS):
      self.wait_accsafety = self.ACC_SAFETY_INIT
      return None


    # 외부 목표 미지정
    if CS.customCS.cruiseGap == CS.customCS.gapSet:
      plan_kph = float(getattr(CS.customCS, "speed_plan_kph", 0.0))
    else:
      plan_kph = CS.customCS.cruise_set_speed_kph

    if plan_kph < self.MIN_SET_SPEED_KPH:
      plan_kph = self.MIN_SET_SPEED_KPH


    # 허용 오차로 비교 (0.05kph 정도면 충분)
    target = self._external_target_kph
    if target is None or abs(plan_kph - target) > 0.05:
      self.set_target_speed(plan_kph)
      target = self._external_target_kph
    elif target is None:
      return None


    # 상태 핸들러 실행 → 버튼 신호 산출
    self._prepare_set_point(CS, target)
    handler = self._case_map[self._state]
    btn_signal = handler(CS)

    if self.wait_accsafety > 0:
      self.wait_accsafety -= 1

    return btn_signal
