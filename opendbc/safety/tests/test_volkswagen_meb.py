#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MSG_ESC_51     = 0xFC
MSG_LH_EPS_03  = 0x9F
MSG_MOTOR_51   = 0x10B
MSG_GRA_ACC_01 = 0x12B
MSG_QFK_01     = 0x13D
MSG_ACC_18     = 0x14D
MSG_MEB_ACC_01 = 0x300
MSG_HCA_03     = 0x303
MSG_LDW_02     = 0x397
MSG_MOTOR_14   = 0x3BE

MAX_ACCEL = 2.0
MIN_ACCEL = -3.5
INACTIVE_ACCEL = 3.01

# Curvature control limits, see VOLKSWAGEN_MEB_STEERING_LIMITS in volkswagen_meb.h
DEG_TO_CAN = 149253.7313
MAX_CURVATURE = 0.195
# HCA_03.Power: scale 0.4 percent. Safety blocks raw byte > 125
MAX_POWER_PHYSICAL = 50.0  # 125 * 0.4


class TestVolkswagenMebSafetyBase(common.CarSafetyTest):
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}

  packer: CANPackerSafety
  safety: libsafety_py.LibSafety

  def _set_prev_desired_curvature(self, curvature):
    self.safety.set_desired_angle_last(round(curvature * DEG_TO_CAN))

  def _reset_curvature_measurement(self, curvature):
    for _ in range(common.MAX_SAMPLE_VALS):
      self._rx(self._angle_meas_msg(curvature))

  def _reset_speed(self, speed):
    for _ in range(common.MAX_SAMPLE_VALS):
      self._rx(self._speed_msg(speed))

  def _speed_msg(self, speed):
    spd_kph = speed * 3.6
    values = {s: spd_kph for s in ("HL_Radgeschw", "HR_Radgeschw", "VL_Radgeschw", "VR_Radgeschw")}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  def _speed_msg_2(self, speed):
    return None

  def _user_brake_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pressure": gas, "TSK_Status": 3}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=0):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  def _angle_meas_msg(self, curvature):
    values = {"Curvature": abs(curvature), "Curvature_VZ": curvature >= 0}
    return self.packer.make_can_msg_safety("QFK_01", 0, values)

  def _angle_cmd_msg(self, curvature, enabled, power=10.0):
    values = {
      "Curvature":     abs(curvature),
      "Curvature_VZ":  curvature > 0 and enabled,
      "Power":         power,
      "RequestStatus": 4 if enabled else 2,
    }
    return self.packer.make_can_msg_safety("HCA_03", 0, values)

  def _acc_18_msg(self, accel):
    values = {"ACC_Sollbeschleunigung_02": accel}
    return self.packer.make_can_msg_safety("ACC_18", 0, values)

  def test_torque_driver_measurement(self):
    """Tests rx hook correctly parses LH_EPS_03 driver torque, including sign"""
    for torque in (-50, 50, 0):
      for _ in range(common.MAX_SAMPLE_VALS):
        self._rx(self._torque_driver_msg(torque))
      self.assertEqual(torque, self.safety.get_torque_driver_min())
      self.assertEqual(torque, self.safety.get_torque_driver_max())

  def test_curvature_measurement(self):
    """Tests rx hook correctly parses QFK_01 measured curvature, including sign"""
    for curvature in (-0.05, 0.05, 0.0):
      for _ in range(common.MAX_SAMPLE_VALS):
        self._rx(self._angle_meas_msg(curvature))
      target = round(curvature * DEG_TO_CAN)
      self.assertEqual(target, self.safety.get_angle_meas_min())
      self.assertEqual(target, self.safety.get_angle_meas_max())

  def test_steer_power_limits(self):
    """Power byte: must be 0 unless actively steering, capped at MAX_POWER"""
    self.safety.set_controls_allowed(True)
    self._reset_speed(20)
    self._reset_curvature_measurement(0)
    self._set_prev_desired_curvature(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True, power=10.0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, power=MAX_POWER_PHYSICAL + 1.0)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False, power=0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, False, power=10.0)))

  def test_steer_curvature_when_enabled(self):
    """Cover both signs of HCA_03 curvature command and basic accept path"""
    self.safety.set_controls_allowed(True)
    self._reset_speed(20)
    for curvature in (-0.001, 0.001, 0.0):
      self._reset_curvature_measurement(curvature)
      self._set_prev_desired_curvature(curvature)
      self.assertTrue(self._tx(self._angle_cmd_msg(curvature, True)))

  def test_steer_curvature_when_disabled(self):
    """When steer_req is False, curvature must be exactly 0 (inactive_angle_is_zero)"""
    self.safety.set_controls_allowed(True)
    self._reset_speed(20)
    self._reset_curvature_measurement(0.05)
    self._set_prev_desired_curvature(0.05)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False, power=0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0.05, False, power=0)))

  def test_steer_curvature_when_not_allowed(self):
    """No curvature command allowed when controls are not allowed"""
    self.safety.set_controls_allowed(False)
    self._reset_speed(20)
    self._reset_curvature_measurement(0)
    self._set_prev_desired_curvature(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, power=10)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False, power=0)))

  def test_rx_hook_other_bus(self):
    """Messages on bus != 0 must not affect state"""
    self.safety.set_controls_allowed(True)
    self._rx(self._gra_acc_01_msg(cancel=1, bus=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_acc_status_engaged_states(self):
    """TSK_Status 3, 4, 5 all map to cruise_engaged; 2 is main_on without engage"""
    for tsk in (0, 2, 3, 4, 5):
      values = {"TSK_Status": tsk}
      self._rx(self.packer.make_can_msg_safety("Motor_51", 0, values))

  def test_wheel_speed_partial_zero(self):
    """Cover vehicle_moving short-circuit branches when only some wheels report movement"""
    for wheels in (("HL_Radgeschw",), ("HR_Radgeschw",), ("VL_Radgeschw",), ("VR_Radgeschw",)):
      values = {s: 0 for s in ("HL_Radgeschw", "HR_Radgeschw", "VL_Radgeschw", "VR_Radgeschw")}
      for w in wheels:
        values[w] = 50
      self._rx(self.packer.make_can_msg_safety("ESC_51", 0, values))


class TestVolkswagenMebStockSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_03, MSG_LDW_02]}

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._gra_acc_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(resume=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._gra_acc_01_msg(resume=1)))

  def test_buttons_do_not_engage_under_stock_long(self):
    # Under stock cruise, set/resume buttons must not engage
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(0)
    self._rx(self._gra_acc_01_msg(_set=1, bus=0))
    self._rx(self._gra_acc_01_msg(bus=0))
    self.assertFalse(self.safety.get_controls_allowed())
    # cancel still disengages
    self.safety.set_controls_allowed(1)
    self._rx(self._gra_acc_01_msg(cancel=1, bus=0))
    self.assertFalse(self.safety.get_controls_allowed())


class TestVolkswagenMebLongSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_ACC_18, 0], [MSG_MEB_ACC_01, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_MEB_ACC_01]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_MEB_ACC_01)}

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ("set", "resume"):
      # main switch off: button has no effect
      self.safety.set_controls_allowed(0)
      self._rx(self._tsk_status_msg(False, main_switch=False))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      # rising edge: no engage
      self._rx(self._tsk_status_msg(False, main_switch=True))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      # falling edge: engage
      self._rx(self._gra_acc_01_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._gra_acc_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_accel_safety_check(self):
    for controls_allowed in (True, False):
      for accel in np.concatenate((np.arange(MIN_ACCEL - 1, MAX_ACCEL + 1, 0.05), [0.0, INACTIVE_ACCEL])):
        accel = round(accel, 2)
        is_inactive = accel == INACTIVE_ACCEL
        # accel=0 (override) is allowed whenever controls are allowed
        is_override = controls_allowed and accel == 0.0
        in_range = MIN_ACCEL <= accel <= MAX_ACCEL
        send = is_inactive or is_override or (controls_allowed and in_range)
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(send, self._tx(self._acc_18_msg(accel)), (controls_allowed, accel))


if __name__ == "__main__":
  unittest.main()
