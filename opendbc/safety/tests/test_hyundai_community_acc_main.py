import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum


class TestHyundaiCommunityAccMainState(unittest.TestCase):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_kia_generic")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCommunity, 0)
    self.safety.init_tests()

  @staticmethod
  def _scc11(main_on, bus=0):
    data = bytes([int(main_on), 0, 0, 0, 0, 0, 0, 0])
    return make_msg(bus, 0x420, 8, data)

  @staticmethod
  def _lkas11():
    return CANPackerSafety("hyundai_kia_generic").make_can_msg_safety(
      "LKAS11", 0, {"CR_Lkas_StrToqReq": 0, "CF_Lkas_ActToi": 0})

  def test_initial_and_main_off_state_blocks_host_commands_but_forwards_stock(self):
    self.safety.set_controls_allowed(True)

    # Until an ACC-main-on frame is received, remain fail-safe in stock passthrough.
    self.assertFalse(self.safety.safety_tx_hook(self._lkas11()))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x251))

    self.assertTrue(self.safety.safety_rx_hook(self._scc11(main_on=False)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self._lkas11()))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x251))

  def test_main_on_reenables_host_tx(self):
    self.assertTrue(self.safety.safety_rx_hook(self._scc11(main_on=True)))

    self.assertTrue(self.safety.safety_tx_hook(self._lkas11()))

  def test_camera_scc_tracks_main_state_only_on_bus_two(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCommunity, HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()

    self.assertFalse(self.safety.safety_rx_hook(self._scc11(main_on=True, bus=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x251))

    self.assertTrue(self.safety.safety_rx_hook(self._scc11(main_on=True, bus=2)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x251))

  def test_missing_scc11_does_not_invalidate_required_rx_health(self):
    self.safety.set_timer(2_000_000)
    required_rx = (
      self.packer.make_can_msg_safety("EMS16", 0, {"AliveCounter": 1}, fix_checksum=checksum),
      make_msg(0, 0x371, 8),
      self.packer.make_can_msg_safety("WHL_SPD11", 0, {
        "WHL_SPD_FL": 0, "WHL_SPD_FR": 0, "WHL_SPD_RL": 0, "WHL_SPD_RR": 0,
        "WHL_SPD_AliveCounter_LSB": 1, "WHL_SPD_AliveCounter_MSB": 0,
      }, fix_checksum=checksum),
      self.packer.make_can_msg_safety("TCS13", 0, {"AliveCounterTCS": 1}, fix_checksum=checksum),
      self.packer.make_can_msg_safety("MDPS12", 0, {}),
      self.packer.make_can_msg_safety("CLU11", 0, {"CF_Clu_AliveCnt1": 1}),
      self.packer.make_can_msg_safety("SCC12", 0, {"CR_VSM_Alive": 1}, fix_checksum=checksum),
    )
    for msg in required_rx:
      self.assertTrue(self.safety.safety_rx_hook(msg))

    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def test_stale_scc11_fails_closed_without_bypassing_forward_filters(self):
    self.safety.set_timer(1_000)
    self.assertTrue(self.safety.safety_rx_hook(self._scc11(main_on=True)))
    self.safety.set_controls_allowed(True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x251))

    self.safety.set_timer(1_000_001)
    self.assertFalse(self.safety.safety_tx_hook(self._lkas11()))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x251))


if __name__ == "__main__":
  unittest.main()
