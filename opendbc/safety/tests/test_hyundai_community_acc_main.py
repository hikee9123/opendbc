import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import make_msg
from opendbc.safety.tests.libsafety import libsafety_py


class TestHyundaiCommunityAccMainState(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCommunity, 0)
    self.safety.init_tests()

  @staticmethod
  def _scc11(main_on, bus=0):
    data = bytes([int(main_on), 0, 0, 0, 0, 0, 0, 0])
    return make_msg(bus, 0x420, 8, data)

  @staticmethod
  def _lkas11():
    return make_msg(0, 0x340, 8)

  def test_initial_and_main_off_state_blocks_host_commands_but_forwards_stock(self):
    self.safety.set_controls_allowed(True)

    # Until an ACC-main-on frame is received, remain fail-safe in stock passthrough.
    self.assertFalse(self.safety.safety_tx_hook(self._lkas11()))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x251))

    self.assertTrue(self.safety.safety_rx_hook(self._scc11(main_on=False)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self._lkas11()))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x251))

  def test_main_on_restores_normal_community_forwarding(self):
    self.assertTrue(self.safety.safety_rx_hook(self._scc11(main_on=True)))

    # ACC-main-on selects normal OP forwarding; safety still controls whether
    # steering is allowed based on the normal button/engagement state.
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x251))

  def test_camera_scc_tracks_main_state_only_on_bus_two(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCommunity, HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()

    self.assertFalse(self.safety.safety_rx_hook(self._scc11(main_on=True, bus=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x251))

    self.assertTrue(self.safety.safety_rx_hook(self._scc11(main_on=True, bus=2)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x251))


if __name__ == "__main__":
  unittest.main()
