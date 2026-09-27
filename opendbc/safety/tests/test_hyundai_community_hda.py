import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum


class TestHyundaiCommunityHdaTransform(unittest.TestCase):
  SCC12_TIMEOUT = 100_000

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_kia_generic")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCommunity, 0)
    self.safety.init_tests()
    self.scc12_counter = 0
    self.safety.safety_rx_hook(self._scc11_msg(main_on=True))

  @staticmethod
  def _scc11_msg(main_on, bus=0):
    return make_msg(bus, 0x420, 8, bytes([int(main_on), 0, 0, 0, 0, 0, 0, 0]))

  def _scc12_msg(self, bus=0):
    values = {"ACCMode": 1, "CR_VSM_Alive": self.scc12_counter % 16}
    self.scc12_counter += 1
    return self.packer.make_can_msg_safety("SCC12", bus, values, fix_checksum=checksum)

  @staticmethod
  def _lfahda_msg(hda_active=1, hda_icon_state=1, bus=2):
    data = bytes([
      0x81 | (hda_active << 2) | (hda_icon_state << 3),
      0x57,
      0x05,  # HDA_Icon_Wheel is initially clear
      0xB0,
    ])
    return make_msg(bus, 0x485, 4, data)

  @staticmethod
  def _payload(msg):
    return bytes(msg[0].data[i] for i in range(4))

  def _set_fresh_scc12(self, bus=0, timestamp=1_000):
    self.safety.set_timer(timestamp)
    self.assertTrue(self.safety.safety_rx_hook(self._scc12_msg(bus)))

  def test_engaged_stock_frame_changes_only_wheel_bit(self):
    self.safety.set_heartbeat_engaged(True)
    self._set_fresh_scc12()

    for hda_active in (0, 1):
      for hda_icon_state in range(4):
        with self.subTest(hda_active=hda_active, hda_icon_state=hda_icon_state):
          msg = self._lfahda_msg(hda_active, hda_icon_state)
          original = self._payload(msg)
          self.safety.safety_fwd_transform(msg, 0)

          expected = original[:2] + bytes([original[2] | 0x10]) + original[3:]
          self.assertEqual(expected, self._payload(msg))

  def test_disengaged_or_stale_state_preserves_original_frame(self):
    self._set_fresh_scc12()
    msg = self._lfahda_msg()
    original = self._payload(msg)
    self.safety.safety_fwd_transform(msg, 0)
    self.assertEqual(original, self._payload(msg))

  def test_stale_scc12_preserves_original_frame(self):
    self.safety.set_heartbeat_engaged(True)
    self._set_fresh_scc12(timestamp=200_000)
    msg = self._lfahda_msg()
    original = self._payload(msg)
    self.safety.set_timer(200_000 + self.SCC12_TIMEOUT)
    self.safety.safety_fwd_transform(msg, 0)
    self.assertEqual(original, self._payload(msg))

  def test_transform_requires_recent_scc12_and_only_changes_community_mode(self):
    self.safety.set_heartbeat_engaged(True)
    msg = self._lfahda_msg()
    original = self._payload(msg)
    self.safety.safety_fwd_transform(msg, 0)
    self.assertEqual(original, self._payload(msg))

    # ACC main off keeps stock HDA frames byte-for-byte unchanged.
    self.safety.safety_rx_hook(self._scc11_msg(main_on=False))
    self.safety.set_heartbeat_engaged(True)
    self._set_fresh_scc12()
    msg = self._lfahda_msg()
    original = self._payload(msg)
    self.safety.safety_fwd_transform(msg, 0)
    self.assertEqual(original, self._payload(msg))

    self.safety.safety_rx_hook(self._scc11_msg(main_on=True))
    self._set_fresh_scc12()
    msg = self._lfahda_msg()
    self.safety.safety_fwd_transform(msg, 0)
    self.assertEqual(0x10, self._payload(msg)[2] & 0x10)

    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()
    msg = self._lfahda_msg()
    original = self._payload(msg)
    self.safety.safety_fwd_transform(msg, 0)
    self.assertEqual(original, self._payload(msg))

  def test_only_stock_camera_to_vehicle_route_is_transformed(self):
    self.safety.set_heartbeat_engaged(True)
    self._set_fresh_scc12()

    for source_bus, destination_bus in ((0, 0), (2, 2)):
      with self.subTest(source_bus=source_bus, destination_bus=destination_bus):
        msg = self._lfahda_msg(bus=source_bus)
        original = self._payload(msg)
        self.safety.safety_fwd_transform(msg, destination_bus)
        self.assertEqual(original, self._payload(msg))

  def test_camera_scc_uses_bus_two_scc12(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCommunity, HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()
    self.safety.safety_rx_hook(self._scc11_msg(main_on=True, bus=2))
    self.safety.set_heartbeat_engaged(True)
    self._set_fresh_scc12(bus=2)

    msg = self._lfahda_msg()
    self.safety.safety_fwd_transform(msg, 0)
    self.assertEqual(0x10, self._payload(msg)[2] & 0x10)

  def test_stock_frame_forwards_and_host_duplicate_is_blocked(self):
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x485))
    self.assertFalse(self.safety.safety_tx_hook(self._lfahda_msg(bus=0)))


if __name__ == "__main__":
  unittest.main()
