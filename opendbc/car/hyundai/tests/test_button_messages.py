from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.carcontroller import CarController, VisualAlert
from opendbc.car.hyundai.custom.carcontroller import CarControllerCustom
from opendbc.car.hyundai.values import Buttons


class TestButtonMessages(unittest.TestCase):
  def test_resume_queues_one_clu11_frame(self):
    controller = CarController.__new__(CarController)
    controller.CP = SimpleNamespace(openpilotLongitudinalControl=False, flags=0)
    controller.hyundai_community = True
    controller.car_fingerprint = "test"
    controller.frame = 20
    controller.last_button_frame = 0
    controller.cancel_counter = 0
    controller.packer = object()
    controller.customCC = SimpleNamespace(NC=SimpleNamespace(reset=lambda: None))

    hud = SimpleNamespace(visualAlert=VisualAlert.none, leftLaneVisible=False, rightLaneVisible=False,
                          leftLaneDepart=False, rightLaneDepart=False)
    control = SimpleNamespace(enabled=False, hudControl=hud,
                               cruiseControl=SimpleNamespace(cancel=False, resume=True))
    car_state = SimpleNamespace(lkas11={}, clu11={}, customCS=SimpleNamespace(mdps12={}),
                                out=SimpleNamespace(cruiseState=SimpleNamespace(available=True)))

    with patch.object(hyundaican, "create_lkas11", return_value=("LKAS11",)), \
         patch.object(hyundaican, "create_mdps12", return_value=("MDPS12",)), \
         patch.object(hyundaican, "create_clu11", return_value=("CLU11",)):
      messages = controller.create_can_msgs(False, 0, False, 0, 0, False, hud, None, car_state, control)
      controller.frame += 1
      next_frame_messages = controller.create_can_msgs(False, 0, False, 0, 0, False, hud, None, car_state, control)

    self.assertEqual(messages.count(("CLU11",)), 1)
    self.assertEqual(next_frame_messages.count(("CLU11",)), 0)

  def test_community_acc_main_off_emits_no_can_messages(self):
    controller = CarController.__new__(CarController)
    controller.CP = SimpleNamespace(openpilotLongitudinalControl=False)
    controller.hyundai_community = True
    car_state = SimpleNamespace(out=SimpleNamespace(cruiseState=SimpleNamespace(available=False)))

    messages = controller.create_can_msgs(False, 0, False, 0, 0, False, None, None, car_state, None)

    self.assertEqual(messages, [])

  def test_community_button_queues_one_clu11_frame(self):
    controller = CarControllerCustom(None)
    controller.NC = SimpleNamespace(update=lambda *_: Buttons.RES_ACCEL)
    messages = []

    with patch.object(hyundaican, "create_clu11", return_value=("CLU11",)):
      controller.create_button_messages(object(), messages, None, SimpleNamespace(clu11={}), 1)

    self.assertEqual(messages, [("CLU11",)])


if __name__ == "__main__":
  unittest.main()
