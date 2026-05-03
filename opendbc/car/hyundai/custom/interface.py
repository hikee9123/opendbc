from cereal import car
from openpilot.common.params import Params
from opendbc.car import get_safety_config





def get_params( ret, candidate ):
  params = Params()
  disengage_on_accelerator = params.get_bool("DisengageOnAccelerator")

  if not disengage_on_accelerator:
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity)]
    return True

  return False
