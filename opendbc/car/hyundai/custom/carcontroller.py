from cereal import car

from opendbc.car.hyundai.values import Buttons
from opendbc.car.hyundai    import hyundaican
from opendbc.car.hyundai.custom.cruisebuttonctrl  import CruiseButtonCtrl

import opendbc.custom.loger as trace1


class CarControllerCustom:
  def __init__(self, CP):
    self.CP = CP
    self.NC = CruiseButtonCtrl( CP)
    self.resume_cnt = 0


  def create_button_messages(self, packer, can_sends, CC: car.CarControl, CS: car.CarState, frame: int):
    #custom
    btn_signal = self.NC.update( CC, CS, frame )
    if btn_signal != None:
      can_sends.extend( [hyundaican.create_clu11( packer, self.resume_cnt, CS.clu11, btn_signal, self.CP)] * 2 )
      self.resume_cnt += 1
    else:
      self.resume_cnt = 0
