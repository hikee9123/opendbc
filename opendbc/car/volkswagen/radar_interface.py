import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volkswagen.values import DBC, VolkswagenFlags

RADAR_ADDR = 0x24F  # MEB_Distance_01: 6 fused tracks emitted by the front radar at 25Hz
LANE_TYPES = ("Same_Lane", "Left_Lane", "Right_Lane")
SIGNAL_SETS = tuple(
  (f"{lane}_0{idx}_ObjectID",
   f"{lane}_0{idx}_Long_Distance",
   f"{lane}_0{idx}_Lat_Distance",
   f"{lane}_0{idx}_Rel_Velo")
  for lane in LANE_TYPES for idx in (1, 2)
)


def get_radar_can_parser(CP):
  if not (CP.flags & VolkswagenFlags.MEB) or Bus.radar not in DBC[CP.carFingerprint]:
    return None
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [("MEB_Distance_01", 25)], 2)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages: set[int] = set()
    self.trigger_msg = RADAR_ADDR
    self._track_id = 0
    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)
    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
      self.updated_messages.clear()
      return ret

    msg = self.rcp.vl["MEB_Distance_01"]
    active: dict[int, tuple[float, float, float]] = {}
    for obj_id_sig, long_sig, lat_sig, vel_sig in SIGNAL_SETS:
      obj_id = int(msg[obj_id_sig])
      if obj_id == 0:
        continue
      # MEB_Distance_01 publishes the same fused object on multiple lane slots
      # (e.g. Same_Lane_01 and Same_Lane_02 holding the lead and the next vehicle).
      # Keep the first occurrence; treating duplicates as a CAN error is too strict.
      if obj_id in active:
        continue
      active[obj_id] = (float(msg[long_sig]), float(msg[lat_sig]), float(msg[vel_sig]))

    for obj_id, (d_rel, y_rel, v_rel) in active.items():
      pt = self.pts.get(obj_id)
      if pt is None:
        pt = structs.RadarData.RadarPoint()
        pt.trackId = self._track_id
        self._track_id += 1
        self.pts[obj_id] = pt
      pt.measured = True
      pt.dRel = d_rel
      pt.yRel = y_rel
      pt.vRel = v_rel
      pt.aRel = math.nan
      pt.yvRel = math.nan

    for stale in self.pts.keys() - active.keys():
      del self.pts[stale]

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
