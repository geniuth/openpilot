#!/usr/bin/env python3
import math
import time
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

# Constants
_LEAD_ACCEL_TAU = 1.0
TAU_GROW = 1.05
TAU_SHRINK = 0.90
TAU_MIN = 0.4
BLEND_KF = 0.2
BLEND_VREL_DERIV = 0.3
V_EGO_STATIONARY = 4.0
RADAR_TO_CAMERA = 1.52

SPEED, ACCEL = 0, 1

class KalmanParams:
  def __init__(self, dt: float):
    assert 0.01 < dt < 0.2
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]

class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = _LEAD_ACCEL_TAU
    self.kf = KF1D([[v_lead], [0.0]], kalman_params.A, kalman_params.C, kalman_params.K)

  def update(self, d_rel, y_rel, v_rel, v_lead, measured):
    self.dRel = float(d_rel)
    self.yRel = float(y_rel)
    self.vRel = float(v_rel)
    self.vLead = float(v_lead)
    self.measured = measured
    if self.cnt > 0:
      self.kf.update(self.vLead)
    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])
    self.aLeadTau = min(max(self.aLeadTau, 0.05) * TAU_GROW, _LEAD_ACCEL_TAU) if abs(self.aLeadK) < 0.5 else max(self.aLeadTau * TAU_SHRINK, TAU_MIN)
    self.cnt += 1

  def get_RadarState(self, model_prob=0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau),
      "status": True,
      "fcw": model_prob > .9,
      "modelProb": float(model_prob),
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego):
    return abs(self.yRel) < 1.0 and v_ego < V_EGO_STATIONARY and 0.75 < self.dRel < 25

def laplacian_pdf(x, mu, b):
  b = max(b, 1e-4)
  return math.exp(-abs(x - mu) / b)

def match_vision_to_track(v_ego, lead, tracks):
  offset_d = lead.x[0] - RADAR_TO_CAMERA
  def prob(track):
    return laplacian_pdf(track.dRel, offset_d, lead.xStd[0]) * \
           laplacian_pdf(track.yRel, -lead.y[0], lead.yStd[0]) * \
           laplacian_pdf(track.vRel + v_ego, lead.v[0], lead.vStd[0])
  best_track = max(tracks.values(), key=prob)
  if abs(best_track.dRel - offset_d) < max(offset_d * .25, 5.0) and \
     (abs(best_track.vRel + v_ego - lead.v[0]) < 10 or v_ego + best_track.vRel > 3):
    return best_track
  return None

def get_RadarState_from_vision(lead_msg, v_ego, model_v_ego):
  now = time.monotonic()
  dt = now - getattr(get_RadarState_from_vision, "prev_ts", now)
  get_RadarState_from_vision.prev_ts = now
  d_rel = float(lead_msg.x[0] - RADAR_TO_CAMERA)
  v_rel = float(lead_msg.v[0] - model_v_ego)
  v_rel_deriv = (d_rel - getattr(get_RadarState_from_vision, "last_d", d_rel)) / dt if dt > 1e-3 else None
  get_RadarState_from_vision.last_d = d_rel
  v_rel_pred = (1.0 - BLEND_VREL_DERIV) * v_rel + BLEND_VREL_DERIV * v_rel_deriv if v_rel_deriv else v_rel
  prev_a = getattr(get_RadarState_from_vision, "prev_aLeadK", 0.0)
  a_raw = lead_msg.a[0] if len(lead_msg.a) else 0.0
  a_blend = (1.0 - BLEND_KF) * float(a_raw) + BLEND_KF * prev_a
  get_RadarState_from_vision.prev_aLeadK = a_blend
  return {
    "dRel": d_rel,
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(v_rel_pred),
    "vLead": float(v_ego + v_rel_pred),
    "vLeadK": float(v_ego + v_rel_pred),
    "aLeadK": float(a_blend),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }

def get_custom_yrel(CP, CP_SP, lead_dict, lead_msg):
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict["yRel"] = float(-lead_msg.y[0])
  return lead_dict

def get_lead(v_ego, ready, tracks, lead_msg, model_v_ego, CP, CP_SP, low_speed_override=True):
  track = match_vision_to_track(v_ego, lead_msg, tracks) if tracks and ready and lead_msg.prob > 0.5 else None
  lead_dict = track.get_RadarState(lead_msg.prob) if track else (
                get_RadarState_from_vision(lead_msg, v_ego, model_v_ego) if ready and lead_msg.prob > 0.5 else {"status": False})
  if track and abs(track.dRel - (lead_msg.x[0] - RADAR_TO_CAMERA)) > 3.0:
    lead_dict = get_custom_yrel(CP, CP_SP, track.get_RadarState(lead_msg.prob), lead_msg)
  if low_speed_override:
    low_speed_tracks = [t for t in tracks.values() if t.potential_low_speed_lead(v_ego)]
    if low_speed_tracks:
      closest = min(low_speed_tracks, key=lambda t: t.dRel)
      if not lead_dict["status"] or closest.dRel < lead_dict["dRel"]:
        lead_dict = closest.get_RadarState()
  return lead_dict

class RadarD:
  def __init__(self, CP, CP_SP, delay=0.0):
    self.CP, self.CP_SP = CP, CP_SP
    self.kalman_params = KalmanParams(DT_MDL)
    self.tracks = {}
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL)) + 1)
    self.last_v_ego_frame = -1
    self.ready = False

  def update(self, sm, rr):
    self.ready = sm.seen['modelV2']
    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego_hist.append(sm['carState'].vEgo)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}
    self.tracks = {i: t for i, t in self.tracks.items() if i in ar_pts}

    for i, (d, y, v, m) in ar_pts.items():
      v_lead = v + self.v_ego_hist[0]
      if i not in self.tracks:
        self.tracks[i] = Track(i, v_lead, self.kalman_params)
      self.tracks[i].update(d, y, v, v_lead, m)

    model_v_ego = sm['modelV2'].velocity.x[0] if len(sm['modelV2'].velocity.x) else sm['carState'].vEgo
    leads = sm['modelV2'].leadsV3
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.valid = sm.all_checks()
    if len(leads) > 1:
      self.radar_state.leadOne = get_lead(sm['carState'].vEgo, self.ready, self.tracks, leads[0], model_v_ego, self.CP, self.CP_SP)
      self.radar_state.leadTwo = get_lead(sm['carState'].vEgo, self.ready, self.tracks, leads[1], model_v_ego, self.CP, self.CP_SP, low_speed_override=False)

  def publish(self, pm):
    pm.send("radarState", messaging.new_message("radarState", valid=True, radarState=self.radar_state))

def main():
  config_realtime_process(5, Priority.CTRL_LOW)
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got parameters")

  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])
  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while True:
    sm.update()
    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)

if __name__ == "__main__":
  main()
