import json
import socket
from types import SimpleNamespace

from openpilot.selfdrive.carrot import nmirror_bridge
from openpilot.selfdrive.carrot.nmirror_bridge import (
  BROADCAST_PORT, CONNECTION_TIMEOUT, SDP_MESSAGE, NMirrorBridge, discovery_targets, to_carrot_traffic_light,
)
from openpilot.selfdrive.carrot.tests.test_external_navigation_priority import navigation_update  # noqa: F401

APP = ("192.168.43.1", 40000)


class _Sock:
  def __init__(self):
    self.sent = []

  def sendto(self, data, addr):
    self.sent.append((data, addr))


def _packet(**obj):
  return json.dumps(obj).encode()


def test_road_limit_is_queued_and_answered_with_sdp():
  bridge, sock = NMirrorBridge(), _Sock()
  road_limit = {"road_limit_speed": 80, "cam_type": 1}
  bridge.handle_packet(_packet(active=1, road_limit=road_limit, traffic_signal={"isRedLightOn": True}), APP, sock, 10.)

  assert bridge.connected
  assert sock.sent == [(SDP_MESSAGE, (APP[0], BROADCAST_PORT))]
  assert bridge.pop() == (road_limit, {"isRedLightOn": True})
  assert bridge.pop() is None


def test_keepalive_without_data_does_not_queue():
  bridge = NMirrorBridge()
  bridge.handle_packet(_packet(active=1), APP, _Sock(), 10.)
  assert bridge.connected
  assert bridge.pop() is None


def test_invalid_payload_is_ignored():
  bridge, sock = NMirrorBridge(), _Sock()
  bridge.handle_packet(b"\xff", APP, sock, 10.)
  bridge.handle_packet(b"[1, 2]", APP, sock, 10.)
  assert not bridge.connected and not sock.sent


def test_shell_commands_are_not_executed(monkeypatch):
  calls = []
  monkeypatch.setattr("os.system", lambda cmd: calls.append(cmd))
  monkeypatch.setattr("subprocess.run", lambda *args, **kwargs: calls.append(args))
  bridge, sock = NMirrorBridge(), _Sock()
  bridge.handle_packet(_packet(cmd="touch /tmp/x", echo_cmd="id"), APP, sock, 10.)
  assert not calls
  assert sock.sent == [(SDP_MESSAGE, (APP[0], BROADCAST_PORT))]


def test_echo_is_returned_to_app():
  bridge, sock = NMirrorBridge(), _Sock()
  bridge.handle_packet(_packet(echo={"t": 1}), APP, sock, 10.)
  assert (json.dumps({"t": 1}).encode(), (APP[0], BROADCAST_PORT)) in sock.sent


def test_gps_is_sent_only_while_requested():
  location = SimpleNamespace(latitude=37.5, longitude=127.0, altitude=30., speed=10., bearingDeg=90.,
                             horizontalAccuracy=3., unixTimestampMillis=1, verticalAccuracy=4.,
                             bearingAccuracyDeg=5., speedAccuracy=6.)
  bridge, sock = NMirrorBridge(get_location=lambda: location), _Sock()
  bridge.handle_packet(_packet(request_gps=1), APP, sock, 10.)
  sock.sent.clear()
  bridge.tick(sock, 10.5)
  assert sock.sent == [(json.dumps({"location": [37.5, 127.0, 30., 10., 90., 3., 1, 4., 5., 6.]}).encode(),
                        (APP[0], BROADCAST_PORT))]

  bridge.handle_packet(_packet(request_gps=0), APP, sock, 11.)
  sock.sent.clear()
  bridge.tick(sock, 12.)
  assert all(data == SDP_MESSAGE for data, _ in sock.sent)


def test_timeout_disconnects_and_restarts_discovery(monkeypatch):
  monkeypatch.setattr(nmirror_bridge, "discovery_targets", lambda: ["192.168.43.1", "192.168.43.2"])
  bridge, sock = NMirrorBridge(), _Sock()
  bridge.handle_packet(_packet(active=1), APP, sock, 10.)
  bridge.tick(sock, 10. + CONNECTION_TIMEOUT - 0.1)
  assert bridge.connected

  sock.sent.clear()
  bridge.tick(sock, 10. + CONNECTION_TIMEOUT + 0.1)
  assert not bridge.connected
  assert sock.sent == [(SDP_MESSAGE, ("192.168.43.1", BROADCAST_PORT)), (SDP_MESSAGE, ("192.168.43.2", BROADCAST_PORT))]


def test_discovery_targets_scan_wlan_subnet_only():
  addr = lambda address: SimpleNamespace(family=socket.AF_INET, address=address)  # noqa: E731
  targets = discovery_targets({"wlan0": [addr("192.168.43.100")], "rmnet_data0": [addr("10.20.30.40")],
                               "lo": [addr("127.0.0.1")]})
  assert len(targets) == 253
  assert "192.168.43.1" in targets and "192.168.43.254" in targets
  assert "192.168.43.100" not in targets


def test_traffic_signal_translation():
  assert to_carrot_traffic_light({"isRedLightOn": True, "redLightRemainTime": 12, "distance": 80}) == {
    "redLightOn": True, "redLightRemainTime": 12,
    "leftLightOn": False, "leftLightRemainTime": 0,
    "greenLightOn": False, "greenLightRemainTime": 0,
    "distance": 80,
  }


def test_speed_camera_controls_speed(navigation_update):  # noqa: F811
  serv, _CS, update = navigation_update
  assert serv.update_nmirror({"road_limit_speed": 80, "is_highway": False, "cam_type": 1,
                              "cam_limit_speed": 60, "cam_limit_speed_left_dist": 300,
                              "current_road_name": "세종대로"})
  result = update()
  assert (result.xSpdType, result.xSpdLimit, result.xSpdDist) == (1, 60, 300)
  assert result.desiredSource == "cam"
  assert result.nRoadLimitSpeed == 80
  assert serv.szPosRoadName == "세종대로"


def test_section_takes_primary_slot(navigation_update):  # noqa: F811
  serv, _CS, update = navigation_update
  serv.update_nmirror({"road_limit_speed": 100, "is_highway": True, "section_limit_speed": 100,
                       "section_left_dist": 2000, "cam_type": 3, "cam_limit_speed": 100,
                       "cam_limit_speed_left_dist": 2000})
  assert (serv.nSdiType, serv.nSdiBlockType, serv.nSdiPlusType) == (4, 2, 3)
  result = update()
  assert (result.xSpdType, result.xSpdLimit, result.xSpdDist) == (4, 100, 2000)
  assert (result.activeCarrot, result.desiredSource, result.desiredSpeed) == (4, "section", 100)


def test_speed_bump_only_off_highway(navigation_update):  # noqa: F811
  serv, _CS, _update = navigation_update
  bump = {"road_limit_speed": 30, "cam_type": 22, "cam_limit_speed_left_dist": 50}
  serv.update_nmirror({**bump, "is_highway": False})
  assert (serv.xSpdType, serv.xSpdDist) == (22, 50)
  serv.update_nmirror({**bump, "is_highway": True})
  assert serv.xSpdType == -1


def test_camera_clears_when_app_drops_it(navigation_update):  # noqa: F811
  serv, _CS, update = navigation_update
  serv.update_nmirror({"road_limit_speed": 80, "cam_type": 1, "cam_limit_speed": 60, "cam_limit_speed_left_dist": 300})
  serv.update_nmirror({"road_limit_speed": 80})
  result = update()
  assert result.xSpdType == -1
  assert result.activeCarrot == 2


def test_connection_expires_without_packets(navigation_update):  # noqa: F811
  serv, _CS, update = navigation_update
  serv.update_nmirror({"road_limit_speed": 80})
  for _ in range(nmirror_bridge_ticks()):
    result = update()
  assert result.activeCarrot == 0


def nmirror_bridge_ticks():
  from openpilot.selfdrive.carrot.carrot_serv import NMIRROR_ACTIVE_TICKS
  return NMIRROR_ACTIVE_TICKS


def test_carrot_navi_has_priority(navigation_update):  # noqa: F811
  serv, _CS, _update = navigation_update
  serv.carrot_navi_active = serv.carrot_navi_has_control = True
  serv.nRoadLimitSpeed = 50
  assert not serv.update_nmirror({"road_limit_speed": 80})
  assert serv.nRoadLimitSpeed == 50
