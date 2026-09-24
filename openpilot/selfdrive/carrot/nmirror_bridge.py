"""nmirror2 앱 수신기 (neokii ROAD_LIMIT_SERVICE 프로토콜).

npilot 의 selfdrive/controls/neokii/navi_controller.py 와 같은 방식으로 앱과 붙는다.
  1. 연결 전에는 5초마다 wlan 서브넷의 모든 호스트 UDP 2899 로 SDP 문자열을 보낸다.
  2. 앱은 UDP 3843 으로 {"active", "road_limit", "traffic_signal", ...} JSON 을 보낸다.
  3. 받을 때마다, 그리고 1초마다 앱의 2899 로 SDP 를 돌려줘 연결을 유지한다.
  4. 앱이 request_gps=1 을 보내면 기기 GPS 를 3Hz 로 앱의 2899 에 보낸다.

앱이 보내는 cmd/echo_cmd(셸 실행)는 같은 LAN 의 누구나 기기에서 명령을 실행할 수 있게
되므로 받지 않는다.

소켓 스레드는 받은 값을 보관만 하고, CarrotServ 반영은 carrot_man 의 메인 루프가
pop() 으로 가져가서 한다. CarrotServ 상태를 한 스레드에서만 바꾸기 위해서다.

받은 원본은 값이 바뀔 때만 cloudlog 로 남긴다. logMessage 로 rlog 에 들어가고
(주행 중), 주차 중에도 swaglog 파일에 남는다. 앱은 같은 값을 1Hz 로 되풀이한다.
"""
import ipaddress
import json
import select
import socket
import threading
import time

import psutil

from openpilot.common.swaglog import cloudlog

SDP_MESSAGE = b"EON:ROAD_LIMIT_SERVICE:v1"
BROADCAST_PORT = 2899  # 기기 -> 앱: SDP, echo, GPS
RECEIVE_PORT = 3843    # 앱 -> 기기: JSON

DISCOVERY_INTERVAL = 5.0
SDP_INTERVAL = 1.0
GPS_INTERVAL = 1. / 3.
CONNECTION_TIMEOUT = 6.0  # npilot 과 같은 값
DISCOVERY_IFACE_PREFIXES = ("wlan", "br")


def discovery_targets(if_addrs=None):
  """wlan 계열 인터페이스가 속한 /24 의 호스트 주소들 (자기 자신 제외)."""
  if if_addrs is None:
    if_addrs = psutil.net_if_addrs()
  targets = []
  for name, addresses in if_addrs.items():
    if not name.startswith(DISCOVERY_IFACE_PREFIXES):
      continue
    for addr in addresses:
      if addr.family != socket.AF_INET or not addr.address or addr.address.startswith("127."):
        continue
      # 핫스팟은 보통 /24 다. 더 넓은 망이어도 자기 주변 /24 만 훑는다.
      network = ipaddress.ip_network(f"{addr.address}/24", strict=False)
      targets.extend(str(host) for host in network.hosts() if str(host) != addr.address)
  return targets


def to_carrot_traffic_light(ts):
  """nmirror2 의 traffic_signal 을 CarrotMan.handle_traffic_light 입력 형식으로 바꾼다."""
  return {
    "redLightOn": bool(ts.get("isRedLightOn")),
    "redLightRemainTime": ts.get("redLightRemainTime", 0),
    "leftLightOn": bool(ts.get("isLeftLightOn")),
    "leftLightRemainTime": ts.get("leftLightRemainTime", 0),
    "greenLightOn": bool(ts.get("isGreenLightOn")),
    "greenLightRemainTime": ts.get("greenLightRemainTime", 0),
    "distance": ts.get("distance", 0),
  }


def location_payload(location):
  # npilot navi_controller.gps_timer 와 같은 배열 순서
  return json.dumps({"location": [
    location.latitude,
    location.longitude,
    location.altitude,
    location.speed,
    location.bearingDeg,
    location.horizontalAccuracy,
    location.unixTimestampMillis,
    location.verticalAccuracy,
    location.bearingAccuracyDeg,
    location.speedAccuracy,
  ]}).encode()


class NMirrorBridge:
  def __init__(self, get_location=None):
    self.get_location = get_location  # () -> GpsLocationData | None
    self.lock = threading.Lock()
    self.remote_addr = None
    self.gps_addr = None
    self.last_recv = 0.0
    self.last_sdp = 0.0
    self.last_discovery = 0.0
    self.last_gps = 0.0
    self._road_limit = None
    self._traffic_signal = None
    self._ignored_cmd_logged = False
    self._logged_road_limit = None
    self._logged_traffic_signal = None

  @property
  def connected(self):
    return self.remote_addr is not None

  def pop(self):
    """마지막 pop 이후 새로 받은 (road_limit, traffic_signal). 둘 다 없으면 None."""
    with self.lock:
      if self._road_limit is None and self._traffic_signal is None:
        return None
      packet = (self._road_limit, self._traffic_signal)
      self._road_limit = self._traffic_signal = None
      return packet

  def handle_packet(self, data, addr, sock, now):
    try:
      obj = json.loads(data.decode())
    except (UnicodeDecodeError, ValueError):
      return
    if not isinstance(obj, dict):
      return

    if self.remote_addr is None or self.remote_addr[0] != addr[0]:
      print(f"[nmirror] connected: {addr[0]}")
      cloudlog.event("nmirror_connected", remote=addr[0])
    self.remote_addr = addr
    self.last_recv = now

    if ("cmd" in obj or "echo_cmd" in obj) and not self._ignored_cmd_logged:
      print("[nmirror] ignoring cmd/echo_cmd (shell execution is not supported)")
      self._ignored_cmd_logged = True

    if "request_gps" in obj:
      self.gps_addr = addr if obj.get("request_gps") == 1 else None

    if "echo" in obj:
      self._sendto(sock, json.dumps(obj["echo"]).encode(), addr[0])

    road_limit = obj.get("road_limit")
    traffic_signal = obj.get("traffic_signal")
    if isinstance(road_limit, dict) and road_limit != self._logged_road_limit:
      self._logged_road_limit = road_limit
      cloudlog.event("nmirror_road_limit", active=obj.get("active"), road_limit=road_limit)
    if isinstance(traffic_signal, dict) and traffic_signal != self._logged_traffic_signal:
      self._logged_traffic_signal = traffic_signal
      cloudlog.event("nmirror_traffic_signal", traffic_signal=traffic_signal)
    with self.lock:
      if isinstance(road_limit, dict):
        self._road_limit = road_limit
      if isinstance(traffic_signal, dict):
        self._traffic_signal = traffic_signal

    self._send_sdp(sock, now)

  def tick(self, sock, now):
    if self.remote_addr is not None and now - self.last_recv > CONNECTION_TIMEOUT:
      print(f"[nmirror] disconnected: {self.remote_addr[0]}")
      cloudlog.event("nmirror_disconnected", remote=self.remote_addr[0])
      self.remote_addr = self.gps_addr = None
      self._logged_road_limit = self._logged_traffic_signal = None  # 재연결 첫 값은 다시 남긴다

    if self.remote_addr is None:
      if now - self.last_discovery >= DISCOVERY_INTERVAL:
        self.last_discovery = now
        for ip in discovery_targets():
          self._sendto(sock, SDP_MESSAGE, ip)
      return

    if now - self.last_sdp >= SDP_INTERVAL:
      self._send_sdp(sock, now)

    if self.gps_addr is not None and self.get_location is not None and now - self.last_gps >= GPS_INTERVAL:
      self.last_gps = now
      try:
        location = self.get_location()
      except Exception:
        location = None
      if location is not None:
        self._sendto(sock, location_payload(location), self.gps_addr[0])

  def _send_sdp(self, sock, now):
    self.last_sdp = now
    self._sendto(sock, SDP_MESSAGE, self.remote_addr[0])

  @staticmethod
  def _sendto(sock, data, ip):
    try:
      sock.sendto(data, (ip, BROADCAST_PORT))
    except OSError:
      pass

  def run(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
          sock.bind(("0.0.0.0", RECEIVE_PORT))
          print(f"[nmirror] listening on UDP {RECEIVE_PORT}")
          while True:
            ready, _, _ = select.select([sock], [], [], 0.1)
            if ready:
              data, addr = sock.recvfrom(4096)
              self.handle_packet(data, addr, sock, time.monotonic())
            self.tick(sock, time.monotonic())
      except Exception as e:
        print(f"[nmirror] socket error, retrying: {e}")
        self.remote_addr = self.gps_addr = None
        time.sleep(2)
