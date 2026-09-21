"""외부 HUD 로 가는 블루투스 RFCOMM 전송.

Navdy 는 WiFi 유저스페이스가 펌웨어에서 빠져 있어(wpa_supplicant/hostapd 없음)
UDP 로는 닿을 수 없다. 대신 기기가 안드로이드이고 루팅돼 있어 우리 앱을 올릴 수
있으므로, 그 앱이 RFCOMM 서버로 듣고 콤마가 클라이언트로 붙는다.

프레임 형식은 Navdy 자체 프로토콜과 같은 모양으로 맞춰 둔다(빅엔디안):

    [타입 2B][길이 4B][내용]

    1  텔레메트리 JSON (remote_hud._packet 결과 그대로)
    2  PING

10Hz 송신 루프가 블루투스 지연에 막히면 안 되므로 실제 전송은 별도 스레드에서
하고, 큐에는 최신 패킷 하나만 남긴다(오래된 주행 상태를 뒤늦게 그려봐야 쓸모없다).
"""
import json
import os
import queue
import socket
import struct
import subprocess
import threading
import time

# 우리 앱이 등록하는 서비스 UUID. 안드로이드 쪽과 반드시 같아야 한다.
SERVICE_UUID = "b8949674-c91b-4c36-a9d0-c24c644826a0"
SERVICE_NAME = "CommaHUD"

TYPE_TELEMETRY = 1
TYPE_PING = 2

BTPROTO_RFCOMM = 3
PING_INTERVAL_S = 5.0
RECONNECT_MIN_S = 2.0
RECONNECT_MAX_S = 30.0
CONNECT_TIMEOUT_S = 12.0
SEND_TIMEOUT_S = 3.0


def _bonded_devices():
  """본딩된 기기 MAC 목록. bluetoothctl 출력이 유일하게 안정적인 경로다."""
  try:
    out = subprocess.run(["bluetoothctl", "devices", "Paired"],
                         capture_output=True, text=True, timeout=5).stdout
  except (OSError, subprocess.SubprocessError):
    return []
  macs = []
  for line in out.splitlines():
    parts = line.split()
    if len(parts) >= 2 and parts[0] == "Device" and len(parts[1]) == 17:
      macs.append(parts[1])
  return macs


def _service_channel(mac):
  """SDP 로 우리 서비스의 RFCOMM 채널을 찾는다. 없으면 None.

  sdptool 의 --uuid 는 128비트 UUID 문자열을 받지 않는다("Invalid uuid").
  전체 레코드를 훑어 UUID 128 줄이 우리 것인 블록의 Channel 을 쓴다.
  """
  try:
    out = subprocess.run(["sdptool", "browse", mac],
                         capture_output=True, text=True, timeout=40).stdout
  except (OSError, subprocess.SubprocessError):
    return None
  ours = False
  for line in out.splitlines():
    line = line.strip()
    if line.startswith("Service Name:"):
      # 새 레코드 시작. 이름으로도 한 번 걸러 둔다.
      ours = line.split(":", 1)[1].strip() == SERVICE_NAME
    elif line.lower().startswith("uuid 128:"):
      ours = line.split(":", 1)[1].strip().lower() == SERVICE_UUID
    elif line.startswith("Channel:") and ours:
      try:
        return int(line.split(":", 1)[1])
      except ValueError:
        return None
  return None


# HUD 렌더러가 실제로 읽는 항목만. 원본에는 계기판용 필드(문·타이어공기압·
# 주차센서·날씨 등)가 많은데 이 화면에는 안 그린다.
# "drive"/"active"/"turnType"/"remainDist"/"remainTime" 는 최상위에 없는 키였다.
# navi 안에 중첩돼 있어서 아무것도 안 나갔다. 최상위에 실제로 있는 이름으로 바꾼다.
KEEP_KEYS = (
  "speed", "limit", "set", "gap", "gear", "alert",
  # 인게이지 여부. 경로 띠 색이 여기서 갈린다.
  "enabled",
  "lanes", "edges", "path", "lead", "lead2", "others", "laneL", "laneR",
  "leftBsd", "rightBsd", "leftBlinker", "rightBlinker",
  "camera", "cameraDist", "cameraSection", "bumpDist",
  "vTurnSpeed", "turnInfo", "turnDist", "desiredSpeed",
)
# 폴리라인 점 개수. 640x480 에서는 33점이나 13점이나 같은 그림이 나온다.
LINE_POINTS = 13
# 이보다 가까운 점은 HUD 투영에서 화면 밖으로 나가 그릴 수 없다.
# 앱의 Projection.MIN_X 와 맞춰 둔다.
MIN_DRAW_X = 2.0
# 이보다 멀면 소실점에 뭉쳐 한 점이 된다.
MAX_DRAW_X = 90.0


def _thin(points, count=LINE_POINTS):
  """보이는 구간만 남기고 솎는다. 가까운 쪽을 촘촘히 둔다.

  modelV2 의 x 간격은 앞쪽이 아주 촘촘하고(0, 0.19, 0.75, 1.69) 뒤로 갈수록
  벌어진다. 인덱스 기준으로 솎으면 그 촘촘한 앞부분만 챙기다가 정작 화면에
  보이는 3~7m 구간을 통째로 건너뛴다(실측: 1.69 다음이 6.75 로 점프해서
  화면 아래 절반이 비었다). 그래서 안 보일 점을 먼저 버리고 솎는다.
  """
  if not points:
    return points
  usable = [p for p in points if MIN_DRAW_X <= p[0] <= MAX_DRAW_X]
  if len(usable) <= count:
    return usable
  last = len(usable) - 1
  idx = sorted({int(round(last * (i / float(count - 1)) ** 1.6)) for i in range(count)})
  return [usable[i] for i in idx]


def slim_packet(packet: dict) -> dict:
  out = {k: packet[k] for k in KEEP_KEYS if k in packet}
  for key in ("lanes", "edges"):
    lines = out.get(key)
    if isinstance(lines, list):
      out[key] = [{**ln, "p": _thin(ln.get("p"))} if isinstance(ln, dict) else ln
                  for ln in lines]
  if isinstance(out.get("path"), list):
    out["path"] = _thin(out["path"])
  return out


class BluetoothLink:
  """최신 패킷 하나만 유지하며 RFCOMM 으로 밀어 넣는다. 끊기면 스스로 다시 붙는다."""

  def __init__(self, mac=""):
    self.mac = mac
    self._queue = queue.Queue(maxsize=1)
    self._stop = threading.Event()
    self._connected = threading.Event()
    self._last_error = ""
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  @property
  def connected(self):
    return self._connected.is_set()

  @property
  def last_error(self):
    return self._last_error

  def send_packet(self, packet: dict) -> None:
    """블루투스로 보낼 만큼만 추려서 보낸다.

    UDP(WiFi) 경로는 원본을 그대로 쓰지만 RFCOMM 은 대역폭이 빠듯하다.
    실측 원본이 5,765B 라 10Hz 면 461kbps 로 RFCOMM 실효대역폭에 걸린다.
    HUD 가 실제로 그리는 것만 남기고 폴리라인을 솎으면 그 절반 이하가 된다.
    """
    self.send(json.dumps(slim_packet(packet), separators=(",", ":"),
                         ensure_ascii=False).encode("utf-8"))

  def send(self, payload: bytes) -> None:
    """가장 최근 것만 남긴다. 큐가 차 있으면 이전 것을 버린다."""
    try:
      self._queue.put_nowait(payload)
    except queue.Full:
      try:
        self._queue.get_nowait()
      except queue.Empty:
        pass
      try:
        self._queue.put_nowait(payload)
      except queue.Full:
        pass

  def close(self):
    self._stop.set()

  # ----- 내부 -----

  def _targets(self):
    if self.mac:
      return [self.mac]
    return _bonded_devices()

  def _dial(self):
    for mac in self._targets():
      channel = _service_channel(mac)
      if channel is None:
        continue
      sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, BTPROTO_RFCOMM)
      sock.settimeout(CONNECT_TIMEOUT_S)
      try:
        sock.connect((mac, channel))
      except OSError as e:
        self._last_error = f"{mac} ch{channel}: {e.__class__.__name__} {e}"
        try:
          sock.close()
        except OSError:
          pass
        continue
      sock.settimeout(SEND_TIMEOUT_S)
      self.mac = mac
      self._last_error = ""
      return sock
    return None

  @staticmethod
  def _frame(type_, payload: bytes) -> bytes:
    return struct.pack(">HI", type_, len(payload)) + payload

  def _run(self):
    backoff = RECONNECT_MIN_S
    sock = None
    last_ping = 0.0
    while not self._stop.is_set():
      if sock is None:
        sock = self._dial()
        if sock is None:
          self._connected.clear()
          self._stop.wait(backoff)
          backoff = min(RECONNECT_MAX_S, backoff * 2)
          continue
        backoff = RECONNECT_MIN_S
        self._connected.set()
        last_ping = time.monotonic()

      try:
        payload = self._queue.get(timeout=0.5)
      except queue.Empty:
        payload = None

      try:
        if payload is not None:
          sock.sendall(self._frame(TYPE_TELEMETRY, payload))
        now = time.monotonic()
        if now - last_ping >= PING_INTERVAL_S:
          sock.sendall(self._frame(TYPE_PING, b"PING"))
          last_ping = now
      except OSError as e:
        self._last_error = f"send: {e.__class__.__name__} {e}"
        self._connected.clear()
        try:
          sock.close()
        except OSError:
          pass
        sock = None

    if sock is not None:
      try:
        sock.close()
      except OSError:
        pass
    self._connected.clear()


def available() -> bool:
  """커널에 블루투스가 없는 기기(구형 AGNOS)에서는 아예 시도하지 않는다."""
  if not os.path.exists("/sys/class/bluetooth"):
    return False
  try:
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, BTPROTO_RFCOMM)
    s.close()
  except OSError:
    return False
  return True
