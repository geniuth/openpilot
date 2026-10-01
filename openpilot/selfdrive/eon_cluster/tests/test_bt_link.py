"""bt_link 의 링크 유지 동작을 로컬 socketpair 로 확인한다. 실제 블루투스는 쓰지 않는다.

기기에는 pytest 가 없어서 `python test_bt_link.py` 로도 돌 수 있게 해 둔다.
"""
import contextlib
import socket
import struct
import time

from openpilot.selfdrive.eon_cluster import bt_link


class Recorder:
  def __init__(self):
    self.events = []

  def event(self, name, **kwargs):
    self.events.append((name, kwargs))

  def names(self):
    return [name for name, _ in self.events]

  def last(self, name):
    return next(kw for n, kw in reversed(self.events) if n == name)


@contextlib.contextmanager
def patched(obj, **attrs):
  old = {k: getattr(obj, k) for k in attrs}
  for k, v in attrs.items():
    setattr(obj, k, v)
  try:
    yield
  finally:
    for k, v in old.items():
      setattr(obj, k, v)


class PairLink(bt_link.BluetoothLink):
  """_dial 이 블루투스 대신 socketpair 한쪽을 돌려준다."""

  def __init__(self, small_buffers=False):
    self.peers = []
    self.dials = 0
    self.small_buffers = small_buffers
    super().__init__("00:11:22:33:44:55")

  def _dial(self):
    self.dials += 1
    ours, theirs = socket.socketpair()
    if self.small_buffers:
      for s in (ours, theirs):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    ours.settimeout(bt_link.SEND_TIMEOUT_S)
    self.peers.append(theirs)
    return ours, 5, self.dials == 1


def stop(link):
  # 스레드가 끝나기 전에 다음 테스트로 넘어가면 그 스레드의 기록이 다음 테스트의
  # Recorder 로 들어간다(cloudlog 를 모듈 단위로 바꿔 끼우기 때문).
  link.close()
  link._thread.join(timeout=5.0)


def wait_for(cond, timeout=3.0):
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    if cond():
      return True
    time.sleep(0.02)
  return False


def read_frames(sock, timeout=2.0):
  sock.settimeout(timeout)
  buf = b""
  frames = []
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    try:
      chunk = sock.recv(65536)
    except socket.timeout:
      break
    if not chunk:
      break
    buf += chunk
    while len(buf) >= 6:
      type_, length = struct.unpack(">HI", buf[:6])
      if len(buf) < 6 + length:
        break
      frames.append((type_, buf[6:6 + length]))
      buf = buf[6 + length:]
    if any(t == bt_link.TYPE_PING for t, _ in frames):
      break
  return frames


def test_sends_telemetry_and_periodic_ping_while_telemetry_flows():
  rec = Recorder()
  with patched(bt_link, cloudlog=rec, PING_INTERVAL_S=0.3):
    link = PairLink()
    try:
      assert wait_for(lambda: link.connected)
      # 텔레메트리가 계속 흘러도 PING 은 주기마다 나가야 한다.
      for _ in range(10):
        link.send(b'{"speed":1}')
        time.sleep(0.05)
      frames = read_frames(link.peers[0])
      types = [t for t, _ in frames]
      assert bt_link.TYPE_TELEMETRY in types
      assert bt_link.TYPE_PING in types
      assert (bt_link.TYPE_TELEMETRY, b'{"speed":1}') in frames
      connected = rec.last("hud_bt_connected")
      assert connected["channel"] == 5 and connected["sdp"] is True
    finally:
      stop(link)


def test_drops_frames_while_blocked_then_reports_stall():
  rec = Recorder()
  with patched(bt_link, cloudlog=rec, WRITABLE_WAIT_S=0.02, STALL_LIMIT_S=1.0):
    link = PairLink(small_buffers=True)
    try:
      assert wait_for(lambda: link.connected)
      # 상대가 읽지 않으니 버퍼가 찬다. 곧바로 끊지 말고 프레임만 버려야 한다.
      # 막힘은 보낼 것이 있을 때 판정하므로 실제처럼 계속 흘려 넣는다.
      blob = b"x" * 2000
      started = time.monotonic()
      while time.monotonic() - started < 0.5:
        link.send(blob)
        time.sleep(0.02)
      assert link.connected, "버퍼가 막혔다고 바로 끊으면 안 된다"
      while "hud_bt_disconnected" not in rec.names() and time.monotonic() - started < 5.0:
        link.send(blob)
        time.sleep(0.05)
      assert "hud_bt_disconnected" in rec.names()
      gone = rec.last("hud_bt_disconnected")
      assert gone["reason"].startswith("stalled")
      assert gone["dropped"] > 0
    finally:
      stop(link)


def test_inactive_drops_link_and_stops_dialing_after_grace():
  rec = Recorder()
  with patched(bt_link, cloudlog=rec, INACTIVE_GRACE_S=0.3):
    link = PairLink()
    try:
      assert wait_for(lambda: link.connected)
      link.set_active(False)
      time.sleep(0.1)
      assert link.connected, "점화가 잠깐 꺼진 정도로는 끊지 않는다"
      assert wait_for(lambda: not link.connected, timeout=2.0)
      assert rec.last("hud_bt_disconnected")["reason"] == "inactive"
      dials = link.dials
      time.sleep(1.0)
      assert link.dials == dials, "시동이 꺼져 있는 동안은 찾지 않는다"
      link.set_active(True)
      assert wait_for(lambda: link.connected)
      assert link.dials == dials + 1
    finally:
      stop(link)


class FakeRfcomm:
  fail = False

  def __init__(self, *args):
    pass

  def settimeout(self, _):
    pass

  def connect(self, _addr):
    if FakeRfcomm.fail:
      raise OSError("refused")

  def close(self):
    pass


class DialOnly(bt_link.BluetoothLink):
  def _run(self):
    pass


def test_dial_reuses_cached_channel_and_forgets_it_after_failure():
  calls = []

  def service_channel(mac):
    calls.append(mac)
    return 7

  with patched(bt_link, _service_channel=service_channel, cloudlog=Recorder()), \
       patched(bt_link.socket, socket=FakeRfcomm):
    link = DialOnly("AA:BB:CC:DD:EE:FF")
    FakeRfcomm.fail = False
    assert link._dial()[1:] == (7, True)
    assert link._dial()[1:] == (7, False)
    assert len(calls) == 1, "채널을 알면 SDP 를 다시 하지 않는다"
    FakeRfcomm.fail = True
    assert link._dial() is None
    FakeRfcomm.fail = False
    assert link._dial()[1:] == (7, True)
    assert len(calls) == 2, "접속에 실패한 채널은 잊고 SDP 를 다시 한다"


if __name__ == "__main__":
  for name, fn in list(globals().items()):
    if name.startswith("test_") and callable(fn):
      fn()
      print("ok", name)
