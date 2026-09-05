import time
from opendbc.car import get_safety_config, structs, uds
from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.radar_interface import RadarInterface
from opendbc.car.volkswagen.values import CANBUS, CAR, NetworkLocation, RADAR_DISABLE_STATE, TransmissionType, VolkswagenFlags, VolkswagenSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volkswagen"
    ret.radarUnavailable = True

    if ret.flags & VolkswagenFlags.PQ:
      # Set global PQ35/PQ46/NMS parameters
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenPq)]
      ret.enableBsm = 0x3BA in fingerprint[0]  # SWA_1

      if 0x440 in fingerprint[0] or docs:  # Getriebe_1
        ret.transmissionType = TransmissionType.automatic
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x1A0, 0xC2)):  # Bremse_1, Lenkwinkel_1
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      # The PQ port is in dashcam-only mode due to a fixed six-minute maximum timer on HCA steering. An unsupported
      # EPS flash update to work around this timer, and enable steering down to zero, is available from:
      #   https://github.com/pd0wm/pq-flasher
      # It is documented in a four-part blog series:
      #   https://blog.willemmelching.nl/carhacking/2022/01/02/vw-part1/
      # Panda ALLOW_DEBUG firmware required.
      ret.dashcamOnly = True

    elif ret.flags & VolkswagenFlags.MEB:
      # Set global MEB parameters (ID.4, ID.5, etc.)
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMeb)]
      if ret.flags & VolkswagenFlags.MEB_GEN2:
        # 2024+ (ID.4 MK2 등): 신형 CRC 변형 - panda safety에 플래그 전달
        ret.safetyConfigs[0].safetyParam |= VolkswagenSafetyFlags.ALT_CRC_VARIANT_1.value
      ret.enableBsm = 0x24C in fingerprint[0]  # MEB_Side_Assist_01
      ret.transmissionType = TransmissionType.direct
      # MEB is curvature-controlled (HCA_03). openpilot routes curvature cars through
      # LatControlAngle (steerControlType == angle); the carcontroller consumes
      # actuators.curvature directly. Using curvatureDEPRECATED instead falls through
      # to LatControlPID, which crashes on the empty MEB pid tune.
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.steerAtStandstill = True

      if 0x25D in fingerprint[0]:  # KLR_01 - capacitive steering wheel module (Emergency Assist hands-on)
        ret.flags |= VolkswagenFlags.STOCK_KLR_PRESENT.value

      # 기어(GE_Fahrstufe) 소스: MK2(GEN2)는 Getriebe_11이 PT버스에 안 오고 Gateway_73으로만 온다.
      # 차종 플래그 대신 실제 메시지 유무로 판정 (commaai/opendbc 방식).
      # MK1 실차 rlog 검증: 두 메시지 모두 존재하며 GE_Fahrstufe 값이 1205/1205 프레임 완전 일치.
      if 0x3DC in fingerprint[0]:  # Gateway_73
        ret.flags |= VolkswagenFlags.ALT_GEAR.value

      if all(msg in fingerprint[2] for msg in (0x1A4, 0x1F0)):  # EA_01, EA_02 - Emergency Assist HUD
        ret.flags |= VolkswagenFlags.STOCK_EA_PRESENT.value

      if any(msg in fingerprint[1] for msg in (0x520, 0x86, 0xFD, 0x13D)):  # MEB gateway messages
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      # On the gateway harness the stock radar stays active; use it (radar+vision fusion) when present.
      # Strukturen_01 (0x24F) is the MEB radar object message. Camera harness keeps radar unavailable.
      # radarDelay(0.8) 보정과 함께 사용 (infiniteCable2와 동일). 보정 없으면 레이더 리드 타이밍이
      # 어긋나 조기제동/재출발 막힘이 났었음.
      if ret.networkLocation == NetworkLocation.gateway:
        ret.radarUnavailable = 0x24F not in fingerprint[0]

    else:
      # Set global MQB parameters
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volkswagen)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01

      if 0xAD in fingerprint[0] or docs:  # Getriebe_11
        ret.transmissionType = TransmissionType.automatic
      elif 0x187 in fingerprint[0]:  # Motor_EV_01
        ret.transmissionType = TransmissionType.direct
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x40, 0x86, 0xB2, 0xFD)):  # Airbag_01, LWI_01, ESP_19, ESP_21
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      if 0x126 in fingerprint[2]:  # HCA_01
        ret.flags |= VolkswagenFlags.STOCK_HCA_PRESENT.value

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerLimitTimer = 0.4
    if ret.flags & VolkswagenFlags.PQ:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif ret.flags & VolkswagenFlags.MEB:
      ret.steerActuatorDelay = 0.3
    else:
      ret.steerActuatorDelay = 0.1
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kf = 0.00006
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.2]

    # DISABLE_RADAR(카메라 하네스 롱컨): MEB + 카메라 하네스에서 MebDisableRadar 파라미터가 켜지면
    # 순정 레이더를 프로그래밍 세션에 가두고 openpilot이 AEB/레이더 메시지를 대체한다 (미검증).
    # 순정 AEB/FCW/EA를 잃는 대가로 카메라 하네스 롱컨을 가능하게 함. 게이트웨이 하네스는 불필요.
    if (ret.flags & VolkswagenFlags.MEB) and ret.networkLocation == NetworkLocation.fwdCamera and not docs:
      try:
        from openpilot.common.params import Params
        if Params().get_int("MebDisableRadar") > 0:
          ret.flags |= VolkswagenFlags.DISABLE_RADAR.value
      except Exception:
        pass

    # Global longitudinal tuning defaults, can be overridden per-vehicle
    # MEB 롱컨: 게이트웨이 하네스(순정 레이더/AEB 유지) 또는 카메라 하네스+DISABLE_RADAR에서 가능.
    ret.alphaLongitudinalAvailable = ret.networkLocation == NetworkLocation.gateway or \
                                     bool(ret.flags & VolkswagenFlags.DISABLE_RADAR) or docs

    # MQB는 carrot 원본과 동일하게 alpha_long만으로 활성(카메라 하네스 MQB 롱컨 유지).
    # MEB는 게이트웨이 하네스 또는 카메라 하네스+DISABLE_RADAR 필요.
    if alpha_long and (ret.alphaLongitudinalAvailable or not (ret.flags & VolkswagenFlags.MEB)):
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= VolkswagenSafetyFlags.LONG_CONTROL.value
      if ret.flags & VolkswagenFlags.DISABLE_RADAR:
        ret.safetyConfigs[0].safetyParam |= VolkswagenSafetyFlags.DISABLE_RADAR.value
      if ret.transmissionType == TransmissionType.manual:
        ret.minEnableSpeed = 4.5

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.stopAccel = -0.55
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.5
    ret.autoResumeSng = ret.minEnableSpeed == -1

    if ret.flags & VolkswagenFlags.MEB:
      # MEB longitudinal: openpilot's "starting" long-control state is required so the carcontroller
      # sends ACC_Anfahren (launch) on re-launch; without it the planner skips to pid, ACC_Anfahren
      # never fires, and the car won't release the EPB hold (re-launch fails). Also a very slow launch
      # can fault the car (EPB shutdown), so keep vEgoStarting ~0.5 m/s.
      ret.startingState = True
      ret.startAccel = 0.8
      ret.vEgoStarting = 0.5
      ret.vEgoStopping = 0.1
      # infiniteCable2와 동일: 종방향 액추에이터 지연 + 레이더 측정 지연 보정.
      # radarDelay 누락 시 레이더 리드의 위치/속도가 어긋나 조기제동·리드 불안정 유발.
      ret.longitudinalActuatorDelay = 0.5
      ret.radarDelay = 0.8
      ret.longitudinalTuning.kiBP = [0., 30.]
      ret.longitudinalTuning.kiV = [0.4, 0.]

    return ret

  # **** DISABLE_RADAR: 순정 레이더 무력화 (카메라 하네스 롱컨) ****
  # infiniteCable2 실코드 이식. 부팅 시 레이더(0x757)를 프로그래밍 세션에 가둬 송신을 멈추고,
  # carcontroller가 대체 AEB/레이더 메시지를 보낸다. 순정 AEB/FCW/EA 상실. 미검증 - 테스터 필요.
  @staticmethod
  def init(CP, can_recv, can_send):
    if CP.openpilotLongitudinalControl and (CP.flags & VolkswagenFlags.DISABLE_RADAR) and (CP.flags & VolkswagenFlags.MEB):
      RADAR_DISABLE_STATE["error"] = False
      # 엔진 On 상태에서는 프로그래밍 세션 요청이 거부됨(레이더 소생 불가) -> 시도하지 않음
      if CarInterface._is_engine_state_allowed_meb(can_recv):
        carlog.warning("Trying to disable the radar")
        if not CarInterface._radar_communication_control(CP, can_recv, can_send):
          RADAR_DISABLE_STATE["error"] = True
      else:
        RADAR_DISABLE_STATE["error"] = True
        carlog.warning("The radar can not be disabled (engine on)")

  @staticmethod
  def _radar_communication_control(CP, can_recv, can_send):
    # 레이더(0x757)를 프로그래밍 세션에 가둬 송신 중지. 기능주소 0x700로 Tester Present.
    bus = CANBUS.pt
    addr_radar, addr_diag, rx_offset = 0x757, 0x700, 0x6A
    retry, timeout = 3, 0.5

    tp_req  = bytes([uds.SERVICE_TYPE.TESTER_PRESENT, 0x00])
    tp_resp = bytes([uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00])
    ext_diag_req  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC])
    ext_diag_resp = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC])
    flash_req  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])

    for i in range(retry):
      try:
        # Tester Present
        query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr_radar, None)], [tp_req], [tp_resp], rx_offset, functional_addrs=[addr_diag])
        if not query.get_data(timeout):
          carlog.warning(f"Tester Present returned no data on attempt {i+1}")
          continue
        # Extended Diagnostic Session
        query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr_radar, None)], [ext_diag_req], [ext_diag_resp], rx_offset)
        if not query.get_data(timeout):
          carlog.warning(f"Radar extended session returned no data on attempt {i+1}")
          continue
        # Programming Session (응답 대기 없이 즉시 - 크루즈 폴트 방지 위해 바로 대체 송신 시작)
        query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr_radar, None)], [flash_req], [b''], rx_offset)
        query.get_data(0)
        carlog.warning(f"Radar disabled by programming session on attempt {i+1}")
        return True
      except Exception as e:
        carlog.error(f"Radar disable exception on attempt {i+1}: {repr(e)}")
        continue

    carlog.error("Radar disable failed")
    return False

  @staticmethod
  def _is_engine_state_allowed_meb(can_recv, timeout: float = 0.5) -> bool:
    # 안전장치: Motor_54(0x14C)의 Engine_On으로 프로그래밍 세션 가능 여부 판정
    end_time = time.monotonic() + timeout
    while time.monotonic() < end_time:
      packets = can_recv(wait_for_one=True) or []
      for packet in packets:
        for msg in packet:
          if msg.address != 0x14C:
            continue
          engine_on = bool((msg.dat[9] >> 5) & 0x01)
          if engine_on:
            carlog.warning("Engine state is not allowed: Engine_On=True")
            return False
          return True
    carlog.warning("Engine state unknown")
    return True
