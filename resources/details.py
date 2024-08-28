sensor_details = {
    1: '1.Topic(/cam_f/symbolic_link_alive)\n3초 유실[fc]|mrm[1-정지]',
    2: '2.Topic(/cam_fd/symbolic_link_alive)\n3초 유실[fd]|mrm[0-주행]',
    4: '4.Topic(/cam_fl/symbolic_link_alive)\n3초 유실[fl]|mrm[1-정지]',
    8: '8.Topic(/cam_fr/symbolic_link_alive)\n3초 유실[fr|mrm[1-정지]',
    16: '16.Topic(/cam_bl/symbolic_link_alive)\n3초 유실[bl]|mrm[1-정지]',
    32: '32.Topic(/cam_br/symbolic_link_alive)\n3초 유실[br]|mrm[1-정지]',
    64: '64.Alive Topic(/depth_fl/symbolic_link_alive)\n3초 유실[ldc]|mrm[1-정지]',
    128: '128.Alive Topic(/depth_fr/symbolic_link_alive)\n3초 유실[rdc]|mrm[1-정지]',
    256: '256.Alive Topic(/imu_port_status)\n3초 유실[imu]|mrm[1-정지]',
    512: '512.Alive Topic(/gnss_port_status)\n3초 유실[gnss]|mrm[1-정지]',
    1024: '1024.Alive Topic(/wheelodom_alive)\n3초 유실[motor drive]|mrm[1-정지]',
    2048: '2048.Alive Topic(/stm_port_status)\n3초 유실[STM]|mrm[1-정지]',
}

cross_details = {
    0: '0.횡단보도 시나리오와\n 관계 없는 상태',
    1: '1.횡단보도로 접근하고\n 있는 상태 ',
    2: '2.횡단보도를 건너기 위해\n 대기하고 있는 상태',
    3: '3.횡단보도를 건너고 있는\n 상태',
    4: '4.횡단 완료 상태',
    5: '5.아직 구체적인 정의가\n 이루어지지 않았음',
}

dangerous_details = {
    1: '1.자율주행모드에서 20초동안\n gnss위치데이터 누적거리가\n 1m이하|mrm[1-정지]',
    2: '2.gnss기반 주행경로\n 진행방향과 120도 차이|mrm[2-정지]',
    4: '4.gnss 최단경로점 4m이상\n 차이(이면도로 7m)(경로이탈)|mrm[1-정지]',
    8: '8.pitch.roll값 무게중신의\n 로봇범위를 벗어나는 33도 이상인 경우|mrm[1-정지]',
    16: '16.ldm맵핑 구간에서 뉴비가\n 주행불가영역을 90%이상 넘어갔을 때|mrm[1-정지]',
    32: '32.DR과 gnss위치 오차데이터가\n 뉴비 앞뒤로 이동하는\n 위험상황판단[TBD]|mrm[1-정지]',
    64: '64.자율주행모드에서 적재함이 열린상태로\n 3초이상 & 0.2m/s 이상속도주행\n or 적재함열린상태\n 3분이상 유지|mrm[1-정지]',
    128: '128.낭떠러지구간(cliff) [TBD]\n or forked_road',
}

behavior_details = {
    1: '1.최접점기준, 종착지 5m 인접시\n 목적지 주행동작신호',
    2: '2.횡단보도 정지',
    4: '4.종방향정지,전방경로위해 횡방향제어\n',
    8: '8.종횡방향정지\n최소대기시간10초유지 후 해제 > 8초타이머 시작?',
    16: '16.전역경로 목적지 변경',
    32: '32.Local_path\n 1.3m이내 사람 검출 (후방 1m)',
    64: '64.자율주행 정지\n 신호 수신',
    128: '128.하강 경사로\n 감속 3초이상 TBD이상 [TBD]',
    256: '256.Local_path\n 1.3m이내 뉴비 검출 (후방 1m)',
    512: '512.전방3m, 좌/우\n 1m내 주행영역 없음 (기준경로에서 발행)',
}

rmt_ctl_details = {
    1: '1.건널목 앞 정지\n 상태에서 플래그 발행',
    2: '2.뉴비가 건널목 주행\n 가능하다고 판단했을 때, 플래그 발행',
    4: '4.전역경로가 인지 기반\n 주행가능영역 위에 위치하지 않을 때, 플래그 발행',
    8: '8.관제주시 신호데이터 정의\n 사양서에는 없음',
}

failure_details = {
    1: '1./ekf_alive\n 3초 유실|mrm 0(주행)',
    2: '2./rtcm_alive\n 3초 유실|mrm 0(주행)',
    4: '4./ldm_msg\n 1초 유실|mrm 1(정지)',
    8: '8./obstacles_tracking_msg\n 0.5초 유실|mrm 1(정지)',
    16: '16./gnss_alive 3초\n 유실|mrm 0(주행)',
    32: '32./imu_alive 3초\n 유실|mrm 0(주행)',
    64: '64./odomgyro 0.1초\n 유실|mrm 1(정지)',
    128: '128./est_states_utm\n 0.1초 유실|mrm 1(정지)',
    256: '256.수신전역경로에서\n 현재 뉴비 위치의 way tag정보 없음|mrm 0(주행)',
    512: '512.ldm_msg에 3d height\n 단차정보 없음|mrm 1(정지)',
    1024: '1024.송신 ldm_msg에\n 3 height 단차 정보 없음|mrm 1(정지)'
}

behavior_dict = {
    0: '0.정상주행',
    1: '1.직진 및 회전정지',
    2: '2.직진정지\n 및 횡방향 유지',
    3: '3.직진 부드러운\n 정지 및 횡방향 유지',
    4: '4.갓길 도착 후\n 종방향 정지',
    5: '5.천천히 직진\n 및 횡방향 즉시정지',
    6: '6.직진방향 천천히 정지\n 및 횡방향 유지'
}