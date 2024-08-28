topic_status = {
    "/cargo_open_status": {
        0 : {"color": "green", "status_text": "Lid closed"},
        1 : {"color": "red", "status_text": "Lid open"}
    },
    "/battery_status": {
        0 : {"color": "green", "status_text": "뚜ㅣ뛰"},
        1 : {"color": "red", "status_text": "빵빵"}
    },
    "/autonomous_stop_signal_from_server" : {
        True : {"color": "green", "status_text": "서버에서 정지신호 내려옴"},
        False : {"color": "red", "status_text": "서버에서 출발신호 내려음"}
    }
    # 여기에 추가적인 토픽 상태를 정의할 수 있습니다.
    # 예를 들어:
    # "/another_topic": {
    #     0: {"color": "blue", "status_text": "Status A"},
    #     1: {"color": "yellow", "status_text": "Status B"}
    # }
}
