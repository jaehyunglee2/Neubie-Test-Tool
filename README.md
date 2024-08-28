# Neubie Test Tool

## 1. Abstract
Neubie Test Tool(이하, NTT)의 메인 폴더들에 대한 상세 설명을 담았습니다.

## 2. 요구사항
기본적인 QA 폴더 세팅은 마무리 되어있어야 하는 상태입니다.
- 만약 세팅이 안되어 있다면 아래의 링크 참조
    - [QA용 ROS2 Packages](https://neubility.atlassian.net/wiki/spaces/AUT/pages/343113937/QA+ROS2+Packages)
    
## 3. 사용 방법
0. 폴더를 받은 다음 setup.py가 위치한 곳에서 아래의 명령어 실행
    - 아래의 명령어는 처음 repo를 받은 경우에만 실행하면 됩니다.
```
pip install -e .
``` 
2. ntt실행 -- 해당 폴더에서 아래의 명령어를 실행하면 됩니다.
```
python3 neubie_test_tool.py
```
## 3. 폴더구조도 및 상세설명
- NTT는 아래와 같은 폴더 구조를 따릅니다
- 아래의 폴더 중, 알려드릴 폴더는 아래와 같습니다(메인 기능이 있는폴더)
    - app
    - resources
    - src/vl
    - src/parameter_setting
    - src/topic_subscriber
```
Neubie Test Tool
│
├── __init__.py
├── neubie_test_tool.py
├── setup.py
├── README.md
├── app
│   └── neubie_test_tool.py
├── icons
│   ├── images.qrc
│   └── nb.svg
├── image
│   ├── nb.ico
│   └── nb.png
├── resources
│   ├── arguments.py
│   ├── details.py
│   ├── ip_info.json
│   ├── mapping.py
│   ├── topic_status_manager.py
│   ├── topic_status.py
│   └── RUT_info.yaml
├── src
│   ├── __init__.py
│   ├── nb_param_set_get
│   │   ├── set_parameter.py
│   │   └── ver.py
│   ├── parameter_setting
│   │   ├── sshgui.py
│   │   └── sshworker.py
│   ├── topic_subscriber
│   │   ├── observer_class.py
│   │   ├── topic_list.py
│   │   ├── topic_list_worker.py
│   │   ├── topic_observer.py
│   │   ├── topic_status_manager.py
│   │   └── topic_status.py
│   └── vl
│       ├── vl_setting.py
│       └── vlwidget.ui
├── ui
│   ├── choose_topics.py
│   ├── left_fr_button.py
│   ├── main_interface.py
│   ├── main_interface.ui
│   ├── main_interface_update.ui
│   └── radio_button_mapping.py
└── workspace_src_checkout.py
```

### 3.1 app
--- 
해당 폴더는 NTT의 Main UI가 있는 폴더 입니다. Main UI에서는
세가지 토픽들을 ROS2 Node를 활용하여 받아옵니다.
```
1. '/nb_ego_status_logger'
2. '/service_target'
3. '/is_crossing'
```
추가적으로 버튼을 활용하여 아래의 스레드를 활성화시킬 수 있도록 작업되어 있습니다. 
> 1. Visual Localization(이하, VL) 세팅
> 2. ROS2 Sub Node 실행 (3가지 외의 다른 토픽들을 받아오기 위해)
> 3. ROS2 Parameter 세팅

따라서 MainUI를 수정하기 위해서는 app/neubie_test_tool.py를 수정하시면 됩니다.

### 2.2 resources
---
해당 폴더는 테스트에 필요한 다양한 변수들을 관리하는 폴더입니다.
<br>
아래는 각 파일들이 관리하는 것들이 무엇인지 설명합니다.

> arguments.py
```
원격 접속에 사용되는 뉴비의 VPN IP, VL에 사용되는 뉴비의 시리얼 번호를 관리하는 폴더입니다.

해당 폴더에 동일한 양식으로 IP와 시리얼 번호를 입력해 주셔야, 
NTT를 실행 할 때 해당 기체로 VPN연결과 VL세팅을 하실 수 있습니다.
```

참고 링크
- [뉴비의 시리얼 번호 확인](https://docs.google.com/spreadsheets/d/1V88kXNa6xioX4mGUf2nF8MRHoy4PegGpYLE7akkRufY/edit?gid=1118169727#gid=1118169727)
- [뉴비의 VPN IP 확인](https://grafana.neubie.co.kr/d/Y_aAWrUVz/neubie-ansible-role-status?orgId=1)

> details.py
```
시스템 모니터링 컨플루언스에 의거하여, 각 플레그에 대한 설명을 작성하는 곳입니다.
각 binary 정보가 무엇인지 dictionary 형태로 작성해 주신 뒤, 

app/neubie_test_tool.py 에 적용해 주시면 그에 맞게 데이터를 열거 해 주며 신호를 시각적으로 확인할 수 있습니다.
```

참고 링크
- [시스템 모니터링 시스템](https://neubility.atlassian.net/wiki/spaces/AUT/pages/174981123/-+Planning+Control)


> ip_info.json
```
NTT ver1 에서 사용한 json 파일로, 더 이상 사용되지 않습니다.
```

> mapping.py
```
뉴비가 받아서 사용하는 정보들이 사용자에게 직관적으로 받아와지지 않는 경우가 있습니다.
(예. road_type_map 이 0 일 경우, 이것을 직관적으로 "도로 정보를 알 수 없다" 의 상황인지 받아와지지 않습니다. )
이러한 부분들을 관리하기 위해 해당 파일에서 각 정보들을 매핑할 수 있도록 작업합니다.

작업한 뒤에는
app/neubie_test_tool.py 에 적용해 주시면 그에 맞게 데이터를 열거 해 주며 신호를 시각적으로 확인할 수 있습니다.
```

참고 링크
- [지도 정보의 코드레벨 정의](https://neubility.atlassian.net/wiki/x/iYDKMQ)
- [노면 기반 경로 보정](https://neubility.atlassian.net/wiki/x/lgDtHQ)

> topic_status_manager.py
```
추가적인 토픽을 시각적으로 보고 싶을 경우, 이 곳에서 토픽들에 대한 조건을 추가해주시면 됩니다.
```

> topic_status.py
```
추가적인 토픽을 시각적으로 보고 싶을 경우, 
topic_status.py 에서 조건들에 대한 상세 설명을 작성하시면 됩니다. 해당 파일에 예시가 작성되어 있습니다.
```
참고 링크
- TBD

> RUT_info.yaml
```
뉴비 내부에 있는 RUT정보를 받아올 수 있도록 하는 yaml파일 입니다.
해당 데이터를 확인하기 위해서는 뉴비 내부의 와이파이를 뜯어서 확인해야하기에, 자주 사용되지는 않습니다.
네트워크 불안정 상태를 만들기 위해 임의로 와이파이를 끌 때 사용되었습니다.
```
참고링크
- [네트워크 종료시키는 곳](https://rms.teltonika-networks.com/management/devices)