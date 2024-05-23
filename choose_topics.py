from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QRadioButton, QCheckBox
from PyQt5.QtCore import Qt
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
import os, sys
from autonomy_ros2_message.msg import LoggingMsg, ServiceTarget, DebugDataList

# pyuic5 nb_ego_status_logger.ui -o nb_ego_status_logger.py
# pyuic5 topic_msg.ui -o topic_msg.py
# pyuic5 topic_msg_wo_radio.ui -o topic_msg_wo_radio_button.py
# pyuic5 main_interface.ui -o main_interface.py


class Topic_add(QDialog):
    def __init__(self, parent=None, title=''):
        super().__init__(parent)
        ui_folder = os.path.join(os.path.dirname(__file__), 'ui')
        ui_file = os.path.join(ui_folder, "nb_ego_status_logger.ui")

        loadUi(ui_file, self)  # UI 파일 불러오기
        self.setWindowTitle(f"Choose {title}'s msgs")
        # self.select_msgs.clicked.connect(lambda: self.choose_msg(msgs))
        self.mainwindow = parent
        # print('self.mainwindow.control_state', self.mainwindow.control_state)
        #체크박스를 보고 추가할 msgs 선택
        self.rmt_ctl_flags_chkBox.stateChanged.connect(self.ego_msgs_show)
        # self.distance_data_chkBox.stateChanged.connect(self.ego_msgs_show) 
        self.distance_data_chkBox.stateChanged.connect(self.ego_msgs_show) 

    #여기에서 어떤 메시지를 받을지 표기를 하고 그때마다 Show_msg를 변경해 줘야함
    def ego_msgs_show(self, state):
        if state == Qt.Checked:
            checked_checkbox = self.sender()  # 이벤트를 발생시킨 체크박스 가져오기
            if checked_checkbox == self.rmt_ctl_flags_chkBox:
                title = self.rmt_ctl_flags_chkBox.text()
                self.show_msgs_dialog = Show_msg(self.mainwindow, title) #이건 숫자 라디오 버튼 필요한 것들
                self.hide()
                self.show_msgs_dialog.show()
            elif checked_checkbox == self.distance_data_chkBox:
                title = self.distance_data_chkBox.text()
                # print('dzdz')
                # self.show_msgs_dialog = Show_msg(self.mainwindow, title) #글만 있는 것은 다른 클래스 만들어서 진행 / 숫자 라디오 버튼 x
                self.show_msg_wo_dialog = Show_msg(self.mainwindow, title)
                self.hide()
                self.show_msg_wo_dialog.show()
            else:
                title = ""  # 다른 체크박스가 체크된 경우

class Show_msg_wo_radio_bt(QDialog):
    def __init__(self, parent=None, title=''):
        super().__init__(parent)
        ui_path = self.resource_path("topic_msg_wo_radio.ui")
        loadUi(ui_path, self)  # 수정된 UI 파일 로드 방식
        # loadUi("topic_msg.ui", self)  # UI 파일 불러오기
        self.mainwindow = parent
        # self.debug_data_wo_radio = self.mainwindow.msg_debug_data.data
        self.mainwindow.updateMessageSignal.connect(self.handleUpdateMessage)
        self.mainwindow.testSignal.connect(self.handletestsignal)
        self.setWindowTitle(f"Topic {title} msgs")
        # self.show_msg()

    def resource_path(self, relative_path):
        """리소스 파일의 절대 경로를 반환합니다."""
        try:
            # PyInstaller가 생성한 임시 폴더에서 실행 중인 경우
            base_path = sys._MEIPASS
        except Exception:
            # 일반적인 Python 환경에서 실행 중인 경우
            base_path = os.path.abspath(".")
        
        return os.path.join(base_path, relative_path)
    
    def handletestsignal(self, counter):
        pass
        # print('받는 쪽 ', counter)
    
    def handleUpdateMessage(self, debug_msg):
        # 데이터를 누적할 변수 선언
        all_data = ""

        # 데이터가 있는지 확인
        if len(debug_msg.data) == 0:
            return
        else:
            # 각 객체의 속성에 접근하여 누적
            for debug_data_element in debug_msg.data:
                data_str = str(debug_data_element)  # DebugDataElement 객체를 문자열로 변환
                first_open_parenthesis_index = data_str.find("(")  # 첫 번째 "("의 인덱스를 찾음
                if first_open_parenthesis_index != -1:
                    data_content = data_str[first_open_parenthesis_index:]  # 첫 번째 "(" 부터 출력
                    all_data += f'{data_content}\n'  # 누적
                    print(data_content)  # 출력

        # GUI 요소에 처리된 데이터 출력
        self.show_topic_msg_wo_radio.setText(all_data)

        # distance_data 체크박스 제목    
class Show_msg(QDialog):
    def __init__(self, parent=None, title=''):
        super().__init__(parent)
        ui_folder = os.path.join(os.path.dirname(__file__), 'ui')
        ui_file = os.path.join(ui_folder, 'topic_msg.ui')
        ui_path = self.resource_path(ui_file)
        loadUi(ui_path, self)  # 수정된 UI 파일 로드 방식
        # loadUi("topic_msg.ui", self)  # UI 파일 불러오기
        self.mainwindow = parent
        self.setWindowTitle(f"Topic {title} msgs")
        self.init_rd_buttons(title)
        
        # print((f"Topic_Title : Topic {title} msgs"))
        
        # 메인 윈도우의 신호를 현재 다이얼로그의 슬롯에 연결 (메인윈도우의 라인임 controlStateChanged = pyqtSignal(int)  # control_state 값의 타입에 따라 변경 (remote_control_state))
        if '관제개입' in title:
            self.mainwindow.controlStateChanged.connect(self.updateBasedOnControlState)
            # print('choose_topics, 관제개입')
        elif 'distance_data' in title:
            # print('distance_data 확인')
            self.mainwindow.distanceDataChanged.connect(self.handleUpdateMessage)
        # elif 'distance_data' in title:
        #     print('함수 만들어서 밸류값 보여주면 됨')
            

    def handleUpdateMessage(self, debug_msg):
        # 모든 라디오 버튼의 기본 색상을 green으로 설정
        self.mainwindow.applyDefaultColor(self.msg_indi)
        # 데이터를 누적할 변수 선언
        all_data = ""

        # 데이터가 있는지 확인
        if len(debug_msg.data) == 0:
            return
        else:
            # DebugDataElement에서 모든 데이터 누적
            for element in debug_msg.data:
                all_data += f"{element.key.data}: {element.value}\n"  # 데이터를 누적
                # print(type(element.key.data))
                # print('key', element.key.data)
                # print(type(element.value))
                # print('value',element.value)
                
                if 'Yield: parking abort by obstacles AEB' in element.key.data and 1 == element.value:
                    # print(element.key.data)

                    # 각 라디오 버튼의 색상을 업데이트 // radio_button_mapping.py 에서 self.distance_monitor_rd_buttons 이 부분과 
                    # # init_rd_buttons 부분 업데이트
                    for remote_control_name, flag_number in self.mainwindow.radio_button_mapping.distance_monitor_rd_buttons.items():
                        # print('remote_control_name', remote_control_name)
                        # print('flag_number', flag_number)
                        radio_button = self.radioButtons[remote_control_name]
                        self.mainwindow.applyStyle(radio_button,"red")                        


        self.desc.setText(f'{all_data}')
        self.desc.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        

        
    def resource_path(self, relative_path):
        """리소스 파일의 절대 경로를 반환합니다."""
        try:
            # PyInstaller가 생성한 임시 폴더에서 실행 중인 경우
            base_path = sys._MEIPASS
        except Exception:
            # 일반적인 Python 환경에서 실행 중인 경우
            base_path = os.path.abspath(".")
        
        return os.path.join(base_path, relative_path)

    def updateBasedOnControlState(self, control_state):
        # 메인 윈도우의 control_state 값에 따라 라디오 버튼의 색상 업데이트
        # 예: self.setRadioButtonColor(control_state)
        # print('control_state 값 확인 받는 쪽', control_state)
        remote_control_status_flag = self.mainwindow.calculate_binary_values(control_state)
        # print('remote_control_status_flag', remote_control_status_flag)

        detailed_remote_control = self.mainwindow.extract_rmt_ctl_flags(remote_control_status_flag)
        # print('detailed_remote_control', detailed_remote_control)
        details_str = '\n'.join([desc for _, desc in detailed_remote_control])
        remote_control_flag_num = [num for num, _ in detailed_remote_control]

        # print('remote_control_flag_num',remote_control_flag_num)

        self.desc.setText(f'{details_str}')
        self.desc.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # 모든 라디오 버튼의 기본 색상을 green으로 설정
        self.mainwindow.applyDefaultColor(self.msg_indi)

        # 각 라디오 버튼의 색상을 업데이트
        for remote_control_name, flag_number in self.mainwindow.radio_button_mapping.remote_monitor_rd_buttons.items():
            if flag_number in remote_control_flag_num:
                # 딕셔너리에서 라디오 버튼 객체 찾기
                if remote_control_name in self.radioButtons:
                    # print('remote_control_name', remote_control_name)
                    # print('self.radioButtons', self.radioButtons)
                    radio_button = self.radioButtons[remote_control_name]
                    # print('radio_button', radio_button)
                    # print('radio_button', type(radio_button))
                    
                    self.mainwindow.applyStyle(radio_button, "red")

    #이 함수에서 Topic_add에서 전달받은 title을 보고 사양서에 맞춰 버튼 추가 하면 됨
    def init_rd_buttons(self, title):
        # print('distance_dataasdfasdfasdfa')
        self.radioButtons = {}  # 라디오 버튼들을 관리할 딕셔너리 초기화
        if "rmt_control_attention" in title:
            # QVBoxLayout 인스턴스를 생성하여 msg_indi 프레임에 설정
            # print('확장 여기까지는')
            self.radioLayout = QVBoxLayout(self.msg_indi)  # msg_indi QFrame
            # 라디오 버튼 생성 및 딕셔너리에 추가
            self.radioButtons["건널목 앞 정지 신호"] = self.createAndStyleRadioButton("건널목 앞 정지 신호")
            self.radioButtons["건널목 주행 가능 신호"] = self.createAndStyleRadioButton("건널목 주행 가능 신호")
            self.radioButtons["전역경로 이상 신호"] = self.createAndStyleRadioButton("전역경로 이상 신호")
            self.radioButtons["8번 사양서에 없음"] = self.createAndStyleRadioButton("8번 사양서에 없음")
            # 필요에 따라 추가 라디오 버튼을 딕셔너리에 계속 추가 (라디오 버튼 값이랑 매핑 시켜야 동작)

            self.mainwindow.applyDefaultColor(self.msg_indi)
        elif "distance_data" in title:
            # QVBoxLayout 인스턴스를 생성하여 msg_indi 프레임에 설정
            self.radioLayout = QVBoxLayout(self.msg_indi)  # msg_indi QFrame
            # 라디오 버튼 생성 및 딕셔너리에 추가
            self.radioButtons["양보점 이동 중 AEB"] = self.createAndStyleRadioButton("양보점 이동 중 AEB")
            # self.radioButtons["test"] = self.createAndStyleRadioButton("test")
            # print('distance_dataasdfasdfasdfa')

    #초기 라디오 버튼 색상설정
    def createAndStyleRadioButton(self, title):
        radioButton = QRadioButton(title)
        self.setRadioButtonStyle(radioButton)
        self.radioLayout.addWidget(radioButton)
        return radioButton  # 생성된 라디오 버튼 객체를 반환

    #초기 라디오 버튼 비활성화 설정
    def setRadioButtonStyle(self, radioButton):
        # 라디오 버튼의 스타일 설정 (색상: 검정, 비활성화 상태)
        radioButton.setStyleSheet("QRadioButton { color: black; }")
        radioButton.setEnabled(False)



# start
# std_msgs.msg.String(data='Approaching: local_result_distance')
# 3.6477224687738383
# std_msgs.msg.String(data='Approaching: remaining_distance_to_waiting_global')
# 2.8991397145986904
# std_msgs.msg.String(data='Approaching: remaining_distance_to_waiting_local')
# 3.6477224687738383
# std_msgs.msg.String(data='Crossing: crosswalk_end_point_is_nearby_flag')
# 1.0
# std_msgs.msg.String(data='Crossing: is_robot_inside_non_crosswalk')
# 1.0
# std_msgs.msg.String(data='Crossing: remaining_distance')
# 2.4141678778114173
# std_msgs.msg.String(data='Crossing: robot_can_cross_flag (T/F)')
# 0.0
# std_msgs.msg.String(data='Not Crossing: remaining_distance_towalk_crosswalk_entrance')
# 43.25441306580905
# std_msgs.msg.String(data='Yield:yield_state_set')
# 0.0
# std_msgs.msg.String(data='cross_state')
# 0.0
# end
        
        # print(type(self.debug_data_wo_radio))
        # print(self.debug_data_wo_radio[0].key) # std_msgs.msg.String(data='Approaching: local_result_distance')
        # print(self.debug_data_wo_radio[1].key)# std_msgs.msg.String(data='Approaching: remaining_distance_to_waiting_global')
        # print(self.debug_data_wo_radio[2].key)# std_msgs.msg.String(data='Approaching: remaining_distance_to_waiting_local')
        # print(self.debug_data_wo_radio[3].key)# std_msgs.msg.String(data='Crossing: crosswalk_end_point_is_nearby_flag')
        # print(self.debug_data_wo_radio[4].key)# std_msgs.msg.String(data='Crossing: is_robot_inside_non_crosswalk')
        # print(self.debug_data_wo_radio[5].key)# std_msgs.msg.String(data='Crossing: remaining_distance')
        # print(self.debug_data_wo_radio[6].key)# std_msgs.msg.String(data='Crossing: robot_can_cross_flag (T/F)')
        # print(self.debug_data_wo_radio[7].key)# std_msgs.msg.String(data='Not Crossing: remaining_distance_towalk_crosswalk_entrance')

        # self.Approaching_local_result_distance = self.debug_data_wo_radio[0].value
        # self.Approaching_remaining_distance_to_waiting_global = self.debug_data_wo_radio[1].value
        # self.Approaching_remaining_distance_to_waiting_local = self.debug_data_wo_radio[2].value
        # self.Crossing_crosswalk_end_point_is_nearby_flag = self.debug_data_wo_radio[3].value
        # self.Crossing_is_robot_inside_non_crosswalk = self.debug_data_wo_radio[4].value
        # self.Crossing_remaining_distance = self.debug_data_wo_radio[5].value
        # self.Crossing_robot_can_cross_flag = self.debug_data_wo_radio[6].value
        # self.Not_Crossing_remaining_distance_towalk_crosswalk_entrance = self.debug_data_wo_radio[7].value
        # print(self.debug_data_wo_radio)