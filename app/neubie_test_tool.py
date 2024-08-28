from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, Qt, pyqtSignal, QObject
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QRadioButton, QSizePolicy
from PyQt5.QtCore import QTimer, Qt

from ui.main_interface import Ui_MainWindow
from ui.choose_topics import Topic_add
from ui.left_fr_button import Domain_worker
from ui.radio_button_mapping import RadioButtonMapping

import os, stat, subprocess, sys, rclpy
import yaml

# from src.nb_param_set_get.set_parameter import ROSParameterGUI
from src.parameter_setting.sshgui import ROSParameterGUI
from src.nb_param_set_get.ver import SSHVersionChecker

from src.vl.vl_setting import vlThread

from src.topic_subscriber.topic_list import GetTopicList
from src.topic_subscriber.topic_list_worker import RosSubscriberThread

import resources.details as details
import resources.mapping as mapping
from resources.arguments import arguments

# pyuic5 main_interface.ui -o main_interface.py

class MainWindow(QMainWindow, Ui_MainWindow):
    controlStateChanged = pyqtSignal(int)  # control_state 값의 타입에 따라 변경 (remote_control_state)
    distanceDataChanged = pyqtSignal(object)
    updateMessageSignal = pyqtSignal(object)  # 새로운 신호 정의
    testSignal = pyqtSignal(int) #test

    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정   
        self.setWindowTitle("Neubie Test Tool")
        self.setup_ros_subscriber()
        self.main_frame.setStyleSheet("QFrame { border: none; }")
        self.radio_button_mapping = RadioButtonMapping()  # RadioButtonMapping 인스턴스 생성

        self.msg_debug_data = '' # 초기화 (라디오버튼 없는 값들 보여주기)

        #왼쪽 프레임을 안보이게 함
        self.left_slide_menu.setVisible(False)  # 처음에는 보이지 않도록 설정
        self.terminal.clicked.connect(self.show_left_slide_menu)
        #dds 설정 버튼 slot설정
        self.dds.clicked.connect(self.set_dds)
        #터미네이터 on 버튼 설정
        self.turnOn_terminator.clicked.connect(self.terminator_on)
        #domain 버튼 설정
        self.apply_domain.clicked.connect(self.domain_apply)
        #pub service target once button
        self.send_destination.clicked.connect(self.send_service_target)
        #robot_sw_ver 확인
        self.robot_sw_ver.clicked.connect(self.robot_sw_version)
        
        #parameter set 버튼
        self.pushButton.clicked.connect(self.parameter_set_get)
        #topic list 버튼
        self.pushButton_2.clicked.connect(self.subscribe_topic_list)
        
        #vlsetting
        self.pushButton_3.clicked.connect(self.vl_setting)
        
        #체크박스를 보고 topic 추가 버튼 연결
        self.nb_ego_checkbox.stateChanged.connect(self.ego_checkStateChange)
        
        #rut_wifi 기체 정보를 보여줌
        self.rut_wifi_button.clicked.connect(self.show_rut_wifi_info)
        
        #변수모음
        self._control_state = 0  # '_control_state' 변수 초기화
        self.control_state = 0 


        #테스트 용도
        self.counter = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateCounter)
        self.timer.start(1000)  # 1000 milliseconds = 1 second

    # topic_list.py의 MainWindow 클래스의 subscribe_topic_list 메서드 호출
    def subscribe_topic_list(self):
        
        self.topic_list_window = GetTopicList(node=self.ros_sub_thread.worker)
        self.topic_list_window.show()

    #show self.show_rut_wifi_info : RUT_info.yaml 파일에 기체 rut 정보를 넣어주면 됨
    def show_rut_wifi_info(self):
        selected_robot = self.rut_wifi.currentText()
        # print(selected_robot)
        self.dds_output.clear()
        
        #yaml 파일 정보 가져오기 (RUT_info.yaml)
        current_directory = os.path.dirname(__file__)
        file_path = os.path.join(current_directory, 'info', 'RUT_info.yaml')
        try:
            with open(file_path) as f:
                devices_info = yaml.safe_load(f)

            info = devices_info[selected_robot]
            info_text = f"WIFI SSID: {info['WIFI_SSID']}\n"
            info_text += f"WIFI PW: {info['WIFI_PW']}\n"
            info_text += f"USERNAME: {info['USERNAME']}\n"
            info_text += f"PASSWORD: {info['PASSWORD']}\n"
            
            info_text += f"SERIAL: {info['SERIAL']}\n"
            info_text += f"IMEI: {info['IMEI']}\n"
            info_text += f"LAN_MAC: {info['LAN_MAC']}\n"
            info_text += f"USIM_NO: {info['USIM_NO']}\n"
            info_text += f"통신사: {info['통신사']}\n"
            info_text += f"요금제: {info['요금제']}\n"
            info_text += f"펌웨어: {info['펌웨어']}\n"

            self.dds_output.setPlainText(info_text)

        except FileNotFoundError:
            self.dds_output.setPlainText("파일을 찾을 수 없습니다.")
        except KeyError:
            self.dds_output.setPlainText("정보를 찾을 수 없습니다.")
            

    #parameter set/get verification
    def parameter_set_get(self):
        self.parameter_set_get = ROSParameterGUI()
        self.parameter_set_get.show()
        
    def vl_setting(self):
        # vlThread 클래스의 인스턴스를 생성
        self.vl_thread = vlThread()

        # 필요한 경우 스레드 종료 후 처리할 작업 설정
        self.vl_thread.finished.connect(self.on_vl_thread_finished)

        # 스레드 시작 
        self.vl_thread.start()

    def on_vl_thread_finished(self):
        # 스레드가 종료된 후 처리할 작업 (필요에 따라 구현)
        print("VL Thread 작업이 완료되었습니다.")


    #robot_sw_version_verification
    def robot_sw_version(self):
        self.ssh_version_checker = SSHVersionChecker()
        self.ssh_version_checker.show()

    #테스트 용도
    def updateCounter(self):
        self.counter += 1
        # self.label.setText(str(self.counter))
        # print(self.counter)
        self.testSignal.emit(self.counter)  # 신호 발생 > 라디오버튼없는 클래스로 감``

    @property
    def control_state(self):
        return self._control_state

    @control_state.setter
    def control_state(self, value):
        if self._control_state != value:
            self._control_state = value
            self.controlStateChanged.emit(value)  # 상태 변경 시 신호 발생
            # print('control_state emit val', value)
        
    def ego_checkStateChange(self, state):
        chk_title = self.nb_ego_checkbox.text()
        if state == Qt.Checked:

            # print('qt_checked')
            # 다이얼로그 인스턴스 생성
            self.topic_dialog = Topic_add(self, chk_title)
            # 다이얼로그의 시그널과 슬롯 연결
            # self.topic_dialog.finished.connect(self.handle_dialog_finished)
            # 다이얼로그를 비모달로 표시
            self.topic_dialog.show()

    def send_service_target(self):
        # 메시지 내용
        service_target_id = "NB_SEONGDONG_00"
        category = "Campus"
        message = f'ros2 topic pub /service_target autonomy_ros2_message/ServiceTarget "{{service_target_id: {service_target_id}, category: {category}}}" --once'

        # 터미널 명령 실행
        subprocess.run(message, shell=True)
        # print("servcie_target 발행완료")
        self.dds_output.clear()
        self.dds_output.setPlainText(f"servcie_target 발행완료\n 성수|캠퍼스 로만 적용")

    def on_thread_finished(self):
        # print("Work completed.")
        pass
    
    # "Apply Domain" 버튼을 클릭했을 때 실행할 함수
    def domain_apply(self):
        domain_id = self.domain_spin_box.value()
        # QThread 객체 생성
        self.thread = QThread()
        # Worker 객체 생성
        self.worker = Domain_worker(domain_id)
        # Worker를 스레드로 이동
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.update_text.connect(self.dds_output.setPlainText)

        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker.finished.connect(self.on_worker_finished)  # 스레드 종료 후 처리를 위한 메서드 연결
        self.thread.start()

    # 스레드가 완전히 종료된 후 처리를 위한 메서드
    def on_worker_finished(self):
        self.thread.quit()
        self.thread.wait()  # 스레드가 완전히 종료 대기
        self.thread.deleteLater()
        # print("도메인 스레드 종료")

    # 왼쪽 메뉴 클릭시 터미네이터 실행
    def terminator_on(self):
        self.dds_output.clear()
        self.dds_output.setPlainText(f"터미널 실행")
        try:
            subprocess.run(["terminator"])
        except subprocess.CalledProcessError as e:
            # 에러 처리 코드 추가
            pass


    #기체별 RUT확인
    def verify_wifi(self):
        self.dds_output.clear()

    # 폴더와 파일이 있는지 확인하고 파일 생성하는 함수
    def set_dds(self):
        self.dds_output.clear()
        try:
            # 홈 디렉토리의 경로를 가져옵니다.
            home_dir = os.path.expanduser("~")
            self.dds_output.setPlainText(f"홈 디렉토리: {home_dir}")

            # 폴더 경로 설정
            folder_path = os.path.join(home_dir, "cmd")
            self.dds_output.appendPlainText(f"폴더 경로: {folder_path}")

            # 파일 경로 설정
            file_path = os.path.join(folder_path, "dds.sh")
            self.dds_output.appendPlainText(f"파일 경로: {file_path}")

            # 폴더 확인 후 생성
            if not os.path.exists(folder_path):
                os.makedirs(folder_path, exist_ok=True)
                self.dds_output.appendPlainText("폴더 생성 완료")
            else:
                self.dds_output.appendPlainText("폴더가 이미 존재합니다. 파일을 확인합니다.")
                
            self.check_dds_file(file_path)

        except Exception as e:
            self.dds_output.appendPlainText(f"에러 발생: {e}")

    def check_dds_file(self, file_path):
        # 파일 확인 후 생성
        if not os.path.exists(file_path):
            with open(file_path, "w") as file:
                file.write("#!/bin/bash\n\n")
                file.write("# XML 파일에서 NetworkInterfaceAddress 값을 변경하는 함수\n")
                file.write("set_network_interface_address() {\n")
                file.write("    local xml_file=\"$1\"\n")
                file.write("    local new_address=\"$2\"\n")
                file.write("    # sed를 사용하여 XML 파일에서 NetworkInterfaceAddress 값을 변경합니다.\n")
                file.write("    sed -i \"s|<NetworkInterfaceAddress>[^<]*</NetworkInterfaceAddress>|<NetworkInterfaceAddress>$new_address</NetworkInterfaceAddress>|\" \"$xml_file\"\n")
                file.write("}\n\n")
                file.write("# 현재 연결된 네트워크 인터페이스의 이름을 가져오는 함수\n")
                file.write("get_connected_interface() {\n")
                file.write("    # `ip route get 1` 명령어로 현재 연결된 인터페이스의 이름을 가져옵니다.\n")
                file.write("    ip route get 1 | awk '{print $5}'\n")
                file.write("}\n\n")
                file.write("# 사용자의 홈 디렉토리\n")
                file.write("home_dir=\"$HOME\"\n\n")
                file.write("# 사용자의 홈 디렉토리에서 cyclonedds.xml 파일을 찾습니다.\n")
                file.write("xml_file=$(find \"$home_dir\" -name \"cyclonedds.xml\" -print -quit)\n\n")
                file.write("# cyclonedds.xml 파일을 찾지 못한 경우 에러 메시지를 출력하고 스크립트를 종료합니다.\n")
                file.write("if [ -z \"$xml_file\" ]; then\n")
                file.write("    echo \"사용자의 홈 디렉토리에서 cyclonedds.xml 파일을 찾을 수 없습니다.\"\n")
                file.write("    exit 1\n")
                file.write("fi\n\n")
                file.write("# 현재 연결된 네트워크 인터페이스의 이름을 가져옵니다.\n")
                file.write("connected_interface=$(get_connected_interface)\n\n")
                file.write("# XML 파일에서 NetworkInterfaceAddress 값을 변경합니다.\n")
                file.write("set_network_interface_address \"$xml_file\" \"$connected_interface\"\n\n")
                file.write("echo \"robot_cfg XML 파일의 NetworkInterfaceAddress 값을 $connected_interface로 변경하였습니다.\"\n")
            self.dds_output.appendPlainText("dds.sh 파일 생성 완료")
            os.chmod(file_path, stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
            self.dds_output.appendPlainText("dds.sh 권한설정 완료")


        else:
            self.dds_output.appendPlainText("파일이 존재 스크립트 실행")
        self.dds_output.appendPlainText("dds.sh 스크립트 실행")

        try:
            # 스크립트 실행
            subprocess.run(file_path, shell=True)
            self.dds_output.appendPlainText("스크립트 실행 성공\n DDS 변경완료")
        except subprocess.CalledProcessError as e:
            self.dds_output.appendPlainText("스크립트 실행 실패/수동으로 진행 ㄱ")
            # print(f"스크립트 실행 실패: {e}")


    def show_left_slide_menu(self):
        is_visible = self.left_slide_menu.isVisible()
        self.left_slide_menu.setVisible(not is_visible)


    def applyStyle(self, radio_button, color):
        # 스타일 적용 함수
        try:
            radio_button.setStyleSheet(f"""
                QRadioButton::indicator {{
                    background-color: {color};
                    border-radius: 6px;
                    border-color: {color};
                }}
                QRadioButton::indicator:checked {{
                    background-color: {color};
                }}
                QRadioButton {{
                    background-color: white;
                    color: black;
                }}""")

        except Exception as e:
            print(f"Error setting QRadioButton style: {e}")

    def setup_ros_subscriber(self):
        self.ros_main_thread = RosSubscriberThread()
        self.ros_main_thread.message_signal.connect(self.update_ui)
        self.ros_main_thread.service_target_signal.connect(self.update_service_info_ui)
        self.ros_main_thread.is_crossing_signal.connect(self.update_is_crossing_info_ui)
        self.ros_main_thread.start()
        
        # Sub_node 설정
        self.ros_sub_thread = RosSubscriberThread(is_main=False, node_name='neubie_test_tool_sub_node')
        self.ros_sub_thread.start()

    def update_is_crossing_info_ui(self, is_crossing_info):
        # print(is_crossing_info)
        if is_crossing_info.data == False:
            crossing_info = 'is_crossing : false(횡단상태아님)'
        else:
            crossing_info = 'is_crossing : True(횡단보도 횡단상태)'
        self.cross_info.setText(f"{crossing_info}")

    def update_service_info_ui(self, service_info):
        #데이터 추출
        # print(service_info)
        service_target = service_info.service_target_id
        service_cartegory = service_info.category
        # print(service_target, service_cartegory)
        self.service_info.setText(f"{service_target} | {service_cartegory}")

    def update_ui(self, msg):
        # 데이터 추출
        battery_status = msg.system_monitor.battery_status
        h_acc = msg.h_acc
        ego_velocity = round(msg.ego_velocity, 5)
        formatted_ego_vel = "{:.3f}".format(ego_velocity)

        mrm_status_flag = msg.system_monitor.mrm_status_flag
        aeb_status = msg.system_monitor.aeb_status_flag

        desired_long = round(msg.desired_longitudinal_velocity_mps, 3)
        formatted_desired_long = "{:.3f}".format(desired_long)
        desired_lat = round(msg.desired_lateral_velocity_dps, 3)
        formatted_desired_lat = "{:.3f}".format(desired_lat)
        sys_info = msg.system_monitor.system_info #2.0.대 이거 때문에 죽음
        
        ##################기타 필요 정보 업데이트
        
        selected_reference = msg.selected_reference#도출기준경로
        road_type = msg.way_road_type#태그정보
        way_rule = msg.way_rule#태그정보
        
        parsed_selected_reference = self.get_dochulgijun(selected_reference)
        parsed_road_type = self.get_road_type(road_type)
        parsed_way_rule = self.get_way_rule(way_rule)
        
        # sys_info = msg.system_monitor
        
        #1.대 sys_info 정보 print 시 아래 정보 표기
        # sys_info 2024.5.21|14:2.2|V-Mem(NBP/Tot): 5.13GB/0%|P-Mem(NBP/Tot): 0.12GB/0%|CPU(NBP/Tot): 9.47%/90.44%|CPU-therm: 69°C|GPU-therm: 67°C|AUX-therm: 63°C|AO-therm: 63°C|PMIC-Die: 100°C|Tboard_tegra: 61°C|Tdiode_tegra: 64°C|thermal-fan-est: 66°C|
        
        # print('sys_info', sys_info)
        autonomy_condition = msg.system_monitor.autonomy_condition #0오퍼레이터/1관제/2자율주행
        

        self.control_state = msg.system_monitor.remote_controller_attention_flags #자율주행 중 관제 개입이 필요한 판단 상황에 대해서 관제에 주시 신호 알림 목적
        # print('self.control_state 변경값 확인',self.control_state)


        # print('값변경 확인', ego_velocity)

        # 정보 업데이트
        # print("msg.system_monitor.behavior_flags", msg.system_monitor.behavior_flags)
        self.update_behavior_flags(msg.system_monitor.behavior_flags)
        
        self.update_sys_info(sys_info)
        self.update_autonomy_data_failure(msg.system_monitor.anomaly_condition_flags.autonomy_data_failure_flags)
        self.update_robot_info(autonomy_condition, battery_status, h_acc, formatted_ego_vel, mrm_status_flag, parsed_selected_reference, aeb_status, formatted_desired_long, formatted_desired_lat, parsed_road_type, parsed_way_rule)
        self.update_sensor_failure_flags(msg.system_monitor.anomaly_condition_flags.sensor_failure_flags)
        self.update_dangerous_status_flags(msg.system_monitor.anomaly_condition_flags.dangerous_status_flags)
        # print(type(msg.debug_data))
                ##msg.debug_data.data 에서 필요한 것만 던지기
        if hasattr(msg.debug_data, 'data') and isinstance(msg.debug_data.data, list):
            for element in msg.debug_data.data:
                key = element.key.data  # std_msgs.msg.String 객체의 data 속성 접근
                value = element.value
                if key == 'cross_state':
                    self.update_crossing_status_flags(value) #라디오 버튼에 표시되게 던짐
                    # print(f'Key: {key}, Value: {value}')        
        
        self.msg_debug_data = msg.debug_data
        self.updateMessageSignal.emit(self.msg_debug_data)  # 신호 발생 > 라디오버튼없는 클래스로 감
        self.distanceDataChanged.emit(self.msg_debug_data)
 

    def get_dochulgijun(self, selected_reference):
        reference_map = mapping.reference_map
        parsed_selected_reference = reference_map.get(selected_reference, 'Unknown')
        # print('도출', selected_reference)
        return parsed_selected_reference


    def get_road_type(self, road_type):
        road_type_map = mapping.road_type_map
        parsed_road_type = road_type_map.get(road_type, '알 수 없음')
        # print('road_type', parsed_road_type)
        return parsed_road_type
    
    def get_way_rule(self, way_rule):
        way_rule_map = mapping.way_rule_map
        parsed_way_rule = way_rule_map.get(way_rule, '알 수 없음')
        # print('way_rule', parsed_way_rule)
        return parsed_way_rule



        # self.print_debug_data(msg.debug_data)
    ###DEBUG 데이터 걍 빼자
    def update_crossing_status_flags(self, crossing_data):
        cross_details = self.extract_crossing_details(crossing_data)
        details_str = '\n'.join([desc for _, desc in cross_details])
        cross_flag_num = [num for num, _ in cross_details]

        self.cross_flags.setText(f'{details_str}')
        self.cross_flags.setWordWrap(True)  # 텍스트 자동 개행 활성화
        self.cross_flags.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # 모든 라디오 버튼의 기본 색상을 green으로 설정
        self.applyDefaultColor(self.cross_indi)

        # 각 라디오 버튼의 색상을 업데이트
        for radio_button, flag_number in self.radio_button_mapping.cross_rd_buttons.items():
            radio_button = getattr(self, radio_button)  # 라디오 버튼 이름을 사용하여 라디오 버튼 객체 가져오기
            if flag_number in cross_flag_num:
                self.applyStyle(radio_button, "red")

    def update_dangerous_status_flags(self, dangerous_status_flags):
        dangerous_status_flag = self.calculate_binary_values(dangerous_status_flags)
        dangerous_details = self.extract_dangerous_details(dangerous_status_flag) 

        details_str = '\n'.join([desc for _, desc in dangerous_details])
        dangerous_flag_num = [num for num, _ in dangerous_details]

        self.dangerous_flags.setText(f'{details_str}')
        self.dangerous_flags.setWordWrap(True)  # 텍스트 자동 개행 활성화
        self.dangerous_flags.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # 모든 라디오 버튼의 기본 색상을 green으로 설정
        self.applyDefaultColor(self.danger_indi)

        # 각 라디오 버튼의 색상을 업데이트
        for radio_button, flag_number in self.radio_button_mapping.dangerous_rd_buttons.items():
            radio_button = getattr(self, radio_button)  # 라디오 버튼 이름을 사용하여 라디오 버튼 객체 가져오기
            if flag_number in dangerous_flag_num:
                self.applyStyle(radio_button, "red")

    def update_sensor_failure_flags(self, sensor_failure_flags):
        sensor_flag = self.calculate_binary_values(sensor_failure_flags)
        sensor_details = self.extract_sensor_details(sensor_flag) 

        details_str = '\n'.join([desc for _, desc in sensor_details])
        sensor_flag_num = [num for num, _ in sensor_details]

        self.sensor_flags.setText(f'{details_str}')
        self.sensor_flags.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # 모든 라디오 버튼의 기본 색상을 green으로 설정
        self.applyDefaultColor(self.sensor_indi)

        # 각 라디오 버튼의 색상을 업데이트
        for radio_button, flag_number in self.radio_button_mapping.sensor_rd_buttons.items():
            radio_button = getattr(self, radio_button)  # 라디오 버튼 이름을 사용하여 라디오 버튼 객체 가져오기
            if flag_number in sensor_flag_num:
                self.applyStyle(radio_button, "red")

    def update_sys_info(self, sys_info):
        # "|" 문자를 기준으로 문자열을 분리하여 리스트로 변환
        data_list = sys_info.split("|")
        # print(data_list)
        date = data_list[0]
        time = data_list[1]
        cpu =  data_list[4]
        # print(date,time,cpu)
        self.sys_info.setText(f'[Neubie Test Tool] {date} {time} {cpu}')


    def update_behavior_flags(self, behavior_flags):
        behavior_flag = self.calculate_binary_values(behavior_flags)
        #print('behavior_flag', behavior_flag) .. 리스트 안에 int
        detailed_behavior = self.extract_behavior_details(behavior_flag)
        details_str = '\n'.join([desc for _, desc in detailed_behavior])
        behavior_flag_num = [num for num, _ in detailed_behavior]

        self.behavior_flags.setText(f'{details_str}')
        self.behavior_flags.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # 모든 라디오 버튼의 기본 색상을 green으로 설정
        self.applyDefaultColor(self.behavior_light)

        # 각 라디오 버튼의 색상을 업데이트
        for behavior_name, flag_number in self.radio_button_mapping.behavior_rd_button.items():
            radio_button = getattr(self, behavior_name)  # 라디오 버튼 이름을 사용하여 라디오 버튼 객체 가져오기
            if flag_number in behavior_flag_num:
                self.applyStyle(radio_button, "red")

    def update_autonomy_data_failure(self, autonomy_data_failure_flags):
        autonomy_data_failure_flag = self.calculate_binary_values(autonomy_data_failure_flags)
        failure_details = self.extract_failure_details(autonomy_data_failure_flag)
        details_str = '\n'.join([desc for _, desc in failure_details])
        failure_flag_num = [num for num, _ in failure_details]

        self.autonomy_data_failure.setText(f'{details_str}')
        self.autonomy_data_failure.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # 모든 라디오 버튼의 기본 색상을 green으로 설정
        self.applyDefaultColor(self.aut_dat_failure)

        # 각 라디오 버튼의 색상을 업데이트
        for radio_button, flag_number in self.radio_button_mapping.autonomy_data_failure_buttons.items():
            radio_button = getattr(self, radio_button)  # 라디오 버튼 이름을 사용하여 라디오 버튼 객체 가져오기
            if flag_number in failure_flag_num:
                self.applyStyle(radio_button, "red")

    def update_robot_info(self, autonomy_condition, battery_status, h_acc, ego_velocity, mrm_status_flag, selected_reference,aeb_status, desired_long, desired_lat, way_road_type, way_rule):
        if autonomy_condition == 0:
            mode_status = "오퍼레이터"
            mode_status_color = "red"
        elif autonomy_condition == 1:
            mode_status = "관제"
            mode_status_color = "yellow"
        else:
            mode_status = "자율주행"
            mode_status_color = "black"
        mrm_behavior = self.get_mrm_behavior(mrm_status_flag)
        

        # print(aeb_status)
        # print(mrm_status_flag)

        
        # 색상 지정 로직을 get_color_by_val 함수를 사용하여 간소화
        aeb_color = self.get_color_by_val(aeb_status)
        mrm_color = self.get_color_by_val(mrm_status_flag)
        gps_color = self.get_color_by_val(h_acc)
        
        

        self.robot_info.setText(f'[<span style="color: {mode_status_color};"> {mode_status}</span>]'
                                f'[{battery_status}%]'
                                f'[<span style="color: {gps_color};">gps: {h_acc}</span>]'
                                f'[입력속도 {desired_long}]'
                                f'[출력속도 {ego_velocity}]'                                
                                f'[회전 {desired_lat}]'
                                f'<br>[<span style="color: {aeb_color};">AEB Status: {aeb_status}</span>]'
                                f'[<span style="color: {mrm_color};">mrm_flag: {mrm_status_flag} | {mrm_behavior}</span>]'
                                f'[도출기준정보: {selected_reference}]'
                                f'[road_type: {way_road_type}]'
                                f'[way_rule: {way_rule}]') #`사이트 설정필요
        self.robot_info.setTextFormat(Qt.RichText)  # 텍스트 형식을 RichText로 설정

    def get_color_by_val(self, val):
        if val == 1:
            return 'red'
        elif val <= 1000 and val != 0:
            return 'green'
        elif val >= 10000:
            return 'purple'
        else:
            return 'black'
        


    def applyDefaultColor(self, widget):
        for radio_button in widget.findChildren(QRadioButton):
            self.applyStyle(radio_button, "green")

    def calculate_binary_values(self, num):
        result_return = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048]
        return [val for val in result_return if num & val]

    def get_mrm_behavior(self, mrm_status_flag):
        behavior_dict = details.behavior_details
        return behavior_dict.get(mrm_status_flag, '알 수 없는 상태')

    def extract_failure_details(self, failure_flags):
        failure_details = details.failure_details
        return [(flag, failure_details.get(flag, '')) for flag in failure_flags]
        
    # def distance_in_debug(self):
    #     print('여기다가 디스턴스 업그레이드 ㄱㄱㄱㄱㄱㄱㄱㄱ')
        
    def extract_rmt_ctl_flags(self, rmt_ctl_flags):
        rmt_ctl_details = details.rmt_ctl_details

        return [(flag, rmt_ctl_details.get(flag, '')) for flag in rmt_ctl_flags]

    def extract_behavior_details(self, behavior_flags):
        behavior_details = details.behavior_details

        # 플래그 값과 해당하는 상세 설명을 튜플 형태로 리턴
        return [(flag, behavior_details.get(flag, '')) for flag in behavior_flags]
    
    def extract_sensor_details(self, sensor_flags):
        sensor_details = details.sensor_details
        return [(flag,sensor_details.get(flag, '')) for flag in sensor_flags]

    def extract_crossing_details(self, cross_flag):
        cross_details = details.cross_details
        # 단일 실수 값에 대한 세부 사항을 추출
        return [(cross_flag, cross_details.get(cross_flag, '정의되지 않은 상태'))]


    def extract_dangerous_details(self, dangerous_flags):
        dangerous_details = details.dangerous_details
        return [(flag, dangerous_details.get(flag, '')) for flag in dangerous_flags]
        
    def closeEvent(self, event):
        # 창이 닫힐 때 노드와 스레드 종료 처리
        if self.ros_main_thread.isRunning():
            # Executor와 Node 종료
            if self.ros_main_thread.executor:
                self.ros_main_thread.executor.shutdown()
            if self.ros_main_thread.worker:
                self.ros_main_thread.worker.destroy_node()

            # 스레드 종료
            self.ros_main_thread.quit()  # QThread의 이벤트 루프 종료
            self.ros_main_thread.wait()  # 스레드가 완전히 종료될 때까지 대기
        
        if self.ros_sub_thread.isRunning():
            # Executor와 Node 종료
            if self.ros_sub_thread.executor:
                self.ros_sub_thread.executor.shutdown()
            if self.ros_sub_thread.worker:
                self.ros_sub_thread.worker.destroy_node()

            # 스레드 종료
            self.ros_sub_thread.quit()  # QThread의 이벤트 루프 종료
            self.ros_sub_thread.wait()  # 스레드가 완전히 종료될 때까지 대기

        rclpy.shutdown()
        event.accept()
        print("ROS2 노드가 종료되었습니다.")

def main():
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()