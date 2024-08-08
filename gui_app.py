import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem, QRadioButton, QWidget, QHBoxLayout
from PyQt5.QtCore import QThread, Qt, pyqtSlot
from mainwindow import Ui_MainWindow
from ros_subscriber_update import RosSubscriberWorker
from function_button import ProviderButtons
from robot_version import SSHVersionChecker
from set_parameter import ROSParameterGUI
from syslog_monitor import SSHLogMonitor
from PyQt5.QtGui import QBrush, QColor
from topic_list import GetTopicList  # topic_list.py 파일의 GetTopicList 클래스 import

import cProfile
import pstats

#pyuic5 main.ui -o mainwindow.py


class RosWorkerThread(QThread):
    def __init__(self):
        super().__init__()
        self.worker = RosSubscriberWorker()

    def run(self):
        profiler = cProfile.Profile()
        profiler.enable()
        self.worker.run()

        profiler.disable()
        # 프로파일링 결과 출력
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.print_stats()

class GuiApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        
        self.worker_thread = RosWorkerThread()
        self.ui.setupUi(self)
        self.init_ui()
        self.init_item()


    def init_item(self):
        self.behavior_flag_items = self.worker_thread.worker.behavior_flag_items
        self.behavior_mapping = self.create_issue_flag_dict(self.behavior_flag_items)
        
        self.sensor_info_items = self.worker_thread.worker.sensor_info_items
        self.sensor_info_mapping = self.create_issue_flag_dict(self.sensor_info_items)

        self.aut_info_items = self.worker_thread.worker.aut_info_items
        self.aut_info_mapping = self.create_issue_flag_dict(self.aut_info_items)

        self.mrm_flag_items = self.worker_thread.worker.mrm_flag_items
        self.mrm_mapping = self.create_issue_flag_dict(self.mrm_flag_items)
        
        # 참고 자료 URL
        # gps 정보 뉴비고에 0,0 으로 표현조건 : https://neubility.atlassian.net/browse/NEUB-878
        # 로봇(배터리) 배차가능 상태 조건 : https://neubility.atlassian.net/wiki/spaces/PMO/pages/752123924
        # 도출기준경로 : https://neubility.atlassian.net/wiki/spaces/AUT/pages/502071446
        # way_road_type : https://neubility.atlassian.net/wiki/spaces/AUT/pages/835354761
        
        # 복잡한 매핑을 위한 사전 정의
        self.key_mapping = {
            '모드': {
                '2': '자율주행',
                '1': '관제',
                '0': '오퍼레이터'
            },
            '배터리 상태': lambda v: '배차가능' if int(v) >= 20 else '배차불가,대기장소/스테이션 가능',
            'GPS': lambda v: '현재위치 0,0 전송조건' if int(v) >= 30000 or int(v) == 0 else '기준경로후보도출가능' if 100 < int(v) < 30000 else '-',
            '회전': lambda v: '우회전' if float(v) > 0 else '좌회전' if float(v) < 0 else '-',
            '도출기준경로': { 
                '-1': '전역경로추종(인지단계-2)', #https://neubility.atlassian.net/wiki/spaces/AUT/pages/502071446/-+Planning+Control#3.-%EB%94%94%EB%B2%84%EA%B9%85
                '0': '우측',
                '1': '좌측',
                '2': '로봇기반',
                '3': '중간',
                '4': '전역경로',
                '5': '점자블록',
                '6': '우측연석',
                '7': '좌측연석',
                '8': '우측노면영역',
                '9': '좌측노면영역'
            },
            'RoadType[태그정보]' : { # https://neubility.atlassian.net/wiki/spaces/AUT/pages/835354761, https://neubility.atlassian.net/wiki/spaces/AUT/pages/367558719/Global+Planning+Map+Its+Tag+Structure
                '0' : '알 수 없음',
                '1' : 'SideWalk',
                '2' : 'DriveWay',
                '3' : 'CrossWalk',
                '4' : 'SideRoad',
                '5' : 'Indoor',
                '6' : 'Wheelchair'
            },
            'WayRule[태그정보]' : {
                '0' : '알 수 없음',
                '1' : '우측주행',
                '2' : '좌측주행',
                '3' : '중앙주행',
                '4' : '보호구역'
            },
            'mrm' : { # https://neubility.atlassian.net/wiki/spaces/AUT/pages/138248207/-+Planning+Control#3.-%EC%9C%84%ED%97%98-%EC%83%81%ED%99%A9%EC%97%90-%EB%8C%80%ED%95%9C-%EC%83%81%ED%83%9C-%ED%8C%90%EB%B3%84-%EA%B8%B0%EB%B0%98-%EC%95%88%EC%A0%84-%EC%A0%9C%EC%96%B4-(MRM)-%EA%B8%B0%EB%8A%A5
                '0' : '정상주행',
                '1' : '직진/회전정지',
                '2' : '직진정지/회전가능',
                '3' : '직진부드러운정지/회전가능'
            }
        }  

    def init_ui(self):
        self.worker_thread.worker.nb_ego_status_logger_signal.connect(self.update_data)
        self.worker_thread.worker.service_target_signal.connect(self.update_service_target)
        self.worker_thread.start()
        
        self.function_button = ProviderButtons() #function_button.py class
        self.ui.pushButton_3.clicked.connect(self.function_button.send_service_target)  # 버튼과 함수 연결
        self.ui.pushButton_4.clicked.connect(self.function_button.set_dds)


        
        

        # "버전 확인" 버튼 클릭 시 SSHVersionChecker의 connect_to_ip 메서드 호출
        self.ui.pushButton_2.clicked.connect(self.robot_sw_version)
        
        # "Prameter SET/GET" 버튼 클릭 시 ROSParameterGUI의 ROSParameterGUI 메서드 호출
        self.ui.pushButton_5.clicked.connect(self.parameter_set_get)

        self.ui.pushButton_6.clicked.connect(self.sys_log)

        # topic_list.py의 MainWindow 클래스 인스턴스 생성
        ## 추가예정
        
        # topic_list.py의 MainWindow 클래스의 subscribe_topic_list 메서드 호출
        self.ui.pushButton.clicked.connect(self.subscribe_topic_list)

    # topic_list.py의 MainWindow 클래스의 subscribe_topic_list 메서드 호출
    def subscribe_topic_list(self):
        self.topic_list_window = GetTopicList()
        self.topic_list_window.show()

    #robot_sw_version_verification
    def robot_sw_version(self):
        self.ssh_version_checker = SSHVersionChecker()
        self.ssh_version_checker.show()
                
    #parameter set/get verification
    def parameter_set_get(self):
        self.parameter_set_get = ROSParameterGUI()
        self.parameter_set_get.show()
                
    def sys_log(self):
        self.sys_log = SSHLogMonitor()
        self.sys_log.show()
                
    @pyqtSlot(str, str)
    def update_service_target(self, service_target, service_category):
        # print(service_target, service_category)
        self.ui.label_4.setText(f"{service_target}({service_category})")


    @pyqtSlot(dict, dict, dict, dict)
    def update_data(self, arg1, arg2, arg3, arg4):
        
        self.update_table(arg1)
        self.update_sensor_table(arg2)
        self.update_cross_table(arg3)
        self.update_debug_text_edit(arg4) 
        # print(arg1, arg2, arg3, arg4)
        
        #열과 행 크기를 내용에 맞게 조정
        self.ui.tableWidget.resizeColumnsToContents()
        self.ui.tableWidget.resizeRowsToContents()
        self.ui.tableWidget_2.resizeColumnsToContents()
        self.ui.tableWidget_2.resizeRowsToContents()
        self.ui.tableWidget_3.resizeColumnsToContents()
        self.ui.tableWidget_3.resizeRowsToContents()
        # self.ui.tableWidget_4.resizeColumnsToContents()
        # self.ui.tableWidget_4.resizeRowsToContents()


    def set_default_item(self, table_widget, row, column, text, color="gray"):
        """Helper method to set a default item with optional color for a specified table widget."""
        item = self.create_centered_item(text, color)
        table_widget.setItem(row, column, item)


    def update_table(self, data): # tablewidget
        self.ui.tableWidget.setRowCount(len(data))

        for row, (key, value) in enumerate(data.items()):
            try:
                # Set the key item in the first column
                self.ui.tableWidget.setItem(row, 0, self.create_centered_item(key))
            except Exception as e:
                print(f"Error setting key item for row {row}: {e}")
                self.set_default_item(self.ui.tableWidget, row, 0, "Unknown")

            try:
                # Process the value if key is in behavior_flag_items
                if key in self.behavior_flag_items:
                    parse_val = self.calculate_binary_values(value)
                    for val in parse_val:
                        if self.behavior_mapping.get(val) == key:
                            value = val
                            break

                # Set the value item in the second column
                self.ui.tableWidget.setItem(row, 1, self.create_centered_item(str(value)))
            except Exception as e:
                print(f"Error processing or setting value for row {row}: {e}")
                self.set_default_item(self.ui.tableWidget, row, 1, "Unknown")

            try:
                # Try to create and set the radio button in the third column
                widget = self.create_radio_button(key, value)
                self.ui.tableWidget.setCellWidget(row, 2, widget)
            except Exception as e:
                print(f"Error creating radio button for row {row} with key '{key}' and value '{value}': {e}")
                widget = self.create_radio_button_cross("gray")
                self.ui.tableWidget.setCellWidget(row, 2, widget)

            try:
                # Try to get the explanation and color
                explanation = self.get_result_value(key, value)
                color = self.get_color(explanation)
                explanation_item = self.create_centered_item(explanation, color)
                self.ui.tableWidget.setItem(row, 3, explanation_item)
            except Exception as e:
                print(f"Error processing explanation or color for row {row} with key '{key}' and value '{value}': {e}")
                self.set_default_item(self.ui.tableWidget, row, 3, "Unknown")
                


    def get_color(self, result_value):
        if result_value == '자율주행': #https://neubility.atlassian.net/wiki/spaces/AUT/pages/174981123/-+Planning+Control
            return QColor('green')
        elif result_value == '수동모드':
            return QColor('red')
        else:
            return QColor('black')

    def get_result_value(self, key, value):
        """
        주어진 key와 value에 따라 결과 값을 반환합니다.

        Args:
            key (str): 데이터의 키
            value (str): 데이터의 값

        Returns:
            str: key와 value에 따른 결과 값
        """

        # 간단한 매핑을 위한 사전 정의
        key_simple_mapping = {
            'V-Mem(NBP/Tot)': 'virtual memory 사용량',
            'P-Mem(NBP/Tot)': 'phsycal memory 사용량',
            'CPU(NBP/Tot)': 'CPU 사용량(NBP/Tot)',
            'CPU-therm': 'CPU 온도',
            'GPU-therm': 'GPU 온도',
            'AUX-therm': 'Aux 온도',
            'AO-therm': 'AO 온도',
            'PMIC-Die': 'PMIC-Die 온도',
            'Tboard_tegra': 'Tboard_tegra 온도',
            'Tdiode_tegr': 'Tdiode_tegra',
            'thermal_fan_est': 'thermal_fan_est 온도',
            'aeb' : '긴급정지',
            '도착지5m근접' : '남은거리추측항법이동',
            '신호대기정지' : '횡단보도앞정지', # https://neubility.atlassian.net/wiki/spaces/AUT/pages/199000115
            '갓길이동정지1' : '종횡방향모두정지', # https://neubility.atlassian.net/wiki/spaces/AUT/pages/199000115
            '갓길이동정지2' : '종방향정지,횡방향정렬',
            '출발전헤딩정렬' : '전역경로방향정렬(횡방향정렬)',
            '보행자감속' : 'localpath1.3m이내보행자검출',
            '자율주행정지신호' : '자율주행정지신호from서버',
            '하강경사로' : '경사로각도3초이상-10도',
            '뉴비감속' : 'localpath1.3m이내뉴비검출',
            '전방주행가능영역없음' : '전방3m,좌우1m내주행영역없음',
            '/way_tag_정보없음' : '미구현상태',
            '/ldm_msg에 3d height(송신)' : '미구현상태', # https://neubility.atlassian.net/wiki/spaces/AUT/pages/138248207/-+Planning+Control#2.3.2.-autonomy_data_failure_flags-(PN-13)
            '120도차이' : '트리거시 mrm-2'
            # '횡단관련없음' : '횡단보도시나리오X',
            # '접근중' : '횡단보도 접근상태 ',
            # '횡단대기' : '횡단 대기상태',
            # '횡단중' : '횡단중',
            # '횡단완료' : '횡단완료상태'
        }

        # 복잡한 매핑 처리
        if key in self.key_mapping:
            mapping = self.key_mapping[key]
            if isinstance(mapping, dict):
                # 키에 해당하는 값 반환, 없으면 '-' 반환
                return mapping.get(value, '-')
            elif callable(mapping):
                # 매핑이 함수인 경우 value를 인자로 호출
                return mapping(value)
        
        # 공통된 'trigger시 mrm-1 정지' 처리
        triggered_mrm_1_keys = ['/cam_f/symbolic_link_alive', 
                             '/cam_fl/symbolic_link_alive', 
                             '/cam_fr/symbolic_link_alive', 
                             '/cam_bl/symbolic_link_alive',
                             '/cam_br/symbolic_link_alive',
                             '/depth_fl/symbolic_link_alive',
                             '/depth_fr/symbolic_link_alive',
                             '/est_states_utm',
                             '/imu_port_status',
                             '/gnss_port_status',
                             '/wheelodom_alive',
                             '/stm_port_status',
                             '/ldm_msg',
                             '/obstacles_tracking_msg',
                             '/est_states_utm',
                             '/ldm_msg에 3d height(수신)',
                             '/odomgyro',
                             '20초1m이하',
                             '경로이탈',
                             'pitch33도',
                             '주행불가영역',
                             'dr_gnss오차',
                             '적재함열림주행',
                             '낭떠러지구간'
                             ]
        if key in triggered_mrm_1_keys:
            return '트리거시 mrm-1'
        
        
        # 공통된 '항상정상주행' 처리
        always_normal_keys = [
            '/cam_fd/symbolic_link_alive',
            '/ekf_alive',
            '/rtcm_alive',
            '/gnss_alive',
            '/imu_alive'
        ]
        if key in always_normal_keys:
            return '항상정상주행'
        
        # 간단한 매핑 처리
        return key_simple_mapping.get(key, '-')


    def update_sensor_table(self, data): # tablewidget_2 / 기능안전데이터
        # print('print', data)
        self.ui.tableWidget_2.setRowCount(len(data))

        for row, (key, value) in enumerate(data.items()):
            try:
                # Set the key item in the first column
                self.ui.tableWidget_2.setItem(row, 0, self.create_centered_item(key))
            except Exception as e:
                print(f"Error setting key item for row {row}: {e}")
                self.set_default_item(self.ui.tableWidget_2, row, 0, "Unknown")

            try:
                # Process the value if key is in sensor_info_items or aut_info_items
                if key in self.sensor_info_items:
                    parse_val = self.calculate_binary_values(value)
                    for val in parse_val:
                        if self.sensor_info_mapping.get(val) == key:
                            value = val
                            break

                elif key in self.aut_info_items:
                    parse_val = self.calculate_binary_values(value)
                    for val in parse_val:
                        if self.aut_info_mapping.get(val) == key:
                            value = val
                            break

                # Set the value item in the second column
                self.ui.tableWidget_2.setItem(row, 1, self.create_centered_item(str(value)))
            except Exception as e:
                print(f"Error processing or setting value for row {row}: {e}")
                self.set_default_item(self.ui.tableWidget_2, row, 1, "Unknown")

            try:
                # Try to create and set the radio button in the third column
                widget = self.create_radio_button(key, value)
                self.ui.tableWidget_2.setCellWidget(row, 2, widget)
            except Exception as e:
                print(f"Error creating radio button for row {row} with key '{key}' and value '{value}': {e}")
                widget = self.create_radio_button_cross("gray")
                self.ui.tableWidget_2.setCellWidget(row, 2, widget)

            try:
                # Try to get the explanation and color
                explanation = self.get_result_value(key, value)
                color = self.get_color(explanation)
                explanation_item = self.create_centered_item(explanation, color)
                self.ui.tableWidget_2.setItem(row, 3, explanation_item)
            except Exception as e:
                print(f"Error processing explanation or color for row {row} with key '{key}' and value '{value}': {e}")
                self.set_default_item(self.ui.tableWidget_2, row, 3, "Unknown")


    def update_cross_table(self, data): # tablewidget_3
    # print('cross', data)
        self.ui.tableWidget_3.setRowCount(len(data))

        for row, (key, value) in enumerate(data.items()):
            try:
                # Set the key item in the first column
                self.ui.tableWidget_3.setItem(row, 0, self.create_centered_item(key))
            except Exception as e:
                print(f"Error setting key item for row {row}: {e}")
                self.set_default_item(self.ui.tableWidget_3, row, 0, "Unknown")

            try:
                # Determine color based on key and value
                color = "green"  # 기본 색상은 녹색
                
                if key == '횡단관련없음' and value == 0.0:
                    color = 'red'
                elif key == '접근중' and value == 1.0:
                    color = 'red'
                elif key == '횡단대기' and value == 2.0:
                    color = 'red'
                elif key == '횡단중' and value == 3.0:
                    color = 'red'
                elif key == '횡단완료' and value == 4.0:
                    color = 'red'
                elif key == '정의된값없음' and value == 5.0:
                    color = 'red'
                
                # Set the value item in the second column
                self.ui.tableWidget_3.setItem(row, 1, self.create_centered_item(str(value)))
            except Exception as e:
                print(f"Error processing or setting value for row {row}: {e}")
                self.set_default_item(self.ui.tableWidget_3, row, 1, "Unknown")

            try:
                # Try to create and set the radio button in the third column
                widget = self.create_radio_button_cross(color)
                self.ui.tableWidget_3.setCellWidget(row, 2, widget)
            except Exception as e:
                print(f"Error creating radio button for row {row} with key '{key}' and value '{value}': {e}")
                widget = self.create_radio_button_cross("gray")
                self.ui.tableWidget_3.setCellWidget(row, 2, widget)

            try:
                # Try to get the explanation and color
                explanation = self.get_result_value(key, value)
                color = self.get_color(explanation)
                explanation_item = self.create_centered_item(explanation, color)
                self.ui.tableWidget_3.setItem(row, 3, explanation_item)
            except Exception as e:
                print(f"Error processing explanation or color for row {row} with key '{key}' and value '{value}': {e}")
                self.set_default_item(self.ui.tableWidget_3, row, 3, "Unknown")

    def update_debug_text_edit(self, data):
        # Clear existing text
        try:
            self.ui.textEdit.clear()
        except Exception as e:
            print(f"Error clearing text edit: {e}")
        
        # Add data items from latest to oldest
        try:
            for key, value in reversed(data.items()):
                self.ui.textEdit.insertPlainText(f'{key}: {value}\n')
        except Exception as e:
            print(f"Error updating text edit with data: {e}")


    # create_radio_button 함수 수정
    def create_radio_button_cross(self, color):
        try:
            radio_button = QRadioButton()

            # Set the color of the radio button
            radio_button.setStyleSheet(f"""
                QRadioButton::indicator {{
                    width: 15px;
                    height: 15px;
                    border-radius: 7px;
                    background-color: {color};
                }}
                QRadioButton::indicator:checked {{
                    background-color: {color};
                }}
            """)

            # Create a widget and set a layout to center the radio button
            widget = QWidget()
            layout = QHBoxLayout(widget)
            layout.addWidget(radio_button)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            widget.setLayout(layout)

            return widget

        except Exception as e:
            print(f"Error creating radio button with color {color}: {e}")
            
            # Handle the exception by creating a default widget with a neutral color
            default_color = "gray"
            radio_button = QRadioButton()
            radio_button.setStyleSheet(f"""
                QRadioButton::indicator {{
                    width: 15px;
                    height: 15px;
                    border-radius: 7px;
                    background-color: {default_color};
                }}
                QRadioButton::indicator:checked {{
                    background-color: {default_color};
                }}
            """)

            # Create a widget and set a layout to center the radio button
            widget = QWidget()
            layout = QHBoxLayout(widget)
            layout.addWidget(radio_button)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            widget.setLayout(layout)

            return widget

    

    def create_centered_item(self, text, color=None):
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignCenter)
        if color:
            item.setForeground(QBrush(color))
        return item
    
    def calculate_binary_values(self, num):
        return [val for val in [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048] if num & val]
    
    def create_issue_flag_dict(self, flags_name_list):
        try:
            return { 2**i : flags_name_list[i] for i in range(len(flags_name_list))}
        except:
            raise ValueError(f"The item {flags_name_list} is not a list. Check it")
        
    def create_radio_button(self, key, value):
        try:
            radio_button = QRadioButton()
            color = "green"  # 기본 색상은 녹색

            # '도착지5m근접', '신호대기정지', 등과 같은 behavior 키에 대한 처리
            if key in self.mrm_flag_items:
                parse_val = self.calculate_binary_values(value)
                self.mrm_mapping

                if any(self.mrm_mapping.get(val) == key for val in parse_val):
                    color = 'red'              

            elif key in self.behavior_flag_items:
                parse_val = self.calculate_binary_values(value)

                if any(self.behavior_mapping.get(val) == key for val in parse_val):
                    color = 'red'                       

            elif key in self.sensor_info_items:
                parse_val = self.calculate_binary_values(value)

                if any(self.sensor_info_mapping.get(val) == key for val in parse_val):
                    color = 'red'

            elif key in self.aut_info_items:
                parse_val = self.calculate_binary_values(value)

                if any(self.aut_info_mapping.get(val) == key for val in parse_val):
                    color = 'red'     

            # '횡단관련없음', '접근중', '횡단대기', '횡단중', '횡단완료', '정의된값없음'에 대한 처리
            elif key in ['횡단관련없음', '접근중', '횡단대기', '횡단중', '횡단완료', '정의된값없음']:
                # print(key, value)
                if value == 0.0:
                    color = 'red'  # 횡단관련없음
                elif value == 1.0:
                    color = 'red'  # 접근중
                elif value == 2.0:
                    color = 'red'  # 횡단대기
                elif value == 3.0:
                    color = 'red'  # 횡단중
                elif value == 4.0:
                    color = 'red'  # 횡단완료
                elif value == 5.0:
                    color = 'red'  # 정의된값없음

            # 'gps'상태 처리
            elif key in 'GPS':
                if int(value) == -1 or int(value) == 0 or int(value) > 300000: #30만 이상 or 0일 경우에 로봇의 현재위치는 0,0
                    color = "yellow"
                elif int(value) > 100:
                    color = "yellow" #인지보정경로 활성화 시작점
            # '배터리 상태', 'GPS', 'aeb'와 같은 다른 키에 대한 처리
            elif key in ['배터리 상태', 'aeb']:
                if (key == '배터리 상태' and int(value) < 20) or (key == 'aeb' and int(value) == 1):
                    color = "red"  # 조건에 맞으면 빨간색

            # 'mrm'에 대한 처리
            elif key == 'mrm':
                color, label_text = self.get_mrm_color_and_label(int(value))
                # print(label_text)
                if label_text:
                    self.ui.label_3.setText(label_text)
                    self.ui.label_3.setStyleSheet(f'color: {color};')  # 라벨 색상 설정   

            #'모드'처리
            elif key == '모드':
                color, label_text = self.get_mode_color_and_label(int(value))
                if label_text:
                    self.ui.label_2.setText(label_text)
                    self.ui.label_2.setStyleSheet(f'color: {color};')  # 라벨 색상 설정   

            # Set the color of the radio button
            radio_button.setStyleSheet(f"""
                QRadioButton::indicator {{
                    width: 15px;
                    height: 15px;
                    border-radius: 7px;
                    background-color: {color};
                }}
                QRadioButton::indicator:checked {{
                    background-color: {color};
                }}
            """)

            # Create a widget and set a layout to center the radio button
            widget = QWidget()
            layout = QHBoxLayout(widget)
            layout.addWidget(radio_button)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            widget.setLayout(layout)
            return widget

        except Exception as e:
            print(f"there is no data in {key}. Skipping now... Exception: {e}")
            radio_button = QRadioButton()
            color = "gray"
            # Create a widget and set a layout to center the radio button in the except block
            widget = QWidget()
            layout = QHBoxLayout(widget)
            layout.addWidget(radio_button)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            widget.setLayout(layout)
            return widget


    def get_mode_color_and_label(self, value):
        if value == 0:
            return 'red', '오퍼레이터'
        elif value == 1:
            return 'blue', '관제'
        elif value == 2:
            return 'green', '자율주행'
        
        else:
            return 'gray', 'Unknown'

    def get_mrm_color_and_label(self, value):
        if value == 0:
            return 'green', 'mrm - 정상주행'
        elif value == 1:
            return 'red', 'mrm - 직진 및 회전정지'
        elif value == 2:
            return 'blue', 'mrm - 직진정지 및 횡방향 유지'
        elif value == 3:
            return 'blue', 'mrm - 직진 부드러운 정지 및 횡방향 유지'
        elif value == 4:
            return 'blue', 'mrm - 갓길 도착 후 종방향 정지'
        elif value == 5:
            return 'blue', 'mrm - 천천히 직진 및 횡방향 즉시정지'
        elif value == 6:
            return 'blue', 'mrm - 직진방향 천천히 정지 및 횡방향 유지'
        else:
            return 'green', ''

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui_app = GuiApp()
    gui_app.show()
    sys.exit(app.exec_())
