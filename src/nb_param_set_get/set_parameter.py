import sys
import json
import os
import paramiko
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QVBoxLayout, QWidget, QLabel, QTextEdit, QListWidget, QHBoxLayout, QLineEdit
from PyQt5.QtCore import QThread, pyqtSignal
from resources.arguments import arguments

class SSHWorker(QThread):
    update_signal = pyqtSignal(str)
    result_signal = pyqtSignal(str)

    def __init__(self, ip_info, selected_ip, bastion_host, user_name, user_password, param_action, param_name, param_value=None):
        super().__init__()
        self.ip_info = ip_info
        self.selected_ip = selected_ip
        self.bastion_host = bastion_host
        self.user_name = user_name
        self.user_password = user_password
        self.param_action = param_action
        self.param_name = param_name
        self.param_value = param_value

    def run(self):
        client = None
        target_client = None
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            self.update_signal.emit("Connecting to bastion host...")
            client.connect(hostname=self.bastion_host, username=self.user_name, password=self.user_password)
            self.update_signal.emit("Connected to bastion host.")

            self.update_signal.emit(f"Establishing SSH connection to target host {self.selected_ip}...")
            transport = client.get_transport()
            dest_addr = (self.selected_ip, 22)
            local_addr = ('localhost', 0)
            sock = transport.open_channel("direct-tcpip", dest_addr, local_addr)

            target_client = paramiko.SSHClient()
            target_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            target_client.connect(self.selected_ip, username=self.ip_info['target']['user'], password=self.ip_info['target']['password'], sock=sock)
            self.update_signal.emit("SSH connection established to target host.")

            # 프롬프트 표시
            shell = target_client.invoke_shell()
            self.update_signal.emit("Shell invoked. Waiting for prompt...")

            # 프롬프트를 읽어와서 표시
            prompt = shell.recv(1024).decode('utf-8')
            self.result_signal.emit(prompt)

            # ROS 2 명령 실행
            if self.param_action == "SET":
                #ros2 param set /NBPlatform 에 대한 세팅 (/NBPlatform에 대한 명령어)
                command = f". /opt/ros/galactic/setup.bash && ros2 param set /NBPlatform {self.param_name} \"{self.param_value}\""
            elif self.param_action == "GET":
                command = f". /opt/ros/galactic/setup.bash && ros2 param get /NBPlatform {self.param_name}"

            self.update_signal.emit(f"Executing: {command}")
            stdin, stdout, stderr = target_client.exec_command(command)
            result = stdout.read().decode() + stderr.read().decode()
            self.result_signal.emit(result)

        except Exception as e:
            self.update_signal.emit(f"Error: {e}")
        finally:
            if target_client:
                target_client.close()
            if client:
                client.close()
            self.update_signal.emit("SSH connection closed.")

class ROSParameterGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Parameter GUI")
        self.resize(600, 1000)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        robot_inform = arguments["robot_inform"]
        self.robot_ip_info = { key : robot_inform.get(key)["ip"] for key in robot_inform }
        
        self.ip_combobox = QComboBox()
        self.ip_combobox.addItems(self.robot_ip_info.keys())
        layout.addWidget(QLabel('Select Robot:'))
        layout.addWidget(self.ip_combobox)

        self.param_action_combobox = QComboBox()
        self.param_action_combobox.addItems(["SET", "GET"])
        layout.addWidget(QLabel('Select Action:'))
        layout.addWidget(self.param_action_combobox)

        self.param_name_combobox = QComboBox()
        self.param_name_combobox.addItems([
            "local_dynamic_map_processing.drivable_class_list", #세팅 필요한 파라미터 리스트 추가하면 됨
            "local_dynamic_map_processing.undrivable_class_list",
            "yield_behavior.campus.enabled",
            "yield_behavior.camping.enabled",
            "yield_behavior.golf.enabled",
            "yield_behavior.campus.class_list",
            "yield_behavior.camping.class_list",
            "yield_behavior.golf.class_list",
            "data_9#노면기반경로보정[-1.0(기본값)|1.0(기준경로)|2.0(전역경로)]",
            "rviz_publisher_params.pub_enable#주행가능영역확인토픽",
            "max_velocity_mps#Permitted Longitudinal Max Speed",
            "velocity_profile_target_speed#경로 기울기에 따른 속도 프로파일 변수",
            "velocity_profile_target_gradient#경로 기울기에 따른 속도 프로파일 변수",
            "behavior_planner_params.perception_crosswalk_resume_flag_enabled#횡단보도자동횡단"
            

            # Add more parameters as needed
        ])
        layout.addWidget(QLabel('Select Parameter:'))
        layout.addWidget(self.param_name_combobox)

        self.item_selection_combobox = QComboBox()
        layout.addWidget(QLabel('Select Item:'))
        layout.addWidget(self.item_selection_combobox)

        self.items_list = QListWidget()
        self.items_list.setSelectionMode(QListWidget.ExtendedSelection)
        layout.addWidget(QLabel('Selected Items (for SET only):'))
        layout.addWidget(self.items_list)

        self.add_item_button = QPushButton("Add Item")
        self.remove_item_button = QPushButton("Remove Selected Item")
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.add_item_button)
        button_layout.addWidget(self.remove_item_button)
        layout.addLayout(button_layout)

        self.param_value_input = QLineEdit()  # 텍스트 입력 필드 추가
        layout.addWidget(QLabel('Input Value:'))
        layout.addWidget(self.param_value_input)

        self.connect_button = QPushButton("Execute")
        self.connect_button.clicked.connect(self.connect_to_ip)
        layout.addWidget(self.connect_button)

        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        layout.addWidget(self.status_display)

        self.setLayout(layout)

        self.param_action_combobox.currentIndexChanged.connect(self.toggle_value_input)
        self.param_name_combobox.currentIndexChanged.connect(self.update_item_list)
        self.add_item_button.clicked.connect(self.add_item)
        self.remove_item_button.clicked.connect(self.remove_item)

        # 초기 설정
        self.update_item_list()
        self.toggle_value_input()

    def toggle_value_input(self):
        selected_param = self.param_name_combobox.currentText()
        if self.param_action_combobox.currentText() == "GET":
            self.items_list.setDisabled(True)
            self.item_selection_combobox.setDisabled(True)
            self.add_item_button.setDisabled(True)
            self.remove_item_button.setDisabled(True)
            self.param_value_input.setDisabled(True)  # 텍스트 입력 필드 비활성화
        else:
            self.items_list.setEnabled(True)
            self.item_selection_combobox.setEnabled(True)
            self.add_item_button.setEnabled(True)
            self.remove_item_button.setEnabled(True)
            if selected_param == "max_velocity_mps#Permitted Longitudinal Max Speed" or \
                selected_param == "velocity_profile_target_speed#경로 기울기에 따른 속도 프로파일 변수" or \
                    selected_param == "velocity_profile_target_gradient#경로 기울기에 따른 속도 프로파일 변수":
                self.param_value_input.setEnabled(True)  # 텍스트 입력 필드 활성화
            else:
                self.param_value_input.setDisabled(True)  # 텍스트 입력 필드 비활성화

    def update_item_list(self):
        self.items_list.clear()
        self.item_selection_combobox.clear()
        selected_param = self.param_name_combobox.currentText()
        #selected_param에 따라서 생성될 아이템 설정
        if selected_param == "local_dynamic_map_processing.drivable_class_list" or selected_param == "local_dynamic_map_processing.undrivable_class_list":
            items = ["background", "blocks", "colored_road", 
                     "cement_asphalt", "crosswalk", "speed_bumps"
                     , "manhole", "curbs", "grass", "soil_gravel", "grating",
                     "guide_blocks", "rock"]
        elif selected_param == "yield_behavior.campus.enabled" or \
            selected_param == "yield_behavior.camping.enabled" or \
            selected_param == "yield_behavior.golf.enabled" or \
            selected_param == "behavior_planner_params.perception_crosswalk_resume_flag_enabled#횡단보도자동횡단" or \
            selected_param == "rviz_publisher_params.pub_enable#주행가능영역확인토픽": #https://neubility.atlassian.net/browse/ATNM-3356, https://neubilityhq.slack.com/archives/C036UFV0SDD/p1719470933937389?thread_ts=1719382311.344959&cid=C036UFV0SDD
            items = ["true", "false"]
        elif selected_param == "yield_behavior.campus.class_list" or \
            selected_param == "yield_behavior.camping.class_list":
            items = ["car", "bus", "truck", "person"]
        elif selected_param == "yield_behavior.golf.class_list":
            items = ["car", "bus", "truck", "cart", "person"]
        elif selected_param == "data_9#노면기반경로보정[-1.0(기본값)|1.0(기준경로)|2.0(전역경로)]":
            items = ["-1.0", "1.0", "2.0"] #https://neubility.atlassian.net/wiki/spaces/AUT/pages/502071446/-+Planning+Control
        elif selected_param == "max_velocity_mps#Permitted Longitudinal Max Speed" or \
            selected_param == "velocity_profile_target_speed#경로 기울기에 따른 속도 프로파일 변수" or \
                selected_param == "velocity_profile_target_gradient#경로 기울기에 따른 속도 프로파일 변수":
            items = []  # max_velocity_mps의 경우 사용자가 직접 값을 입력하므로 빈 리스트
        else:
            items = []

        self.item_selection_combobox.addItems(items)
        self.items_list.addItems(items)

    def add_item(self):
        current_item = self.item_selection_combobox.currentText()
        if current_item:
            self.items_list.addItem(current_item)

    def remove_item(self):
        for item in self.items_list.selectedItems():
            self.items_list.takeItem(self.items_list.row(item))

    def connect_to_ip(self):
        selected_robot_number = self.ip_combobox.currentText()
        selected_ip = self.robot_ip_info[selected_robot_number]
        bastion_host = self.ip_info['bastion']['host']
        user_name = os.environ.get('MY_USERNAME')
        user_password = os.environ.get('MY_PW')
        param_action = self.param_action_combobox.currentText()
        param_name = self.param_name_combobox.currentText()

        # Get selected items from the QListWidget
        selected_items = []
        for index in range(self.items_list.count()):
            selected_items.append(self.items_list.item(index).text())

        # Generate param_value based on selected items and param_name
        if param_action == "SET":
            if param_name in [
                "yield_behavior.campus.enabled",
                "yield_behavior.camping.enabled",
                "yield_behavior.golf.enabled",
                "rviz_publisher_params.pub_enable#주행가능영역확인토픽",
                "behavior_planner_params.perception_crosswalk_resume_flag_enabled#횡단보도자동횡단"
            ]:
                param_value = selected_items[0] if selected_items else "false"
            elif param_name == "data_9#노면기반경로보정[-1.0(기본값)|1.0(기준경로)|2.0(전역경로)]":
                param_value = selected_items[0] if selected_items else "-1.0"
            elif param_name == "max_velocity_mps#Permitted Longitudinal Max Speed" or \
                param_name == "velocity_profile_target_speed#경로 기울기에 따른 속도 프로파일 변수" or \
                    param_name == "velocity_profile_target_gradient#경로 기울기에 따른 속도 프로파일 변수": 
                param_value = self.param_value_input.text()  # 사용자가 입력한 값을 사용
            else:
                param_value = f"[{', '.join([repr(item) for item in selected_items])}]"
        else:
            param_value = None

        # Remove anything after # in param_name and param_value (#포함 모두 삭제)
        param_name = param_name.split('#')[0].strip()
        if param_value:
            param_value = param_value.split('#')[0].strip()

        self.worker = SSHWorker(self.ip_info, selected_ip, bastion_host, user_name, user_password, param_action, param_name, param_value)
        self.worker.update_signal.connect(self.update_status_display)
        self.worker.result_signal.connect(self.show_result)
        self.worker.start()

    def update_status_display(self, message):
        self.status_display.append(message)

    def show_result(self, result):
        self.status_display.append(f"Result:\n{result}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROSParameterGUI()
    window.show()
    sys.exit(app.exec_())
