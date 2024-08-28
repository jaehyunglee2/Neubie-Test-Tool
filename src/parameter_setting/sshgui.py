
import os
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QVBoxLayout, QWidget, QLabel, QTextEdit, QListWidget, QHBoxLayout, QLineEdit

from resources.arguments import arguments
from src.parameter_setting.sshworker import SSHWorker


class ROSParameterGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Parameter GUI")
        self.resize(600, 1000)

        self.ssh_worker = None
        self.ssh_target_robot = None

        self.init_ui()
        self.select_robot_num = self.ip_combobox.currentText()

    def init_ui(self):
        layout = QVBoxLayout()

        robot_inform = arguments["robot_inform"]
        self.robot_ip_info = { key : robot_inform.get(key)["ip"] for key in robot_inform }
        
        robot_selection_layout = QHBoxLayout()

        ip_selection_layout = QVBoxLayout()
        self.select_robot_label = QLabel('Select Robot:')
        self.ip_combobox = QComboBox()
        self.ip_combobox.addItems(self.robot_ip_info.keys())
        ip_selection_layout.addWidget(self.select_robot_label)
        ip_selection_layout.addWidget(self.ip_combobox)

        robot_selection_layout.addLayout(ip_selection_layout)

        version_layout = QVBoxLayout()
        self.version_button = QPushButton("Check Neubie Version")
        self.version_button.setEnabled(False)
        self.version_button.clicked.connect(self.check_version)
        self.version_label = QLabel("Staus : Idle")
        version_layout.addWidget(self.version_button)
        version_layout.addWidget(self.version_label)

        robot_selection_layout.addLayout(version_layout)

        layout.addLayout(robot_selection_layout)
        self.start_ssh_button = QPushButton("Start SSH Worker")
        self.start_ssh_button.clicked.connect(self.init_sshworker)
        layout.addWidget(self.start_ssh_button)

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
        self.connect_button.setEnabled(False)
        self.connect_button.clicked.connect(self.execute_connection)
        layout.addWidget(self.connect_button)

        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        layout.addWidget(self.status_display)

        self.setLayout(layout)

        self.param_action_combobox.currentIndexChanged.connect(self.toggle_value_input)
        self.param_name_combobox.currentIndexChanged.connect(self.update_item_list)
        self.ip_combobox.currentIndexChanged.connect(self.update_robot_num)
        self.add_item_button.clicked.connect(self.add_item)
        self.remove_item_button.clicked.connect(self.remove_item)

        # 초기 설정
        self.update_item_list()
        self.toggle_value_input()

    def init_sshworker(self):
        
        if self.ssh_target_robot == self.select_robot_num:
            self.status_display.append("이미 동일한 로봇의 ssh가 실행 중 입니다.")
            print("이미 동일한 로봇의 ssh가 실행 중 입니다.")
            return
        
        self.on_ssh_disconnected()

        robot_num = self.select_robot_num
        if not self.ssh_worker:
            robot_ip = arguments["robot_inform"][robot_num]['ip']
            self.ssh_worker = SSHWorker(arguments['target'], robot_ip, arguments['bastion']['host'], os.environ.get('MY_USERNAME'), os.environ.get('MY_PW'))
            self.ssh_target_robot = robot_num
        else:
            print("새로운 기체의 ssh로 연결 합니다")
            self.ssh_worker.close_ssh_connection()
            robot_ip = arguments["robot_inform"][robot_num]['ip']
            self.ssh_worker = SSHWorker(arguments['target'], robot_ip, arguments['bastion']['host'], os.environ.get('MY_USERNAME'), os.environ.get('MY_PW'))
            self.ssh_target_robot = robot_num

        self.ssh_worker.finished_signal.connect(self.on_ssh_finished)
        self.ssh_worker.result_signal.connect(self.show_result)
        self.ssh_worker.version_signal.connect(self.get_version_signal)
        self.ssh_worker.update_signal.connect(self.update_status_display)

        self.ssh_worker.start()

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

    def update_robot_num(self):
        self.select_robot_num = self.ip_combobox.currentText()


    def add_item(self):
        current_item = self.item_selection_combobox.currentText()
        if current_item:
            self.items_list.addItem(current_item)

    def remove_item(self):
        for item in self.items_list.selectedItems():
            self.items_list.takeItem(self.items_list.row(item))

    def execute_connection(self):
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

        if param_action == "SET":
            self.ssh_worker.set_param("/NBPlatform", param_name, param_value)
        elif param_action == "GET":
            self.ssh_worker.get_param("/NBPlatform", param_name)


    def update_status_display(self, message):
        if "Secsh channel 0 open FAILED: Connection timed out: Connect failed" in message:
            self.ssh_target_robot = None
        self.status_display.append(message)

    def show_result(self, result):
        self.status_display.append(f"Result:\n{result}")

    def on_ssh_disconnected(self):
        self.start_ssh_button.setEnabled(False)
        self.version_button.setEnabled(False)
        self.connect_button.setEnabled(False)

    def on_ssh_finished(self):
        self.start_ssh_button.setEnabled(True)
        self.version_button.setEnabled(True)
        self.connect_button.setEnabled(True)
        
    def check_version(self):
        self.ssh_worker.get_version()

    def get_version_signal(self, message):
        self.version_label.setText(message)

    def closeEvent(self, event):
        # 창이 닫힐 때 호출되는 메서드
        if self.ssh_worker is not None:
            self.ssh_worker.close_ssh_connection()
            self.ssh_worker = None

        event.accept()

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = ROSParameterGUI()
    window.show()
    sys.exit(app.exec_())
