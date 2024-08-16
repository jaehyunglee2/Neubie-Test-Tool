import rclpy
import time
from rclpy.node import Node
from autonomy_ros2_message.msg import LoggingMsg, ServiceTarget
from PyQt5.QtCore import QObject, pyqtSignal
from std_msgs.msg import Bool
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSlot
from PyQt5.QtWidgets import QApplication, QWidget, QTableWidget, QTableWidgetItem, QLineEdit, QHeaderView, QMessageBox
from python_qt_binding import loadUi

import os
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
import requests

vl_sys_path = os.path.dirname(os.path.abspath(__file__))
server_setting_list = ["/mapping/api/set_robot_using_map", "/mapping/api/get_using_map"]
server_setting_list = ["로봇이 사용할 맵 세팅하기", "로봇이 사용하는 맵 불러오기"]

robot_serial_no = {
    " " : "로봇을 입력해주세요",
    "09" : "NAAAKA1-1220902003",
    "12" : "NAAAKA1-1220902006",
    "24" : "NAAAKA1-1220916010",
    "로봇 추가 바랍니다" : "로봇 추가 바랍니다"
}

target_dict =  { **{ " " : "로봇을 입력해주세요" }, **robot_serial_no, **{ "로봇 추가 바랍니다" : "로봇 추가 바랍니다" } }
prod_server = "http://1.229.180.178:27000"
get_map_list = f'{prod_server}/mapping/api/get_map_list'

class vlThread(QThread):
    def __init__(self):
        super().__init__()
        self.worker = vlServiceSettingWorker()

    def run(self):
        self.worker.run()

class vlServiceSettingWorker(QObject):

    def __init__(self):
        super().__init__()
        self.ui = QWidget()  # UI가 로드될 QWidget 인스턴스 생성
        ui_file = os.path.join(vl_sys_path, 'vlwidget.ui')
        loadUi(ui_file, self.ui)  # loadUi 함수를 통해 UI 파일 로드

        self.init_widget()

    def init_widget(self):
        self.request_map_name = list()
        self.ui.robotNo.addItems(target_dict.keys())
        self.ui.robotNo.currentIndexChanged.connect(self.updateSerialNoLabel)

        self.ui.paramOn.clicked.connect(self.param_on_btn_clicked)
        self.ui.paramOff.clicked.connect(self.param_off_btn_clicked)
        self.ui.severSetBtn.clicked.connect(self.server_set_btn_clicked)
        self.ui.severSetBtn_2.clicked.connect(self.fetch_and_display_data)

        # ProgressBar 초기화
        self.ui.progressBar.setValue(0)
        self.ui.statusLabel.setText('Idle')
        self.ui.statusLabel_2.setText('Waiting for work')
        
        self.ui.serverSet.addItems(server_setting_list)
        self.fetch_and_display_data()
        self.ui.serverSet.currentIndexChanged.connect(self.update_setting_table)
        self.update_setting_table()

    def update_setting_table(self):
        # 선택된 옵션에 따라 테이블 위젯을 업데이트
        selected_option = self.ui.serverSet.currentText()

        if selected_option == "로봇이 사용할 맵 세팅하기":
            # 로봇 ID와 요청 맵 이름 필드를 추가
            self.ui.settingTable.setRowCount(2)
            self.ui.settingTable.setColumnCount(2)
            self.ui.settingTable.setHorizontalHeaderLabels(["Field", "Value"])

            self.ui.settingTable.setItem(0, 0, QTableWidgetItem("robot_id"))
            self.ui.settingTable.setItem(1, 0, QTableWidgetItem("request_map_name"))

            robot_id_input = QLineEdit()
            request_map_name_input = QLineEdit()

            # 입력 필드 수정 불가 설정
            robot_id_input.setReadOnly(False)
            request_map_name_input.setReadOnly(False)

            self.ui.settingTable.setCellWidget(0, 1, robot_id_input)
            self.ui.settingTable.setCellWidget(1, 1, request_map_name_input)

        elif selected_option == "로봇이 사용하는 맵 불러오기":
            # 로봇 ID 필드만 추가
            self.ui.settingTable.setRowCount(1)
            self.ui.settingTable.setColumnCount(2)
            self.ui.settingTable.setHorizontalHeaderLabels(["Field", "Value"])

            self.ui.settingTable.setItem(0, 0, QTableWidgetItem("robot_id"))

            robot_id_input = QLineEdit()

            # 입력 필드 수정 불가 설정
            robot_id_input.setReadOnly(False)

            self.ui.settingTable.setCellWidget(0, 1, robot_id_input)

        # 테이블 수정 불가 설정
        for row in range(self.ui.settingTable.rowCount()):
            for column in range(self.ui.settingTable.columnCount()):
                item = self.ui.settingTable.item(row, column)
                if item:
                    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

        # Field 열 크기는 고정하고, Value 열이 나머지 공간을 채우도록 설정
        self.ui.settingTable.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.ui.settingTable.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)

    def display_result_in_table(self, data):
        if isinstance(data, dict):
            self.ui.resultTable.setRowCount(len(data))
            self.ui.resultTable.setColumnCount(2)
            self.ui.resultTable.setHorizontalHeaderLabels(["Key", "Value"])

            for row, (key, value) in enumerate(data.items()):
                self.ui.resultTable.setItem(row, 0, QTableWidgetItem(str(key)))
                self.ui.resultTable.setItem(row, 1, QTableWidgetItem(str(value)))

        elif isinstance(data, list):
            self.ui.resultTable.setRowCount(len(data))
            self.ui.resultTable.setColumnCount(len(data[0]) if data else 0)

            for row, item in enumerate(data):
                if isinstance(item, dict):
                    for col, (key, value) in enumerate(item.items()):
                        self.ui.resultTable.setHorizontalHeaderItem(col, QTableWidgetItem(str(key)))
                        self.ui.resultTable.setItem(row, col, QTableWidgetItem(str(value)))
                else:
                    self.ui.resultTable.setItem(row, 0, QTableWidgetItem(str(item)))
        elif isinstance(data, str):
            self.ui.resultTable.setRowCount(1)
            self.ui.resultTable.setColumnCount(1)
            self.ui.resultTable.setHorizontalHeaderLabels(["Value"])
            self.ui.resultTable.setItem(0, 0, QTableWidgetItem(data))

        else:
            # 만약 예상하지 못한 타입의 데이터가 들어왔을 때 처리
            self.ui.resultTable.setRowCount(1)
            self.ui.resultTable.setColumnCount(1)
            self.ui.resultTable.setHorizontalHeaderLabels(["Value"])
            self.ui.resultTable.setItem(0, 0, QTableWidgetItem("Unsupported data format"))

        # 테이블 수정 불가 설정
        for row in range(self.ui.resultTable.rowCount()):
            for col in range(self.ui.resultTable.columnCount()):
                item = self.ui.resultTable.item(row, col)
                if item:
                    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

        self.ui.resultTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def server_set_btn_clicked(self):
        # settingTable의 값을 dict로 변환하여 출력
        selected_option = self.ui.serverSet.currentText()
        table_data = {}
        for row in range(self.ui.settingTable.rowCount()):
            field_item = self.ui.settingTable.item(row, 0)
            value_widget = self.ui.settingTable.cellWidget(row, 1)

            if field_item and value_widget:
                field = field_item.text()
                value = value_widget.text()
                table_data[field] = value

        print(selected_option)
        print(table_data)  # dict 형태로 출력

        if selected_option == "로봇이 사용할 맵 세팅하기":
            serial_no = None
            request_map = None
            if table_data.get('robot_id') in robot_serial_no.keys():
                serial_no = robot_serial_no.get(table_data.get('robot_id'))
                print("로봇 시리얼 번호", serial_no)
            else:
                err = "잘못된 로봇 번호 입력"
                self.display_result_in_table(err)
                raise ValueError(err)

            if len(self.request_map_name) == 0:
                err = "서버가 동작을 하지 않아 Map 정보 못받음"
                self.display_result_in_table(err)
                raise ValueError(err)
            elif table_data.get('request_map_name') in self.request_map_name:
                request_map = table_data.get('request_map_name')
            else:
                err = "잘못된 지도정보 입력"
                self.display_result_in_table(err)
                raise ValueError(err)
            url = f"http://1.229.180.178:27000/mapping/api/{serial_no}/set_robot_using_map?request_map_name={request_map}"

            # POST 요청 보내기
            try:

                log = None
                response = requests.post(url, timeout=10)
                if response.status_code == 200:
                    log = f"맵 세팅 성공: {response.json()}"
                else:
                    log = f"맵 세팅 실패: {response.status_code}, {response.json()}"
                print(log)
                self.display_result_in_table(log)
            except requests.exceptions.Timeout:
                self.display_result_in_table("서버가 응답하지 않습니다.")
            except requests.RequestException as e:
                print("POST 요청 중 오류 발생:", e)
                self.display_result_in_table(e)

        elif selected_option == "로봇이 사용하는 맵 불러오기":
            serial_no = None
            if table_data.get('robot_id') in robot_serial_no.keys():
                serial_no = robot_serial_no.get(table_data.get('robot_id'))
                print("로봇 시리얼 번호", serial_no)
            else:
                err = "잘못된 로봇 번호 입력"
                self.display_result_in_table(err)
                raise ValueError(err)
            
            url = f"http://1.229.180.178:27000/mapping/api/{serial_no}/get_using_map"
            # GET 요청 보내기
            try:
                log = None
                response = requests.get(url, timeout=10)
                if response.status_code == 200:
                    log = f"맵 불러오기 성공: {response.json()}"
                else:
                    log = f"맵 불러오기 실패: {response.status_code}, {response.json()}"
                print(log)
                self.display_result_in_table(log)
            except requests.exceptions.Timeout:
                self.display_result_in_table("서버가 응답하지 않습니다.")
            except requests.RequestException as e:
                print("GET 요청 중 오류 발생:", e)  
                self.display_result_in_table(e)

    def param_on_btn_clicked(self):
        # 작업 시작 시 ProgressBar 초기화
        self.ui.progressBar.setValue(0)
        self.ui.statusLabel.setText('Processing...')
        self.ui.statusLabel.setStyleSheet('color: black')

        # 작업 실행
        QTimer.singleShot(200, self.process_on_commands)
        
    def process_on_commands(self):
        try:
            # 첫 번째 명령어 실행
            print("Executing: Set visual_localization_server_url")
            process1 = subprocess.Popen(['ros2', 'param', 'set', '/visual_localization_client', 'visual_localization_server_url', f'"{prod_server}/localization/api/robot"'])
            self.processes.append(process1)
            self.ui.statusLabel_2.setText('Set visual_localization_server_url')
            self.ui.progressBar.setValue(33)
            process1.wait()

            # 두 번째 명령어 실행
            print("Executing: Set position_bounding")
            process2 = subprocess.Popen(['ros2', 'param', 'set', '/visual_localization_client', 'position_bounding', 'true'])
            self.processes.append(process2)
            self.ui.statusLabel_2.setText('Set position_bounding')
            self.ui.progressBar.setValue(66)
            process2.wait()

            # 세 번째 명령어 실행
            print("Executing: Set visual_localization_needed")
            process3 = subprocess.Popen(['ros2', 'param', 'set', '/visual_localization_client', 'visual_localization_needed', 'true'])
            self.processes.append(process3)
            self.ui.statusLabel_2.setText('Set visual_localization_needed')
            self.ui.progressBar.setValue(100)
            process3.wait()


            # 작업 성공 시
            self.ui.statusLabel.setText('Completed')
            self.ui.statusLabel_2.setText('Ready to Use Visual_localization')
            self.ui.statusLabel.setStyleSheet('color: green')
        except subprocess.CalledProcessError:
            # 오류 발생 시
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)

    def param_off_btn_clicked(self):
        # 작업 시작 시 ProgressBar 초기화
        self.ui.progressBar.setValue(0)
        self.ui.statusLabel.setText('Processing...')
        self.ui.statusLabel.setStyleSheet('color: black')

        # 작업 실행
        QTimer.singleShot(100, self.process_off_commands)

    def fetch_and_display_data(self):
        try:
            # GET 요청을 보내고 응답을 JSON으로 파싱
            response = requests.get(get_map_list, headers={'accept': 'application/json'}, timeout=5)
            response.raise_for_status()
            self.request_map_name = response.json()
            print(self.request_map_name)

            # settingTable_2 초기화
            self.ui.settingTable_2.setRowCount(0)  # 기존 행 제거
            self.ui.settingTable_2.setColumnCount(1)  # 컬럼 수 설정
            self.ui.settingTable_2.setHorizontalHeaderLabels(["사용가능한 VL Map 명"])

            for row_index, row_data in enumerate(self.request_map_name):
                self.ui.settingTable_2.insertRow(row_index)
                table_item = QTableWidgetItem(str(row_data))
                self.ui.settingTable_2.setItem(row_index, 0, table_item)

            self.ui.settingTable_2.setEditTriggers(QTableWidget.NoEditTriggers)  # 전체 테이블 수정 불가능 설정
            self.ui.settingTable_2.setWordWrap(True)  # 텍스트 줄바꿈 설정
            self.ui.settingTable_2.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)  # 수평 스크롤
            self.ui.settingTable_2.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)  # 수직 스크롤
            self.ui.settingTable_2.resizeColumnsToContents()
            self.ui.settingTable_2.resizeRowsToContents()
            self.ui.settingTable_2.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)

        except requests.exceptions.Timeout:
            self.show_error_message("서버가 응답하지 않습니다. 서버가 죽은 것 같습니다. \nlocalization Cell의 기백님께에 요청 바랍니다. ")
        except requests.RequestException as e:
            self.show_error_message(f"HTTP 요청 중 오류 발생: {e}")

    def process_off_commands(self):
        try:
            # ROS 2 명령어 실행
            subprocess.run(['ros2', 'param', 'set', '/visual_localization_client', 'visual_localization_needed', 'false'], check=True)
            self.ui.statusLabel_2.setText('Set visual_localization_needed')
            self.ui.progressBar.setValue(50)
            time.sleep(1)

            subprocess.run(['ros2', 'param', 'set', '/visual_localization_client', 'position_bounding', 'false'], check=True)
            self.ui.statusLabel_2.setText('Set position_bounding')
            self.ui.progressBar.setValue(100)

            # 작업 성공 시
            self.ui.statusLabel.setText('Completed')
            self.ui.statusLabel_2.setText('Stop using Visual_localization')
            self.ui.statusLabel.setStyleSheet('color: green')
        except subprocess.CalledProcessError:
            # 오류 발생 시
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)

    def updateSerialNoLabel(self):
        selected_robotNo = self.ui.robotNo.currentText()
        self.ui.serialNo.setText(target_dict[selected_robotNo])

    def run(self):
        self.ui.show()  # UI를 표시하는 함수
    def show_error_message(self, message):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setWindowTitle("오류")
        msg_box.setText(message)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    
    thread = vlThread()
    thread.start()

    sys.exit(app.exec_())