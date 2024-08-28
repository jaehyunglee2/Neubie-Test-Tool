from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSlot
from PyQt5.QtWidgets import QApplication, QWidget, QTableWidget, QTableWidgetItem, QLineEdit, QHeaderView, QMessageBox
from python_qt_binding import loadUi

import os
from src.parameter_setting.sshworker import SSHWorker
from resources.arguments import arguments
import requests

# 자세한 설명은 https://neubility.atlassian.net/wiki/spaces/AUT/pages/890077319/Visual+Localization 참조
vl_sys_path = os.path.dirname(os.path.abspath(__file__))
server_setting_list = ["/mapping/api/set_robot_using_map", "/mapping/api/get_using_map"]
server_setting_list = ["로봇이 사용할 맵 세팅하기", "로봇이 사용하는 맵 불러오기"]

robot_inform = arguments["robot_inform"]

robot_serial_no = { key : robot_inform.get(key)["serial"] for key in robot_inform }
robot_ip_no = { key : robot_inform.get(key)["ip"] for key in robot_inform }

target = arguments["target"]

target_dict =  { **{ " " : "로봇을 입력해주세요" }, **robot_serial_no }
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
        self.last_result = False
        self.ssh_worker = None
        self.robot_no = None
        self.ssh_robot_no = None
        ui_file = os.path.join(vl_sys_path, 'vlwidget.ui')
        loadUi(ui_file, self.ui)  # loadUi 함수를 통해 UI 파일 로드

        self.init_widget()

    def init_widget(self):
        self.request_map_name = list()
        self.ui.robotNo.addItems(target_dict.keys())
        self.ui.robotNo.currentIndexChanged.connect(self.updateSerialNoLabel)

        self.ui.paramOn.clicked.connect(self.param_on_btn_clicked)
        self.ui.paramOff.clicked.connect(self.param_off_btn_clicked)
        self.ui.resetButton.clicked.connect(self.param_reset_btn_clicked)
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
            selected_robotNo = self.ui.robotNo.currentText()
            robot_id_input.setText(target_dict[selected_robotNo])
            # 입력 필드 수정 불가 설정
            robot_id_input.setReadOnly(True)
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
            selected_robotNo = self.ui.robotNo.currentText()
            robot_id_input.setText(target_dict[selected_robotNo])
            # 입력 필드 수정 불가 설정
            robot_id_input.setReadOnly(True)

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
        waiting = "요청을 기다리는 중"
        self.display_result_in_table(waiting)

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

        serial_no = table_data.get('robot_id')
        
        if selected_option == "로봇이 사용할 맵 세팅하기":
            if not serial_no or serial_no not in robot_serial_no.values():
                err = "잘못된 시리얼 번호 입력"
                self.display_result_in_table(err)
                return

            if len(self.request_map_name) == 0:
                err = "서버가 동작을 하지 않아 Map 정보 못받음"
                self.display_result_in_table(err)
                return
            
            request_map = table_data.get('request_map_name')
            if request_map not in self.request_map_name:
                err = "잘못된 지도정보 입력"
                self.display_result_in_table(err)
                return

            url = f"{prod_server}/mapping/api/{serial_no}/set_robot_using_map?request_map_name={request_map}"
            request_type = "POST"

        elif selected_option == "로봇이 사용하는 맵 불러오기":
            if not serial_no or serial_no not in robot_serial_no.values():
                err = "잘못된 시리얼 번호 입력"
                self.display_result_in_table(err)
                return
            
            url = f"{prod_server}/mapping/api/{serial_no}/get_using_map"
            request_type = "GET"

        # 비동기 요청을 처리할 스레드를 시작합니다.
        self.server_thread = ServerRequestThread(url, request_type)
        self.server_thread.result_signal.connect(self.display_result_in_table)
        self.server_thread.start()

    def param_on_btn_clicked(self):
        self.ui.paramOn.setEnabled(False)
        # 작업 시작 시 ProgressBar 초기화
        self.ui.progressBar.setValue(0)
        self.ui.statusLabel.setText('Processing...')
        self.ui.statusLabel.setStyleSheet('color: black')

        # 작업 실행
        QTimer.singleShot(200, self.process_on_commands)
        
    def process_on_commands(self):
        selected_robot_key = self.ui.robotNo.currentText()
        robot_ip = robot_ip_no.get(selected_robot_key)

        if not robot_ip:
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)
            self.show_error_message(f"로봇 IP를 찾을 수 없습니다: {selected_robot_key}")
            self.ui.paramOn.setEnabled(True) 
            return

        # 명령어 리스트 설정
        commands = [
            ("visual_localization_server_url", f"{prod_server}/localization/api/robot", 33),
            ("position_bounding", 'true', 66),
            ("visual_localization_needed", 'true', 100)
        ]

        # SSHWorker를 사용하여 SSH 연결 설정
        self.setting_ssh_worker(robot_ip)

        if not self.ssh_worker.target_client:
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.show_error_message("SSH 연결 설정에 실패했습니다.")
            self.ui.paramOn.setEnabled(True) 
            return

        try:
            for param_name, param_value, progress in commands:
                print(f"Setting: {param_name} to {param_value}")
                self.ui.statusLabel_2.setText(f"Setting: {param_name} to {param_value}")
                success = self.execute_ssh_command(param_name, param_value, progress)
                if not success:
                    raise RuntimeError(f"Failed to set {param_name}")

            # 작업 성공 시
            self.ui.statusLabel.setText('Completed')
            self.ui.statusLabel_2.setText('Ready to Use Visual_localization')
            self.ui.statusLabel.setStyleSheet('color: green')

        except Exception as e:
            # 오류 발생 시 바로 처리
            print(f"An error occurred: {e}")
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)
        finally:
            self.ssh_worker.close_ssh_connection()  # SSH 연결 종료
            self.ui.paramOn.setEnabled(True) 

    def execute_ssh_command(self, param_name, param_value, progress_value):
        
        # 파라미터 설정
        self.ssh_worker.set_param("/visual_localization_client", param_name, param_value)

        # SSH 작업이 완료될 때까지 대기
        self.ssh_worker.wait()

        # 진행률 업데이트
        self.ui.progressBar.setValue(progress_value)

        # handle_ssh_result에서 설정된 self.last_result 반환
        return self.last_result

    def param_off_btn_clicked(self):
        # 작업 시작 시 ProgressBar 초기화
        self.ui.paramOff.setEnabled(False) 
        self.ui.progressBar.setValue(0)
        self.ui.statusLabel.setText('Processing...')
        self.ui.statusLabel.setStyleSheet('color: black')

        # 작업 실행
        QTimer.singleShot(100, self.process_off_commands)

    def process_off_commands(self):
        selected_robot_key = self.ui.robotNo.currentText()
        robot_ip = robot_ip_no.get(selected_robot_key)

        if not robot_ip:
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)
            self.show_error_message(f"로봇 IP를 찾을 수 없습니다: {selected_robot_key}")
            self.ui.paramOff.setEnabled(True) 
            return

        # 명령어 리스트 설정
        commands = [
            ("position_bounding", 'false', 50),
            ("visual_localization_needed", 'false', 100)
        ]

        # SSHWorker를 사용하여 SSH 연결 설정
        self.setting_ssh_worker(robot_ip)

        if not self.ssh_worker.target_client:
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.show_error_message("SSH 연결 설정에 실패했습니다.")
            return

        try:
            for param_name, param_value, progress in commands:
                print(f"Setting: {param_name} to {param_value}")
                self.ui.statusLabel_2.setText(f"Setting: {param_name} to {param_value}")
                success = self.execute_ssh_command(param_name, param_value, progress)
                if not success:
                    raise RuntimeError(f"Failed to set {param_name}")

            # 작업 성공 시
            self.ui.statusLabel.setText('Completed')
            self.ui.statusLabel_2.setText('Turn Off the Visual_localization')
            self.ui.statusLabel.setStyleSheet('color: green')

        except Exception as e:
            # 오류 발생 시 바로 처리
            print(f"An error occurred: {e}")
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)
        finally:
            self.ssh_worker.close_ssh_connection()  # SSH 연결 종료
            self.ui.paramOff.setEnabled(True) 

    def param_reset_btn_clicked(self):
        # 작업 시작 시 ProgressBar 초기화
        self.ui.resetButton.setEnabled(False) 
        self.ui.progressBar.setValue(0)
        self.ui.statusLabel.setText('Resetting...')
        self.ui.statusLabel.setStyleSheet('color: black')

        # 작업 실행
        QTimer.singleShot(100, self.reset_commands)

    def reset_commands(self):
        selected_robot_key = self.ui.robotNo.currentText()
        robot_ip = robot_ip_no.get(selected_robot_key)

        if not robot_ip:
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)
            self.show_error_message(f"로봇 IP를 찾을 수 없습니다: {selected_robot_key}")
            self.ui.resetButton.setEnabled(True) 
            return

        # SSHWorker를 사용하여 SSH 연결 설정
        self.setting_ssh_worker(robot_ip)

        if not self.ssh_worker.target_client:
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.show_error_message("SSH 연결 설정에 실패했습니다.")
            return

        try:
            # 리셋 명령어 실행 (sudo 비밀번호 포함)
            print("Restarting autonomy-visual-localization-client service")
            self.ui.statusLabel_2.setText("Restarting service")
            command = "echo {} | sudo -S systemctl restart autonomy-visual-localization-client".format(target["password"])
            stdin, stdout, stderr = self.ssh_worker.target_client.exec_command(command)

            # 명령 실행 결과 처리
            result = stdout.read().decode().strip()
            error = stderr.read().decode().strip()

            if error:
                self.ui.statusLabel.setText('Failed')
                self.ui.statusLabel.setStyleSheet('color: red')
                self.show_error_message(f"Reset failed: {error}")
                raise RuntimeError(f"Failed to set {error}")
            else:
                self.ui.statusLabel.setText('Reset Completed')
                self.ui.statusLabel.setStyleSheet('color: green')
                self.ui.statusLabel_2.setText("Reseting autonomy-visual-localization-client Completed")

        except Exception as e:
            print(f"An error occurred during reset: {e}")
            self.ui.statusLabel.setText('Failed')
            self.ui.statusLabel.setStyleSheet('color: red')
            self.ui.progressBar.setValue(0)
            self.show_error_message(str(e))

        finally:
            self.ssh_worker.close_ssh_connection()  # SSH 연결 종료
            self.ui.resetButton.setEnabled(True)  

    def fetch_and_display_data(self):
        try:
            # GET 요청을 보내고 응답을 JSON으로 파싱
            response = requests.get(get_map_list, headers={'accept': 'application/json'}, timeout=10)
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

    def updateSerialNoLabel(self):
        selected_robotNo = self.ui.robotNo.currentText()
        self.robot_no = self.ui.robotNo.currentText()
        self.ui.serialNo.setText(target_dict[selected_robotNo])
        serial_no = target_dict.get(selected_robotNo, "")

        # settingTable에서 "robot_id"를 가진 필드를 찾고, 해당 셀을 업데이트합니다.
        for row in range(self.ui.settingTable.rowCount()):
            field_item = self.ui.settingTable.item(row, 0)
            if field_item and field_item.text() == "robot_id":
                widget_item = self.ui.settingTable.cellWidget(row, 1)
                if isinstance(widget_item, QLineEdit):
                    widget_item.setText(serial_no)
                break

    def run(self):
        self.ui.show()  # UI를 표시하는 함수

    def setting_ssh_worker(self, robot_ip):

        if self.ssh_robot_no == self.robot_no:
            print("동일한 ssh 사용 중입니다.")
            return
        
        self.robot_no = self.ssh_robot_no

        if not self.ssh_worker:
            # SSHWorker를 사용하여 SSH 연결 설정
            self.ssh_worker = SSHWorker(target, robot_ip, arguments['bastion']['host'], os.environ.get('MY_USERNAME'), os.environ.get('MY_PW'))
            self.ssh_worker.update_signal.connect(self.update_ui_status)
            self.ssh_worker.rlt_bool_signal.connect(self.handle_ssh_result)

            self.ssh_worker.start()
            self.ssh_worker.wait()  # SSH 연결 설정 대기
            return
        
        self.ssh_worker.close_ssh_connection()
        self.ssh_worker = SSHWorker(target, robot_ip, arguments['bastion']['host'], os.environ.get('MY_USERNAME'), os.environ.get('MY_PW'))
        self.ssh_worker.update_signal.connect(self.update_ui_status)
        self.ssh_worker.rlt_bool_signal.connect(self.handle_ssh_result)

        self.ssh_worker.start()
        self.ssh_worker.wait()  # SSH 연결 설정 대기



    def closeEvent(self, event):
        # 창이 닫힐 때 호출되는 메서드
        if self.ssh_worker is not None:
            self.ssh_worker.close_ssh_connection()
            self.ssh_worker = None
        event.accept()
        
    def show_error_message(self, message):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setWindowTitle("오류")
        msg_box.setText(message)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

    @pyqtSlot(str)
    def update_ui_status(self, message):
        # self.ui.statusLabel_2.setText(message)
        pass
    
    @pyqtSlot(str)
    def show_result(self, message):
        self.ui.statusLabel_2.setText(message)
        pass        

    @pyqtSlot(bool)
    def handle_ssh_result(self, result):
        print(f"SSH Result: {result}")
        # 받은 결과를 self.last_result에 저장
        self.last_result = result
        return

class ServerRequestThread(QThread):
    result_signal = pyqtSignal(str)

    def __init__(self, url, request_type, parent=None):
        super(ServerRequestThread, self).__init__(parent)
        self.url = url
        self.request_type = request_type

    def run(self):
        try:
            if self.request_type == "POST":
                response = requests.post(self.url, timeout=10)
            elif self.request_type == "GET":
                response = requests.get(self.url, timeout=10)

            if response.status_code == 200:
                result = f"요청 성공: {response.json()}"
            else:
                result = f"요청 실패: {response.status_code}, {response.json()}"
        except requests.exceptions.Timeout:
            result = "서버가 응답하지 않습니다."
        except requests.RequestException as e:
            result = f"요청 중 오류 발생: {e}"
        
        self.result_signal.emit(result)

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    
    thread = vlThread()
    thread.start()

    sys.exit(app.exec_())


