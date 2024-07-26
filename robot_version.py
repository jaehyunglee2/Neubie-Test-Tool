import sys
import json
import os
import re
import paramiko
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QThread, pyqtSignal

class SSHWorker(QThread):
    update_signal = pyqtSignal(str)

    def __init__(self, ip_info, selected_ip, bastion_host, user_name, user_password):
        super().__init__()
        self.ip_info = ip_info
        self.selected_ip = selected_ip
        self.bastion_host = bastion_host
        self.user_name = user_name
        self.user_password = user_password

    def run(self):
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # Bastion 호스트를 통한 SSH 연결 설정
            client.connect(hostname=self.bastion_host, username=self.user_name, password=self.user_password)

            # Target 호스트로 SSH 연결 설정
            transport = client.get_transport()
            dest_addr = (self.selected_ip, 22)
            local_addr = ('localhost', 0)
            sock = transport.open_channel("direct-tcpip", dest_addr, local_addr)

            # Target 호스트로 연결된 SSH 세션 설정
            target_client = paramiko.SSHClient()
            target_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            target_client.connect(
                self.selected_ip,
                username=self.ip_info['target']['user'],
                password=self.ip_info['target']['password'],
                sock=sock
            )

            # 명령 실행
            stdin, stdout, stderr = target_client.exec_command('cat /var/log/ansible.log')

            # 결과 출력
            version = None
            for line in stdout:
                if "neubie-binary-deployment" in line:
                    match = re.search(r'version (\S+)', line)
                    if match:
                        version = match.group(1)

            # 마지막에 쉼표가 있는 경우 제거
            if version and version.endswith(','):
                version = version[:-1]

            # 결과를 메인 GUI로 전달
            if version:
                self.update_signal.emit(f"Version: {version}")
            else:
                self.update_signal.emit("Version not found")

            # SSH 연결 종료
            target_client.close()

        except Exception as e:
            self.update_signal.emit(f"Error: {e}")

        finally:
            client.close()

class SSHVersionChecker(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SSH Version Checker")
        self.resize(600, 200)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # JSON 파일에서 정보 불러오기
        file_path = self.get_data_file_path('info', 'ip_info.json')
        with open(file_path) as f:
            self.ip_info = json.load(f)

        self.ip_combobox = QComboBox()
        self.ip_combobox.addItems(self.ip_info['ips'].keys())
        layout.addWidget(self.ip_combobox)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_to_ip)
        layout.addWidget(self.connect_button)

        self.version_label = QLabel("")
        layout.addWidget(self.version_label)

        self.setLayout(layout)

    def get_data_file_path(self, *path_parts):
        """PyInstaller의 임시 디렉토리와 개발 환경에서의 파일 경로를 지원합니다."""
        if getattr(sys, 'frozen', False):
            # PyInstaller로 빌드된 실행파일 내부에서
            base_path = sys._MEIPASS
        else:
            # 개발 환경에서
            base_path = os.path.dirname(__file__)
        
        return os.path.join(base_path, *path_parts)

    def connect_to_ip(self):
        selected_ip_number = self.ip_combobox.currentText()
        selected_ip = self.ip_info['ips'][selected_ip_number]
        bastion_host = self.ip_info['bastion']['host']
        user_name = os.environ.get('MY_USERNAME')
        user_password = os.environ.get('MY_PW')

        if not user_name or not user_password:
            self.version_label.setText("Error: Environment variables 'MY_USERNAME' or 'MY_PW' are not set.")
            return

        self.worker = SSHWorker(self.ip_info, selected_ip, bastion_host, user_name, user_password)
        self.worker.update_signal.connect(self.update_version_label)
        self.worker.start()

    def update_version_label(self, text):
        self.version_label.setText(text)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SSHVersionChecker()
    window.show()
    sys.exit(app.exec_())
