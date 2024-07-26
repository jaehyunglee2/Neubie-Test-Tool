import sys
import json
import os
import re
import paramiko
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QVBoxLayout, QWidget, QTextEdit
from PyQt5.QtCore import QThread, pyqtSignal, Qt

class LogMonitorThread(QThread):
    log_signal = pyqtSignal(str)

    def __init__(self, ip_info, selected_ip, bastion_host, user_name, user_password, keywords, special_ip_info=None, parent=None):
        super(LogMonitorThread, self).__init__(parent)
        self.ip_info = ip_info
        self.selected_ip = selected_ip
        self.bastion_host = bastion_host
        self.user_name = user_name
        self.user_password = user_password
        self.keywords = keywords
        self.special_ip_info = special_ip_info  # 추가된 특수 IP 정보
        self.running = True

    def run(self):
        try:
            if self.special_ip_info:
                # "자비에" IP에 대한 특별한 처리
                special_user = 'linkxavier'
                special_password = 'neubility'
                target_client = paramiko.SSHClient()
                target_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                target_client.connect(self.selected_ip, username=special_user, password=special_password)
                stdin, stdout, stderr = target_client.exec_command('journalctl -f')
                for line in iter(stdout.readline, ""):
                    if not self.running:
                        break
                    self.log_signal.emit(self.highlight_keywords(line))
                target_client.close()
            else:
                # 일반 처리
                client = paramiko.SSHClient()
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                client.connect(hostname=self.bastion_host, username=self.user_name, password=self.user_password)

                transport = client.get_transport()
                dest_addr = (self.selected_ip, 22)
                local_addr = ('localhost', 0)
                sock = transport.open_channel("direct-tcpip", dest_addr, local_addr)

                target_client = paramiko.SSHClient()
                target_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                target_client.connect(self.selected_ip, username=self.ip_info['target']['user'], password=self.ip_info['target']['password'], sock=sock)

                stdin, stdout, stderr = target_client.exec_command('journalctl -f')
                for line in iter(stdout.readline, ""):
                    if not self.running:
                        break
                    self.log_signal.emit(self.highlight_keywords(line))
                target_client.close()

            client.close()
        except Exception as e:
            self.log_signal.emit(f"Error: {e}")

    def highlight_keywords(self, text):
        for keyword in self.keywords:
            text = re.sub(f"({keyword})", r"\033[1;31m\1\033[0m", text)
        return text

    def stop(self):
        self.running = False


class SSHLogMonitor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SSH Log Monitor")
        self.resize(800, 600)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        file_path = self.get_data_file_path('info', 'ip_info.json')
        with open(file_path) as f:
            self.ip_info = json.load(f)

        self.ip_combobox = QComboBox()
        self.ip_combobox.addItems(self.get_ip_keys())
        layout.addWidget(self.ip_combobox)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_to_ip)
        layout.addWidget(self.connect_button)

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        layout.addWidget(self.log_output)

        self.setLayout(layout)

    def get_data_file_path(self, *path_parts):
        if getattr(sys, 'frozen', False):
            base_path = sys._MEIPASS
        else:
            base_path = os.path.dirname(__file__)
        
        return os.path.join(base_path, *path_parts)

    def get_ip_keys(self):
        return list(self.ip_info['ips'].keys())

    def connect_to_ip(self):
        selected_ip_key = self.ip_combobox.currentText()

        if '자비에' in selected_ip_key:
            selected_ip = self.ip_info['ips'][selected_ip_key]
            bastion_host = None
            user_name = None
            user_password = None
            special_ip_info = True  # 특수 IP 정보가 사용됨을 나타냅니다.
        else:
            selected_ip = self.ip_info['ips'][selected_ip_key]
            bastion_host = self.ip_info['bastion']['host']
            user_name = os.environ.get('MY_USERNAME')
            user_password = os.environ.get('MY_PW')
            special_ip_info = None

        if not (user_name or special_ip_info):
            self.log_output.append("Error: Required credentials are not set.")
            return

        keywords = ["ERROR", "FAIL", "WARNING"]  # 하이라이트할 키워드 목록
        self.worker = LogMonitorThread(self.ip_info, selected_ip, bastion_host, user_name, user_password, keywords, special_ip_info)
        self.worker.log_signal.connect(self.update_log_output)
        self.worker.start()

    def update_log_output(self, text):
        self.log_output.append(text)
        self.log_output.ensureCursorVisible()

    def closeEvent(self, event):
        if hasattr(self, 'worker'):
            self.worker.stop()
            self.worker.wait()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SSHLogMonitor()
    window.show()
    sys.exit(app.exec_())
