### 기체 원격접속 후 robot_sw version 확인 ###
import json, time, os, re, paramiko
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QVBoxLayout, QWidget, QLabel
from resources.arguments import arguments

class SSHVersionChecker(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SSH Version Checker")
        layout = QVBoxLayout()

        # JSON 파일에서 정보 불러오기
        # 환경설정 필요
        # ~/.bashrc 파일에서 기체 접속에 필요한 username 및 pw 입력 필요
        # export MY_USERNAME=""
        # export MY_PW=""
        self.user_name = os.environ.get('MY_USERNAME')
        self.user_password = os.environ.get('MY_PW')
        # print(self.user_name, self.user_password)
        
        robot_inform = arguments["robot_inform"]
        self.robot_ip_info = { key : robot_inform.get(key)["ip"] for key in robot_inform }

        self.ip_combobox = QComboBox()
        self.ip_combobox.addItems(self.robot_ip_info.keys())
        layout.addWidget(self.ip_combobox)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_to_ip)
        layout.addWidget(self.connect_button)

        self.version_label = QLabel("")
        layout.addWidget(self.version_label)

        self.setLayout(layout)

    def connect_to_ip(self):
        # 선택한 번호 가져오기
        selected_robot_number = self.ip_combobox.currentText()
        selected_ip = self.robot_ip_info[selected_robot_number]

        # SSH 클라이언트 생성
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        self.version_label.clear()
        self.version_label.setText("버전 읽는 중..")
        time.sleep(3)
        
        try:
            # bastion 호스트를 통한 SSH 연결 설정
            client.connect(hostname=self.ip_info['bastion']['host'], username=self.user_name, password=self.user_password)

            # target 호스트로 SSH 연결 설정
            transport = client.get_transport()
            dest_addr = (selected_ip, 22)  # target 호스트의 SSH 포트는 일반적으로 22입니다.
            local_addr = ('localhost', 0)
            sock = transport.open_channel("direct-tcpip", dest_addr, local_addr)

            # target 호스트로 연결된 SSH 세션 설정
            target_client = paramiko.SSHClient()
            target_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            target_client.connect(selected_ip, username=self.ip_info['target']['user'], password=self.ip_info['target']['password'], sock=sock)

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

            # 버전 정보 출력
            if version:
                self.version_label.setText(f"Version: {version}")
            else:
                self.version_label.setText("Version not found")
            
            # SSH 연결 종료
            target_client.close()

        except Exception as e:
            self.version_label.setText(f"Error: {e}")

        finally:
            # SSH 클라이언트 종료
            client.close()


if __name__ == '__main__':
    app = QApplication([])
    window = SSHVersionChecker()
    window.show()
    app.exec_()
