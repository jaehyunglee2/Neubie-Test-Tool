import paramiko, re
from PyQt5.QtCore import QThread, pyqtSignal

class SSHWorker(QThread):
    update_signal = pyqtSignal(str)
    result_signal = pyqtSignal(str)
    rlt_bool_signal = pyqtSignal(bool)
    version_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self, target, selected_ip, bastion_host, user_name, user_password):
        super().__init__()
        self.target = target
        self.selected_ip = selected_ip
        self.bastion_host = bastion_host
        self.user_name = user_name
        self.user_password = user_password
        self.client = None
        self.target_client = None

    def run(self):
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.update_signal.emit("Connecting to bastion host...")
            self.client.connect(hostname=self.bastion_host, username=self.user_name, password=self.user_password)
            self.update_signal.emit("Connected to bastion host.")

            self.update_signal.emit(f"Establishing SSH connection to target host {self.selected_ip}...")
            transport = self.client.get_transport()
            dest_addr = (self.selected_ip, 22)
            local_addr = ('localhost', 0)
            sock = transport.open_channel("direct-tcpip", dest_addr, local_addr)
            
            self.target_client = paramiko.SSHClient()
            self.target_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.target_client.connect(self.selected_ip, username=self.target['user'], password=self.target['password'], sock=sock)
            self.update_signal.emit("SSH connection established to target host.")
            
            # 프롬프트 표시
            shell = self.target_client.invoke_shell()
            self.update_signal.emit("Shell invoked. Waiting for prompt...")

            # 프롬프트를 읽어와서 표시
            prompt = shell.recv(1024).decode('utf-8')
            self.result_signal.emit(prompt)
            
            transport.set_keepalive(60)

        except Exception as e:
            self.update_signal.emit(f"Error: {e}")
            if self.target_client:
                self.target_client.close()
            if self.client:
                self.client.close()
            self.target_client = None
        finally :
            self.finished_signal.emit()

    def set_param(self, target_node, param_name, param_value):
        try:
            self.update_signal.emit(f"Set {param_name} : {param_value}")
            command = f". /opt/ros/galactic/setup.bash && ros2 param set {target_node} {param_name} \"{param_value}\""
            self.update_signal.emit(f"Executing: {command}")
            
            stdin, stdout, stderr = self.target_client.exec_command(command)
            error = stderr.read().decode()
            output = stdout.read().decode()
            result = output + error

            self.result_signal.emit(result)

            # result = stdout.read().decode().strip()
            # error = stderr.read().decode().strip()
            
            # print("r",result)
            # print("e",error)
            filtered_errors = [line for line in error.splitlines() if 'DeprecationWarning' not in line]
            if filtered_errors:     
                for line in filtered_errors:
                    if 'Wait for service timed out' in line:
                        self.result_signal.emit(f'Error: Service timed out while setting {param_name}')
                        self.rlt_bool_signal.emit(False)
                        return
                    elif 'Node not found' in line:
                        self.result_signal.emit(f'Error: Node not found while setting {param_name}')
                        self.rlt_bool_signal.emit(False)
                        return


            if 'Set parameter successful' in result:
                self.result_signal.emit(f'Success: {param_name} set successfully')
                self.rlt_bool_signal.emit(True)
            else:
                self.result_signal.emit(f'Unknown result: {result}')
                self.rlt_bool_signal.emit(False)

        except Exception as e:
            self.update_signal.emit(f"Error: {e}")

    def get_param(self, target_node, param_name):
        self.update_signal.emit(f"Get {param_name}'s value")
        command = f". /opt/ros/galactic/setup.bash && ros2 param get {target_node} {param_name}"

        stdin, stdout, stderr = self.target_client.exec_command(command)
        result = stdout.read().decode() + stderr.read().decode()
        self.result_signal.emit(result)

    def get_version(self):
        try:
            self.version_signal.emit("Check Version_started...")
            # 명령 실행
            stdin, stdout, stderr = self.target_client.exec_command('cat /var/log/ansible.log')

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
                self.version_signal.emit(f"Version: {version}")
            else:
                self.version_signal.emit("Version not found")
        except Exception as e:
            self.version_signal.emit(f"Error: {e}")

    def close_ssh_connection(self):
        if self.target_client:
            self.target_client.close()
        if self.client:
            self.client.close()
        self.update_signal.emit("SSH connection closed.")