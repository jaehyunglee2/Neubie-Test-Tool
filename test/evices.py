import sys
import yaml, os
from PyQt5.QtWidgets import QApplication, QComboBox, QLabel, QVBoxLayout, QWidget

class DeviceInfoViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Device Information Viewer")
        layout = QVBoxLayout()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_file_path = os.path.join(current_dir, 'devices.yaml')

        # YAML 파일 읽기
        with open(yaml_file_path) as f:
            self.devices_info = yaml.safe_load(f)

        # 콤보박스 생성
        self.device_combobox = QComboBox()
        self.device_combobox.addItems(self.devices_info.keys())
        self.device_combobox.currentIndexChanged.connect(self.show_device_info)
        layout.addWidget(self.device_combobox)

        # 정보 표시 레이블
        self.info_label = QLabel()
        layout.addWidget(self.info_label)

        self.setLayout(layout)

    def show_device_info(self):
        # 선택한 기기명 가져오기
        selected_device = self.device_combobox.currentText()

        # 선택한 기기의 정보 가져오기
        device_info = self.devices_info[selected_device]

        # 정보 텍스트 생성
        info_text = f"WIFI SSID: {device_info['WIFI_SSID']}\n"
        info_text += f"WIFI PW: {device_info['WIFI_PW']}\n"
        info_text += f"USERNAME: {device_info['USERNAME']}\n"
        info_text += f"PASSWORD: {device_info['PASSWORD']}\n"

        # 정보 텍스트를 레이블에 설정
        self.info_label.setText(info_text)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = DeviceInfoViewer()
    viewer.show()
    sys.exit(app.exec_())
