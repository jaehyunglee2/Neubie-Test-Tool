import sys
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QCheckBox, QTextEdit, QScrollArea, QLineEdit, QLabel
import rclpy

class GetTopicList(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS 2 Topic Subscriber')
        self.resize(800, 600)  # 창의 크기를 800x600으로 설정
        self.layout = QVBoxLayout()
        self.topic_checkboxes = []
        self.init_ui()
        self.setLayout(self.layout)

    def init_ui(self):
        self.refresh_button = QPushButton('Refresh Topics')
        self.refresh_button.clicked.connect(self.refresh_topics)
        self.layout.addWidget(self.refresh_button)

        self.search_label = QLabel('Search Topics:')
        self.layout.addWidget(self.search_label)

        self.search_input = QLineEdit()
        self.search_input.textChanged.connect(self.filter_topics)
        self.layout.addWidget(self.search_input)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_content)
        self.scroll_content.setLayout(self.scroll_layout)
        self.scroll_area.setWidget(self.scroll_content)
        self.layout.addWidget(self.scroll_area)

        self.subscribe_button = QPushButton('Subscribe to Selected Topics')
        self.subscribe_button.clicked.connect(self.subscribe_topics)
        self.layout.addWidget(self.subscribe_button)

    def refresh_topics(self):
        self.topic_list = self.get_topic_list()
        self.update_topic_checkboxes()

    def get_topic_list(self):
        command = 'ros2 topic list'
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return result.stdout.split('\n')

    def update_topic_checkboxes(self):
        # Clear existing checkboxes
        for checkbox in self.topic_checkboxes:
            checkbox.deleteLater()
        self.topic_checkboxes = []

        # Add checkboxes for each topic
        for topic in self.topic_list:
            if topic:
                checkbox = QCheckBox(topic)
                checkbox.setChecked(False)
                self.topic_checkboxes.append(checkbox)
                self.scroll_layout.addWidget(checkbox)

        self.filter_topics()

    def filter_topics(self):
        search_text = self.search_input.text().lower()
        for checkbox in self.topic_checkboxes:
            topic_name = checkbox.text().lower()
            checkbox.setVisible(search_text in topic_name)

    def subscribe_topics(self):
        for checkbox in self.topic_checkboxes:
            if checkbox.isChecked():
                topic_name = checkbox.text()
                subprocess.Popen(['gnome-terminal', '--', 'ros2', 'topic', 'echo', topic_name])

    # 두 번째 창을 열기 위한 메서드 추가
    def open_second_window(self):
        self.second_window = SecondWindow()
        self.second_window.show()

class SecondWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Second Window')
        self.resize(1000, 800)  # 두 번째 창의 크기를 1000x800으로 설정

        layout = QVBoxLayout()
        label = QLabel('This is the Second Window')
        layout.addWidget(label)
        self.setLayout(layout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = GetTopicList()
    window.show()
    sys.exit(app.exec_())
