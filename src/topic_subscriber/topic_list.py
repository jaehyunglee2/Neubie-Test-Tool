from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QCheckBox, QScrollArea, QLineEdit, QLabel
from src.topic_subscriber.topic_list_worker import TopicListWorker
from src.topic_subscriber.topic_observer import TopicObserver

class GetTopicList(QWidget):
    def __init__(self, node=None):
        super().__init__()
        self.setWindowTitle('ROS 2 Topic Subscriber')
        self.resize(800, 600)  # 창의 크기를 800x600으로 설정
        self.layout = QVBoxLayout()

        self.worker = node
        self.topic_dict = dict()
        self.topic_checkboxes = []
        
        # self.init_node()
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
        if self.worker is not None:
            self.topic_dict = { topic_name : topic_type for topic_name, topic_type in self.worker.get_topics()}
            self.update_topic_checkboxes()
        else:
            print("ROS2 노드가 초기화되지 않았습니다.")

    def update_topic_checkboxes(self):
        # Clear existing checkboxes
        for checkbox in self.topic_checkboxes:
            checkbox.deleteLater()
        self.topic_checkboxes = []

        # Add checkboxes for each topic
        for topic_name in self.topic_dict:
            if topic_name:
                checkbox = QCheckBox(topic_name)
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
        subscirbe_target = []
        for checkbox in self.topic_checkboxes:

            if checkbox.isChecked():
                topic_name = checkbox.text()
                topic_type = self.topic_dict[topic_name]
                subscirbe_target.append((topic_name, topic_type))

        print(subscirbe_target)
        self.open_second_window(subscirbe_target)

    # 두 번째 창을 열기 위한 메서드 추가
    def open_second_window(self, target_topic):
        self.second_window = TopicObserver(node=self.worker, to_subscribe=target_topic)
        self.second_window.show()



if __name__ == '__main__':
    import sys, rclpy
    from PyQt5.QtWidgets import QApplication
    import threading
    from rclpy.executors import SingleThreadedExecutor

    app = QApplication(sys.argv)

    if not rclpy.ok():
        rclpy.init() 

    # ROS2 노드와 실행기 생성
    node = TopicListWorker()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # 스레드에서 실행기 스핀 시작
    spin_thread = threading.Thread(target=executor.spin, args=())
    spin_thread.start()

    window = GetTopicList(node)
    window.show()
    # 애플리케이션 종료 시, 노드 및 스레드 종료 처리
    def cleanup():
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        rclpy.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join()
        print("ROS2 노드가 종료되었습니다.")

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec_())
