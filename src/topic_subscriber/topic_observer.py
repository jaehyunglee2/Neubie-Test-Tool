from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QRadioButton, QLabel, QSizePolicy
from PyQt5.QtCore import Qt
from src.topic_subscriber.topic_list_worker import TopicListWorker
from resources.topic_status_manager import *

class TopicObserver(QWidget):
    def __init__(self, node : TopicListWorker, to_subscribe):
        super().__init__()
        self.setWindowTitle('Second Window')
        self.resize(1000, 800)  # 두 번째 창의 크기를 1000x800으로 설정

        selected_topic_layout = QVBoxLayout()
        selected_topic_layout.setAlignment(Qt.AlignTop)
        self.setLayout(selected_topic_layout)

        self.node = node
        self.to_subscribe = to_subscribe

        # 각 토픽에 대해 개별 레이아웃 추가
        self.topic_widgets = {}
        for topic_name, topic_type in self.to_subscribe:
            topic_widget, r, l = self.create_topic_widget(topic_name, topic_type)
            selected_topic_layout.addWidget(topic_widget)

            msg_listener = self.create_msg_listener(topic_name)
            self.topic_widgets[topic_name] = {
                'button' : r,
                'status' : l,
                'msg_listener' : msg_listener
            }

            # 이미 존재하는 구독자라면, 새로 추가하지 않고 리스너만 추가
            if self.node.sub_subscribers.get(topic_name):
                self.node.get_logger().info(f'{topic_name} 구독자가 이미 존재합니다. listener만 추가합니다.')
                self.node.sub_subscribers[topic_name].listeners.append(msg_listener)
            else:
                # 새로운 구독자 생성
                self.node.set_subscriber([(topic_name, topic_type, msg_listener)])


    def create_topic_widget(self, topic_name, topic_type):
        """
        각 토픽에 대해 별도의 위젯을 생성하고 반환합니다.
        """
        topic_layout = QVBoxLayout()
        status_layout = QHBoxLayout()
        status_layout.setAlignment(Qt.AlignLeft)

        # 상태 레이아웃 설정
        radio_button = QRadioButton()
        radio_button.setEnabled(False)
        status_label = QLabel()  # 예시로 'Active' 상태 표시
        status_label.setText('Status: Idle')

        status_layout.addWidget(radio_button)
        status_layout.addWidget(status_label)

        # # 토픽 이름과 상태 레이블 추가
        topic_name_label = QLabel(f'Topic: {topic_name}')

        topic_layout.addWidget(topic_name_label)
        topic_layout.addLayout(status_layout)

        # 전체를 포함하는 위젯 반환
        topic_widget = QWidget()
        topic_widget.setLayout(topic_layout)
        topic_widget.setFixedHeight(100)  # 원하는 높이로 설정 (예: 100px)

        return topic_widget, radio_button, status_label
    
    def create_msg_listener(self, topic_name):
        """
        특정 토픽에 대한 메시지 콜백을 동적으로 생성합니다.
        """
        def msg_listener(msg):
            
            if self.topic_widgets.get(topic_name) == None : return

            radio_button = self.topic_widgets[topic_name]['button']
            status_label = self.topic_widgets[topic_name]['status']

            update_status(msg, topic_name, radio_button, status_label)

        return msg_listener
    
    def closeEvent(self, event):
        """
        창이 닫힐 때 모든 구독자를 제거합니다.
        """
        for topic_name, topic_type in self.to_subscribe:
            topic_subscriber = self.node.sub_subscribers.get(topic_name)
            msg_listener = self.topic_widgets[topic_name]['msg_listener']

            if not topic_subscriber : continue
            topic_subscriber.remove_listener(msg_listener)
            if not len(topic_subscriber.listeners):
                self.node.remove_subscriber(topic_name)
        event.accept()  # 창 닫기를 허용

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

    to_subscribe = [
        ('/avoid_path_points_robot', ['sensor_msgs/msg/PointCloud2']), 
        ('/battery_status', ['std_msgs/msg/Int32']), 
        ('/cargo_open_status', ['std_msgs/msg/Int8']),
        ('/autonomous_stop_signal_from_server', ['std_msgs/msg/Bool']),
    ]  # 예시 데이터
    
    window = TopicObserver(node, to_subscribe)
    
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
