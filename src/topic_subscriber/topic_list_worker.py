from typing import Tuple, List
import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from autonomy_ros2_message.msg import LoggingMsg, ServiceTarget
from std_msgs.msg import Bool


from src.topic_subscriber.observer_class import TopicSubscriber, load_msg_class

class TopicListWorker(Node):
    def __init__(self, node_name='neubie_test_tool_node', **kargs):
        super().__init__(node_name)
        self.get_logger().info('노드가 시작되었습니다.')

        self.main_subscribers = dict()
        self.sub_subscribers = dict()
        for topic_name in kargs:
            self.main_subscribers[topic_name] = TopicSubscriber(self, topic_name, kargs[topic_name]['msg_type'], msg_listener=kargs[topic_name]['msg_listener'])
        # self.subscribers['/nb_ego_status_logger'] = TopicSubscriber(self, '/nb_ego_status_logger', LoggingMsg, msg_listener=None)
        # self.subscribers['/service_target'] = TopicSubscriber(self, '/service_target', ServiceTarget, msg_listener=None)
        # self.subscribers['/is_crossing'] = TopicSubscriber(self, '/is_crossing', Bool, msg_listener=None)

    def get_topics(self, no_demangle: bool = False) -> rclpy.List[Tuple[str, List[str]]]:
        return super().get_topic_names_and_types(no_demangle)
    
    def get_nodes(self) -> rclpy.List[str]:
        return super().get_node_names()
    
    def set_subscriber(self, topics: list):
        """
        새로운 구독자를 추가하고, 이를 관리 리스트에 저장합니다.
        
        :param topics: 구독할 토픽 이름과 메시지 타입의 리스트. 각 항목은 (topic_name, msg_type_str, msg_listener) 형식의 튜플로 구성되어야 합니다.
        """
        for topic_name, msg_type_str, msg_listener in topics:
            try:
                if topic_name in self.sub_subscribers: 
                    self.get_logger().info(f'{topic_name} 구독자가 이미 존재합니다.')
                    continue 
                msg_type = load_msg_class(msg_type_str[0])

                subscriber = TopicSubscriber(self, topic_name, msg_type, msg_listener)
                self.sub_subscribers[topic_name] = subscriber
                self.get_logger().info(f'{topic_name} 구독자가 추가되었습니다.')
            except Exception as e:
                print(f"{topic_name}에 해당하는 모듈이 없습니다. 무시하고 넘어갑니다.")
                print(e)

    def remove_subscriber(self, topic_name: str):
        """
        기존의 구독자를 제거합니다.

        :param topic_name: 제거할 구독자의 토픽 이름
        """
        if topic_name in self.sub_subscribers:
            self.sub_subscribers[topic_name].destroy()
            del self.sub_subscribers[topic_name]
            self.get_logger().info(f'{topic_name} 구독자가 제거되었습니다.')
        else:
            self.get_logger().warning(f'{topic_name}에 해당하는 구독자가 존재하지 않습니다.')


class RosSubscriberThread(QThread):
    def __init__(self, is_main=True, node_name='neubie_test_tool_node'):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        
        if is_main:
            signal_connect = {
                '/nb_ego_status_logger': {'msg_type': LoggingMsg, 'msg_listener': self.message_received},
                '/service_target': {'msg_type': ServiceTarget, 'msg_listener': self.service_target_received},
                '/is_crossing': {'msg_type': Bool, 'msg_listener': self.is_crossing_received},
            }
            self.worker = TopicListWorker(node_name=node_name, **signal_connect)
        else:
            self.worker = TopicListWorker(node_name='neubie_test_tool_sub_node')

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.worker)

    def run(self):
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.executor.shutdown()
            self.worker.destroy_node()

    def message_received(self, msg):
        self.message_signal.emit(msg)

    def service_target_received(self, msg):
        self.service_target_signal.emit(msg)

    def is_crossing_received(self, msg):
        self.is_crossing_signal.emit(msg)

    message_signal = pyqtSignal(LoggingMsg)
    service_target_signal = pyqtSignal(ServiceTarget)
    is_crossing_signal = pyqtSignal(Bool)