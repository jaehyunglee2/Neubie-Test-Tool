
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.node import Node

class TopicPublisher:
    
    qos_depth = 10
    QOS_RKL10TL = QoSProfile(
        reliability = QoSReliabilityPolicy.RELIABLE,
        history = QoSHistoryPolicy.KEEP_LAST,
        depth = qos_depth,
        durability = QoSDurabilityPolicy.VOLATILE
    )
    
    def __init__(self, node : Node, topic_name, msg_type):
        
        if not isinstance(node, Node) :
            print(f"제공해준 node의 type{type(node)}이 적절하지(Node) 않습니다")
            self.__del__()
            return 
        
        self.node = node
        self.topic_name = topic_name
        self.msg = msg_type()
        self.msg_type = self.msg.__class__
        
        self.publisher = self.node.create_publisher(self.msg_type, self.topic_name, TopicPublisher.QOS_RKL10TL)

    def create_msg(self, msg):
        # self.msg = msg
        pass

    def publish_msg(self, msg):
        self.publisher.publish(msg)

    def destroy(self):
        self.node.destroy_publisher(self.publisher)


class TopicSubscriber:

    qos_depth = 10
    QOS_RKL10TL = QoSProfile(
        reliability = QoSReliabilityPolicy.RELIABLE,
        history = QoSHistoryPolicy.KEEP_LAST,
        depth = qos_depth,
        durability = QoSDurabilityPolicy.VOLATILE
    )

    def __init__(self, node : Node, topic_name, msg_type, msg_listener = None):
        '''
        msg_listener는 외부에서 참조하는 함수로 msg_callback과 동일하게
        구독하는 메시지를 받아올 수 있도록 동작하는 '함수'형으로 받아와져야한다.

        '''
        if not isinstance(node, Node) :
            print(f"제공해준 node의 type{type(node)}이 적절하지(Node) 않습니다")
            self.__del__()
            return 

        self.node = node
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.msg = None
        self.listeners = []  # 리스너 목록을 저장하기 위한 리스트

        if msg_listener:
            self.listeners.append(msg_listener)

        self.subscription = self.node.create_subscription(
            self.msg_type,
            self.topic_name,
            self.msg_callback,
            TopicSubscriber.QOS_RKL10TL)

    def msg_callback(self, msg):
        self.msg = msg
        # self.node.get_logger().info(f'{self.topic_name} - {msg}')
        for listener in self.listeners:
            try:
                listener(msg)
            except:
                self.node.get_logger().warn(f'error while {self.topic_name} msg send ')
                
    def add_listener(self, listener):
        self.listeners.append(listener)

    def remove_listener(self, listener):
        if listener:
            self.listeners.remove(listener)

    def get_latest_msg(self):
        return self.msg
    
    def destroy(self):
        self.node.destroy_subscription(self.subscription)


    
def load_msg_class(full_class_string):
    '''
    동적으로 메시지 클래스를 로드하는 함수.
    '''
    try:
        # '/'를 기준으로 모듈과 클래스 이름 분리
        full_class_list = full_class_string.split('/')

        module_list = full_class_list[:-1]
        module_str = ".".join(module_list)

        class_str = full_class_list[-1]

        module = __import__(module_str, fromlist=[class_str])
        msg_class = getattr(module, class_str)

        return msg_class
    except ModuleNotFoundError:
        raise ModuleNotFoundError("No module Imported")
    
"""
이런식으로 동적으로 할당 가능
    def append_items(self, **kargs):
        '''
        Topic정보와, Topic의 Class를 정리하여 Rosbag을 전송할 수 있도록 Topic subscriber를 구분한다.
        '''
        topic = kargs.get('topic')
        msg_class = kargs.get('msg_class')

        if not (topic and msg_class) : 
            raise ValueError(f'topic - {topic} & msg_class - {msg_class}. \n if one of these is None, please check this values')


        if (msg_class.__name__ == Image.__name__) or (msg_class.__name__ == CompressedImage.__name__):
            self.get_logger().info(f'get Image - {topic}')
            subscriber = TopicSubscriber(node=self, topic_name=f"{topic}/copy", msg_type=msg_class)
            self.image_subscribers.append(subscriber)
        else:
            if not ('header' in msg_class.get_fields_and_field_types()) : return
            self.get_logger().info(f'get Report - {topic} - msg_class {msg_class}')
            subscriber = TopicSubscriber(node=self, topic_name=f"{topic}/copy", msg_type=msg_class)
            self.report_subscribers.append(subscriber)
"""