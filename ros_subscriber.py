import rclpy
from rclpy.node import Node
from autonomy_ros2_message.msg import LoggingMsg, ServiceTarget
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from std_msgs.msg import Bool

class RosSubscriberWorker(QObject):
    message_signal = pyqtSignal(LoggingMsg)
    service_target_signal = pyqtSignal(ServiceTarget)  # service_target
    is_crossing_signal = pyqtSignal(Bool)  # service_target

    def __init__(self):
        super().__init__()

    def run(self):
        rclpy.init()  # ROS 2 초기화
        node = rclpy.create_node('ros_subscriber_node')
        subscription_logging = node.create_subscription(
            LoggingMsg,
            '/nb_ego_status_logger', # 구독 토픽
            self.callback, 10)

        # ServiceTarget 토픽 구독
        self.subscription_service_target = node.create_subscription(
            ServiceTarget,
            '/service_target',  # 추가 구독할 토픽 이름
            self.service_target_callback,  # 추가된 토픽의 콜백 함수
            10)
        
        # is_crossing 토픽 구독
        self.subscription_is_crossing = node.create_subscription(
            Bool,
            '/is_crossing',
            self.is_crossing_callback,
            10)


        rclpy.spin(node)  # 노드가 콜백을 처리할 수 있도록 스핀
        # 종료 시 ROS 2 정리
        node.destroy_node()
        rclpy.shutdown()
        
    def is_crossing_callback(self, msg):
        # print('msg.data', msg.data)
        # self.service_target_signal.emit(msg)  # 추가된 토픽의 메시지 데이터를 PyQt5 GUI에 전달
        self.is_crossing_signal.emit(msg)

    def callback(self, msg):
        # print(msg)
        # utm_downscale_x = msg.utm_downscale_x
        # print(utm_downscale_x)
        self.message_signal.emit(msg)  # PyQt5 GUI에 메시지 데이터 전달

    def service_target_callback(self, msg):
        # print(msg)
        self.service_target_signal.emit(msg)  # 추가된 토픽의 메시지 데이터를 PyQt5 GUI에 전달

if __name__ == "__main__":
    worker = RosSubscriberWorker()
    worker.run()


# class RosSubscriberThread(QThread):
#     def __init__(self):
#         super().__init__()
#         self.worker = RosSubscriberWorker()

#     def run(self):
#         self.worker.message_signal.connect(self.message_received)
#         self.worker.run()

#     def stop(self):
#         self.worker.stop()

    # def message_received(self, msg):
        # 메시지를 받았을 때 처리하는 코드
        # utm_downscale_x = msg.utm_downscale_x
        # utm_downscale_y = msg.utm_downscale_y
        # ego_utm_scaled_x = msg.ego_utm_scaled_x
        # ego_utm_scaled_y = msg.ego_utm_scaled_y
        # wheel_speed = msg.wheel_speed
        # wheel_fl_speed = msg.wheel_fl_speed
        # wheel_fr_speed = msg.wheel_fr_speed
        # wheel_rl_speed = msg.wheel_rl_speed
        # wheel_rr_speed = msg.wheel_rr_speed
        # motor_ampere_fl = msg.motor_ampere_fl
        # motor_ampere_fr = msg.motor_ampere_fr
        # motor_ampere_rl = msg.motor_ampere_rl
        # motor_ampere_rr = msg.motor_ampere_rr
        # motor_status_fl = msg.motor_status_fl
        # motor_status_fr = msg.motor_status_fr
        # motor_status_rl = msg.motor_status_rl
        # motor_status_rr = msg.motor_status_rr
        # ego_velocity = msg.ego_velocity
        # ego_linear_velocity_x = msg.ego_linear_velocity_x
        # ego_linear_velocity_y = msg.ego_linear_velocity_y
        # ego_linear_velocity_z = msg.ego_linear_velocity_z
        # ego_linear_acceleration_x = msg.ego_linear_acceleration_x
        # ego_linear_acceleration_y = msg.ego_linear_acceleration_y
        # ego_linear_acceleration_z = msg.ego_linear_acceleration_z
        # ego_angular_velocity_x = msg.ego_angular_velocity_x
        # ego_angular_velocity_y = msg.ego_angular_velocity_y
        # ego_angular_velocity_z = msg.ego_angular_velocity_z
        # ego_angular_acceleration_x = msg.ego_angular_acceleration_x
        # ego_angular_acceleration_y = msg.ego_angular_acceleration_y
        # ego_angular_acceleration_z = msg.ego_angular_acceleration_z
        # ego_roll_deg = msg.ego_roll_deg
        # ego_pitch_deg = msg.ego_pitch_deg
        # ego_yaw_deg = msg.ego_yaw_deg
        # ego_dem_roll_deg = msg.ego_dem_roll_deg
        # ego_dem_pitch_deg = msg.ego_dem_pitch_deg
        # desired_longitudinal_velocity_mps = msg.desired_longitudinal_velocity_mps
        # desired_lateral_velocity_dps = msg.desired_lateral_velocity_dps
        # h_acc = msg.h_acc
        # selected_reference = msg.selected_reference
        # way_road_type = msg.way_road_type
        # way_rule = msg.way_rule
        # reference_path_offset = msg.reference_path_offset
        # global_planner_thread_running_time = msg.system_monitor.global_planner_thread_running_time
        # local_planner_thread_running_time = msg.system_monitor.local_planner_thread_running_time
        # decision_making_thread_running_time = msg.system_monitor.decision_making_thread_running_time 
        # control_thread_running_time = msg.system_monitor.control_thread_running_time
        # perception_thread_running_time = msg.system_monitor.perception_thread_running_time
        # robot_chassis_thread_loop_time = msg.system_monitor.robot_chassis_thread_loop_time
        # battery_status = msg.system_monitor.battery_status
        # autonomy_condition = msg.system_monitor.autonomy_condition
        # behavior_flags = msg.system_monitor.behavior_flags
        # aeb_status_flag = msg.system_monitor.aeb_status_flag
        # mrm_status_flag = msg.system_monitor.mrm_status_flag
        # sensor_failure_flags = msg.system_monitor.anomaly_condition_flags.sensor_failure_flags
        # autonomy_data_failure_flags = msg.system_monitor.anomaly_condition_flags.autonomy_data_failure_flags
        # dangerous_status_flags = msg.system_monitor.anomaly_condition_flags.dangerous_status_flags
        # system_failure_flags = msg.system_monitor.anomaly_condition_flags.system_failure_flags
        # remote_controller_attention_flags = msg.system_monitor.remote_controller_attention_flags //관제주시신호데이터정의 https://neubility.atlassian.net/wiki/spaces/AUT/pages/393379853/-+Planning+Control
        # voice_sound_play_flags = msg.system_monitor.voice_sound_play_flags
        # delivery_ready_flag = msg.system_monitor.delivery_ready_flag
        # destination_departure_flag = msg.system_monitor.destination_departure_flag
        # system_info = msg.system_monitor.system_info
        # debug_data = msg.debug_data
        # crossing = debug_data.data[0].value
        # not_crossing = debug_data.data[1].value
        # _yield = debug_data.data[2].value
        # cross_state = debug_data.data[3].value


        # print('utm_downscale_x', utm_downscale_x)


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     ros_thread = RosSubscriberThread()
#     ros_thread.start()
    
#     # Qt 애플리케이션 실행
#     sys.exit(app.exec_())
