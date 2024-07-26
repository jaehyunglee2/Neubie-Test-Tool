import rclpy
import time
from rclpy.node import Node
from autonomy_ros2_message.msg import LoggingMsg, ServiceTarget
from PyQt5.QtCore import QObject, pyqtSignal
from std_msgs.msg import Bool


class RosSubscriberWorker(QObject):
    nb_ego_status_logger_signal = pyqtSignal(dict, dict, dict, dict)
    service_target_signal = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()

        self.behavior_flag_items = ['도착지5m근접', '신호대기정지', '갓길이동정지1', '갓길이동정지2', '출발전헤딩정렬', 
                                    '보행자감속', '자율주행정지신호', '하강경사로', '뉴비감속', '전방주행가능영역없음']

        self.sensor_info_items = ['/cam_f/symbolic_link_alive', '/cam_fd/symbolic_link_alive', 
                                  '/cam_fl/symbolic_link_alive', '/cam_fr/symbolic_link_alive', 
                                  '/cam_bl/symbolic_link_alive', '/cam_br/symbolic_link_alive', 
                                  '/depth_fl/symbolic_link_alive', '/depth_fr/symbolic_link_alive', 
                                  '/imu_port_status', '/gnss_port_status', 
                                  '/wheelodom_alive', '/stm_port_status']

        self.aut_info_items = ['/ekf_alive', '/rtcm_alive', '/ldm_msg', '/obstacles_tracking_msg', '/gnss_alive', '/imu_alive', '/odomgyro', '/est_states_utm', '/way_tag_정보없음',
                               '/ldm_msg에 3d height(수신)', '/ldm_msg에 3d height(송신)']

        self.mrm_flag_items = ['20초1m이하', '120도차이', '경로이탈', 'pitch33도', '주행불가영역', 'dr_gnss오차',
                               '적재함열림주행', '낭떠러지구간']

        self.crossing_info_items = ['횡단관련없음', '접근중', '횡단대기', '횡단중', '횡단완료', '정의된값없음']

    def run(self):
        rclpy.init()
        node = rclpy.create_node('ros_subscriber_node')

        subscription_logging = node.create_subscription(
            LoggingMsg,
            '/nb_ego_status_logger',
            self.nb_ego_status_logger_callback, 10)

        subscription_service_target = node.create_subscription(
            ServiceTarget,
            '/service_target',
            self.service_target_callback, 10)

        subscription_is_crossing = node.create_subscription(
            Bool,
            '/is_crossing',
            self.is_crossing_callback, 10)

        while rclpy.ok():
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                print("Shutting down gracefully.")
                break
            except Exception as e:
                print(f"Error occurred: {e}. Retrying in 2 seconds...")
                time.sleep(2)

        node.destroy_node()
        rclpy.shutdown()

    def is_crossing_callback(self, msg):
        pass

    def nb_ego_status_logger_callback(self, msg):
        try:
            parse_sys_info = self.parse_sys_info(msg)
            parse_sensor_info = self.parse_sensor_info(msg) #기능안전데이터 통합 (sensor / autonomy data / 경로이탈 등등...)
            parse_cross_info = self.parse_cross_info(msg)
            parse_debug_info = self.parse_debug_info(msg)
            self.nb_ego_status_logger_signal.emit(parse_sys_info, parse_sensor_info, parse_cross_info, parse_debug_info)
            # print('No error')
            
        except IndexError as e:
            print(f"IndexError occurred: {e}. Data might be incomplete. Waiting for next message.")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    def parse_debug_info(self, msg): # tablewidget_4
        debug_info = dict()

        if hasattr(msg.debug_data, 'data') and isinstance(msg.debug_data.data, list):
            for element in msg.debug_data.data:
                key = element.key.data  # std_msgs.msg.String 객체의 data 속성 접근
                value = element.value
                debug_info[key] = value  # debug_info 딕셔너리에 키와 값을 추가

        return debug_info


    def parse_cross_info(self, msg):  # tablewidget_3
        cross_info = dict()
        if hasattr(msg.debug_data, 'data') and isinstance(msg.debug_data.data, list):
            for element in msg.debug_data.data:
                key = element.key.data  # std_msgs.msg.String 객체의 data 속성 접근
                value = element.value
                if key == 'cross_state':
                    cross_info['횡단관련없음'] = value
                    cross_info['접근중'] = value
                    cross_info['횡단대기'] = value
                    cross_info['횡단중'] = value
                    cross_info['횡단완료'] = value
                    cross_info['정의된값없음'] = value

        return cross_info
    
    

    def parse_sensor_info(self, msg):  # tablewidget_2
        sensor_flag = msg.system_monitor.anomaly_condition_flags.sensor_failure_flags
        aut_flag = msg.system_monitor.anomaly_condition_flags.autonomy_data_failure_flags
        sensor_info = dict()
        for item in self.sensor_info_items:
            sensor_info[item] = sensor_flag
        for item in self.aut_info_items:
            sensor_info[item] = aut_flag
            
        #dangerous_flag/sensor_flag 모두 기능안전데이터 안에 있음 두개 합침
        dangerous_flag = msg.system_monitor.anomaly_condition_flags.dangerous_status_flags    
        for item in self.mrm_flag_items:
            sensor_info[item] = dangerous_flag
            
        return sensor_info

    def parse_sys_info(self, msg):  # tablewidget
        autonomy_condition = msg.system_monitor.autonomy_condition  # 0오퍼레이터/1관제/2자율주행
        sys_info = msg.system_monitor.system_info
        battery_status = msg.system_monitor.battery_status
        h_acc = msg.h_acc
        ego_velocity = round(msg.ego_velocity, 5)
        formatted_ego_vel = "{:.3f}".format(ego_velocity)
        selected_reference = msg.selected_reference
        way_road_type = msg.way_road_type
        way_rule = msg.way_rule
        mrm_status_flag = msg.system_monitor.mrm_status_flag
        aeb_status = msg.system_monitor.aeb_status_flag
        desired_long = round(msg.desired_longitudinal_velocity_mps, 3)
        formatted_desired_long = "{:.3f}".format(desired_long)
        desired_lat = round(msg.desired_lateral_velocity_dps, 3)
        formatted_desired_lat = "{:.3f}".format(desired_lat)
        behavior_flag = msg.system_monitor.behavior_flags

        values = sys_info.split("|")
        values = [value.strip() for value in values if value.strip()]
        system_info = dict()

        system_info['모드'] = str(autonomy_condition)
        system_info['날짜'] = values[0]
        system_info['시간'] = values[1]
        for item in values[2:]:
            key, value = item.split(':', 1)
            system_info[key.strip()] = value.strip()
        system_info['배터리 상태'] = str(battery_status)
        system_info['GPS'] = str(h_acc)
        system_info['입력속도'] = str(formatted_desired_long)
        system_info['출력속도'] = str(formatted_ego_vel)
        system_info['회전'] = str(formatted_desired_lat)
        system_info['도출기준경로'] = str(selected_reference)
        system_info['RoadType[태그정보]'] = str(way_road_type)
        system_info['WayRule[태그정보]'] = str(way_rule)
        system_info['mrm'] = str(mrm_status_flag)
        system_info['aeb'] = str(aeb_status)

        for item in self.behavior_flag_items:
            system_info[item] = behavior_flag


        return system_info

    def service_target_callback(self, msg):
        service_target = msg.service_target_id
        service_category = msg.category
        self.service_target_signal.emit(service_target, service_category)


if __name__ == '__main__':
    subscriber = RosSubscriberWorker()
    subscriber.run()
