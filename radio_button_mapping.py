# radio_button_mapping.py

class RadioButtonMapping:
    def __init__(self):
        self.behavior_rd_button = {
            "destination": 1,
            "direc_stop": 2,
            "parking": 4,
            "parking_time": 8,
            "destination_changed": 16,
            "person": 32,
            "stop_automotic": 64,
            "hagang": 128,
            "neubie": 256,
            "no_road": 512,
            # 추가적인 라디오 버튼과 번호는 추가
        }

        self.autonomy_data_failure_buttons = {
            "ekf_alive": 1,
            "rtcm_alive": 2,
            "ldm_msg": 4,
            "obstacles_tracking_msg": 8,
            "gnss_alive": 16,
            "imu_alive": 32,
            "odomgyro": 64,
            "est_states_utm": 128,
            "no_way_tag": 256,
            "no_3d_height_rc": 512,
            "no_3d_height_tr": 1024,
            # 추가적인 라디오 버튼과 번호는 추가
        }

        self.sensor_rd_buttons = {
            "cam_f": 1,
            "cam_fd": 2,
            "cam_fl": 4,
            "cam_fr": 8,
            "cam_bl": 16,
            "cam_br": 32,
            "depth_fl": 64,
            "depth_fr": 128,
            "imu_port_status": 256,
            "gnss_port_status": 512,
            "wheelodom_alive": 1024,
            "stm_port_status": 2048,
            # 추가적인 라디오 버튼과 번호는 추가
        }

        self.dangerous_rd_buttons = {
            "unaccu_1m": 1,
            "degree_120": 2,
            "path_devia": 4,
            "pitch_roll": 8,
            "no_route_90": 16,
            "dr_gnss": 32,
            "open_cargo": 64,
            "cliff_forked": 128,
            # 추가적인 라디오 버튼과 번호는 추가
        }

        self.cross_rd_buttons = {
            "not_crossing": 0,
            "approch": 1,
            "wait": 2,
            "crossing_2": 3,
            "complete": 4,
            "aborted": 5,
        }
        
        #매핑 
        self.remote_monitor_rd_buttons = {
            "건널목 앞 정지 신호": 1,
            "건널목 주행 가능 신호":2,
            "전역경로 이상 신호":4,
            "8번 사양서에 없음":8,
        }
        
        self.distance_monitor_rd_buttons = {
            "양보점 이동 중 AEB" :1,
            # "test" : 2,
        }