from PyQt5.QtCore import pyqtSignal, QObject
import os, subprocess

# Worker 클래스 정의
class Domain_worker(QObject):
    finished = pyqtSignal()
    update_text = pyqtSignal(str)  # 텍스트 업데이트를 위한 신호


    def __init__(self, domain_id):
        super().__init__()
        self.domain_id = str(domain_id)


    # # ROS 도메인을 설정하는 함수
    def run(self):
        # ROS_DOMAIN_ID 환경 변수의 값을 읽어옴
        print('ros_domain_id', self.domain_id) 

        try:
            bashrc_path = os.path.expanduser("~/.bashrc")
            new_setting = f'export ROS_DOMAIN_ID="{self.domain_id}" # Added by Python script\n'
            found = False

            # .bashrc 파일 읽기
            with open(bashrc_path, "r") as file:
                lines = file.readlines()

            # .bashrc 파일 쓰기
            with open(bashrc_path, "w") as file:
                for line in lines:
                    # ROS_DOMAIN_ID 설정 있는지 확인
                    if "export ROS_DOMAIN_ID=" in line:
                        if not found:
                            # 첫 설정 새 값으로 교체
                            file.write(new_setting)
                            found = True
                        # 중복 설정 무시
                    else:
                        # 다른 줄 그대로 유지
                        file.write(line)
                
                # ROS_DOMAIN_ID 설정 없으면 파일 끝에 추가
                if not found:
                    file.write(new_setting)
            
            # 변경사항 적용 안내 메시지 메인 스레드로 전송
            message = "ROS_DOMAIN_ID 변경완료 프로그램 재실행 필요"
            self.update_text.emit(message)
            
            # 작업 완료 신호 발생
            self.finished.emit()
        except subprocess.CalledProcessError as e:
            # 에러 발생 시 에러 메시지를 전송합니다.
            self.update_text.emit(str(e))
            self.finished.emit()  # 작업 완료 신호를 발생


