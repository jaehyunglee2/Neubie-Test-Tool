import os, stat, subprocess, sys

class ProviderButtons():
    def __init__(self):
        super().__init__()
        print('---dds setting---')
        self.set_dds()
    
    def send_service_target(self):
        # 메시지 내용
        service_target_id = "NB_SEONGDONG_00"
        category = "Camping"
        message = f'ros2 topic pub /service_target autonomy_ros2_message/ServiceTarget "{{service_target_id: {service_target_id}, category: {category}}}" --once'

        # 터미널 명령 실행
        subprocess.run(message, shell=True)
        # print("servcie_target 발행완료")
        # self.dds_output.clear()
        # self.dds_output.setPlainText(f"servcie_target 발행완료\n 성수|캠퍼스 로만 적용")
        # print('button test')
        
    # 폴더와 파일이 있는지 확인하고 파일 생성하는 함수
    def set_dds(self):
        # self.dds_output.clear()
        try:
            # 홈 디렉토리의 경로를 가져옵니다.
            home_dir = os.path.expanduser("~")
            print(f"홈 디렉토리: {home_dir}")

            # 폴더 경로 설정
            folder_path = os.path.join(home_dir, "cmd")
            print(f"폴더 경로: {folder_path}")

            # 파일 경로 설정
            file_path = os.path.join(folder_path, "dds.sh")
            print(f"파일 경로: {file_path}")

            # 폴더 확인 후 생성
            if not os.path.exists(folder_path):
                os.makedirs(folder_path, exist_ok=True)
                print("폴더 생성 완료")
            else:
                print("폴더가 이미 존재합니다. 파일을 확인합니다.")
                
            self.check_dds_file(file_path)

        except Exception as e:
            print(f"에러 발생: {e}")        

    def check_dds_file(self, file_path):
        # 파일 확인 후 생성
        if not os.path.exists(file_path):
            with open(file_path, "w") as file:
                file.write("#!/bin/bash\n\n")
                file.write("# XML 파일에서 NetworkInterfaceAddress 값을 변경하는 함수\n")
                file.write("set_network_interface_address() {\n")
                file.write("    local xml_file=\"$1\"\n")
                file.write("    local new_address=\"$2\"\n")
                file.write("    # sed를 사용하여 XML 파일에서 NetworkInterfaceAddress 값을 변경합니다.\n")
                file.write("    sed -i \"s|<NetworkInterfaceAddress>[^<]*</NetworkInterfaceAddress>|<NetworkInterfaceAddress>$new_address</NetworkInterfaceAddress>|\" \"$xml_file\"\n")
                file.write("}\n\n")
                file.write("# 현재 연결된 네트워크 인터페이스의 이름을 가져오는 함수\n")
                file.write("get_connected_interface() {\n")
                file.write("    # `ip route get 1` 명령어로 현재 연결된 인터페이스의 이름을 가져옵니다.\n")
                file.write("    ip route get 1 | awk '{print $5}'\n")
                file.write("}\n\n")
                file.write("# 사용자의 홈 디렉토리\n")
                file.write("home_dir=\"$HOME\"\n\n")
                file.write("# 사용자의 홈 디렉토리에서 cyclonedds.xml 파일을 찾습니다.\n")
                file.write("xml_file=$(find \"$home_dir\" -name \"cyclonedds.xml\" -print -quit)\n\n")
                file.write("# cyclonedds.xml 파일을 찾지 못한 경우 에러 메시지를 출력하고 스크립트를 종료합니다.\n")
                file.write("if [ -z \"$xml_file\" ]; then\n")
                file.write("    echo \"사용자의 홈 디렉토리에서 cyclonedds.xml 파일을 찾을 수 없습니다.\"\n")
                file.write("    exit 1\n")
                file.write("fi\n\n")
                file.write("# 현재 연결된 네트워크 인터페이스의 이름을 가져옵니다.\n")
                file.write("connected_interface=$(get_connected_interface)\n\n")
                file.write("# XML 파일에서 NetworkInterfaceAddress 값을 변경합니다.\n")
                file.write("set_network_interface_address \"$xml_file\" \"$connected_interface\"\n\n")
                file.write("echo \"robot_cfg XML 파일의 NetworkInterfaceAddress 값을 $connected_interface로 변경하였습니다.\"\n")
            print("dds.sh 파일 생성 완료")
            os.chmod(file_path, stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
            print("dds.sh 권한설정 완료")


        else:
            print("파일이 존재 스크립트 실행")
        print("dds.sh 스크립트 실행")

        try:
            # 스크립트 실행
            subprocess.run(file_path, shell=True)
            print("스크립트 실행 성공\n DDS 변경완료")
        except subprocess.CalledProcessError as e:
            print("스크립트 실행 실패/수동으로 진행 ㄱ")
            # print(f"스크립트 실행 실패: {e}")
