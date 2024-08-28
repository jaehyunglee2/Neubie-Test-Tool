from resources.topic_status import topic_status

def update_radio_button_status(radio_button, color):
    try:
        radio_button.setStyleSheet(f"""
            QRadioButton::indicator {{
                border: 2px solid black;
                border-radius: 10px;
                background-color: {color};
            }}
            QRadioButton::indicator:checked {{
                background-color: {color};
            }}
        """)
    except:
        pass

def update_status(msg, topic_name, radio_button, status_label):
    try:
        # 어떤 정보를 확인해야하는지 모르기 때문에 직접 관리해줘야합니다.
        # 들어오는 데이터 정보가 # msg.data인지, odom.x, odom.y, odom.z인지 알 수 없기 때문
        # 위의 예시 말고도, 다양한 형태가 있기 때문에 최소 기능으로 이러한 작업이 필요합니다.
        # 메시지 형태는 print(topic_name, '-', msg)를 통해 확인할 수 있습니다.
 
        # topic_status.py의 topic_status 에서 토픽들 정보를 관리해야합니다. 
        target_topic_info = topic_status.get(topic_name, {})
        # topic_status에 정보가 없는 것은 무시하고 넘어갑니다.
        if len(target_topic_info.keys()) == 0: 
            update_radio_button_status(radio_button, 'black')
            status_label.setText(f'Status: Error - 토픽과 관련 된 status정보가 없습니다. \n topic_status.py 에 status를 추가 바랍니다.')
            return

        if topic_name == '/cargo_open_status':
            result = msg.data
            update_radio_button_status(radio_button, target_topic_info[result]['color'])
            status_label.setText(f'Status: {target_topic_info[result]["status_text"]}')

        elif topic_name == '/battery_status':
            result = msg.data
            update_radio_button_status(radio_button, target_topic_info[result]['color'])
            status_label.setText(f'Status: {target_topic_info[result]["status_text"]}')

        elif topic_name == '/autonomous_stop_signal_from_server':
            result = msg.data
            update_radio_button_status(radio_button, target_topic_info[result]['color'])
            status_label.setText(f'Status: {target_topic_info[result]["status_text"]}')

        ## 아래에 확인하고자 하는 topic을 추가하시면 됩니다.


    except Exception as e :
        if radio_button :
            update_radio_button_status(radio_button, "purple") 
            status_label.setText(f'Status: Error - 올바르지 않은 매칭정보 \n topic_status.py 에 등록한 status와 매칭이 되는지 참고 바랍니다')
        else:
            print(e)