import paho.mqtt.client as mqtt
import json

# 1. 콜백 함수 설정 (연결 성공 시 실행)
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")

client = mqtt.Client()
client.on_connect = on_connect

# 2. 브로커(서버) 주소로 연결
client.connect("브로커IP주소", 1883, 60)

# 3. UWB 데이터(Az, Elev 등) 발행
uwb_data = {"az": 45.2, "elev": -5.1, "lat": 37.566, "lng": 126.978}
client.publish("uwb/data", json.dumps(uwb_data))

client.loop_start()