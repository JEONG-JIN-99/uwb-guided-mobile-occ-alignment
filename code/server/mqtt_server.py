import paho.mqtt.client as mqtt
import json

# 1. 브로커 연결 성공 시 실행되는 콜백 함수
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("connected")
        # 연결 성공 시 구독할 토픽을 지정합니다.
        client.subscribe("uwb/data")
    else:
        print(f"faile to connect{rc})")

# 2. 메시지가 도착했을 때 실행되는 콜백 함수
def on_message(client, userdata, msg):
    try:
        # JSON 문자열을 파이썬 딕셔너리로 변환
        data = json.loads(msg.payload.decode())
        
        print(f"\n--- [UWB 데이터 수신: {msg.topic}] ---")
        print(f" 방위각(Az): {data.get('az')}°")
        print(f" 고도각(Elev): {data.get('elev')}°")
        print(f" 위치(Lat/Lng): {data.get('lat')}, {data.get('lng')}")
        
    except Exception as e:
        print(f"데이터 파싱 에러: {e}")

# 3. 클라이언트 설정
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) # 최신 v2.0 기준
client.on_connect = on_connect
client.on_message = on_message

# 4. 브로커 접속 (IP주소는 실제 브로커 주소로 변경하세요)
broker_address = "192.168.0.32" # 본인 PC에서 테스트 시
client.connect(broker_address, 1883, 60)

# 5. 네트워크 루프 시작 (메시지를 계속 기다림)
print("📡 UWB 데이터 대기 중...")
client.loop_forever()