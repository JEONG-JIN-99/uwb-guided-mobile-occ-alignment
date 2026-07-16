import time
import socket

GIMBAL_IP = "127.0.0.1" 
PORT = 5005


if __name__ == "__main__":
    # Mock UWB data
    meas_distance = 5.0  # meters
    meas_azimuth = 45.0  # degrees
    meas_elevation = 10.0  # degrees

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        m_uwb_read_time_ns = time.time_ns()
        m_mode, m_dist, m_az, m_elev, m_lat, m_lng = 1,meas_distance,meas_azimuth,meas_elevation,0,0
        message = (f"{m_mode},{m_dist},{m_az},{m_elev},{m_lat},{m_lng},{m_uwb_read_time_ns}")
        sock.sendto(message.encode(), (GIMBAL_IP, PORT))
        print(f"transmission message: {message}")
        time.sleep(0.1) # 10Hz 출력