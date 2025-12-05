#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

# --- ROS 노드 및 퍼블리셔 초기화 ---
try:
    rospy.init_node('gps_socket_server', anonymous=True)
    
    # ROS 퍼블리셔 설정
    # /destination_gps 토픽으로 sensor_msgs/NavSatFix 메시지를 발행
    # NavSatFix는 위도(latitude), 경도(longitude), 고도(altitude) 등을 포함하는 표준 GPS 메시지
    pub = rospy.Publisher('/destination_gps', NavSatFix, queue_size=10)

except rospy.ROSInterruptException as e:
    print(f"ROS 초기화 실패: {e}")
    exit(1)
except Exception as e:
    print(f"알 수 없는 오류로 ROS 초기화 실패: {e}")
    exit(1)


HOST = '0.0.0.0'  # 모든 IP에서 오는 연결을 허용
PORT = 12345

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((HOST, PORT))
        except socket.error as e:
            rospy.logerr(f"소켓 바인딩 실패: {e}")
            return

        s.listen()
        
        # ★ 중요: 1초 타임아웃 설정
        # s.accept()가 무한정 대기하는 것을 방지합니다.
        # 이를 통해 while 루프가 주기적으로 rospy.is_shutdown()을 확인할 수 있어
        # Ctrl+C (SIGINT)로 노드를 깔끔하게 종료할 수 있습니다.
        s.settimeout(1.0)
        
        rospy.loginfo(f" {HOST}:{PORT}에서 GPS 데이터 수신 대기 중...")

        while not rospy.is_shutdown():
            try:
                # 1. 클라이언트 연결 대기
                conn, addr = s.accept()
                
                with conn:
                    rospy.loginfo(f"{addr}에서 연결됨")
                    conn.settimeout(1.0) # 클라이언트 소켓에도 타임아웃 설정
                    
                    while not rospy.is_shutdown():
                        try:
                            # 2. 데이터 수신
                            data = conn.recv(1024)
                            if not data:
                                break # 클라이언트가 연결을 끊음
                            
                            # 수신된 데이터(위도,경도)를 디코딩
                            location_str = data.decode('utf-8').strip()
                            # print(f"수신된 원본 데이터: {location_str}") # 디버깅용

                            # 3. 데이터 파싱 (형식: "위도,경도")
                            try:
                                parts = location_str.split(',')
                                if len(parts) >= 2:
                                    lat = float(parts[0].strip())
                                    lon = float(parts[1].strip())
                                    
                                    # 4. NavSatFix 메시지 생성
                                    gps_msg = NavSatFix()
                                    gps_msg.header = Header()
                                    gps_msg.header.stamp = rospy.Time.now()
                                    gps_msg.header.frame_id = 'gps_link' # (선택) 적절한 frame_id로 변경
                                    
                                    gps_msg.latitude = lat
                                    gps_msg.longitude = lon
                                    gps_msg.altitude = 0.0 # 고도 정보가 없다면 0 또는 다른 값
                                    
                                    # GPS 수신 상태 (예: '수신 양호'로 가정)
                                    gps_msg.status.status = NavSatStatus.STATUS_FIX 
                                    gps_msg.status.service = NavSatStatus.SERVICE_GPS
                                    
                                    # 5. ROS 토픽 발행
                                    pub.publish(gps_msg)
                                    rospy.loginfo(f"Published to /destination_gps: lat={lat}, lon={lon}")
                                
                                else:
                                    rospy.logwarn(f"수신된 데이터 형식이 잘못되었습니다: {location_str}")

                            except (ValueError, IndexError) as e:
                                rospy.logerr(f"데이터 파싱 오류: {e} | 원본 데이터: {location_str}")
                            except Exception as e:
                                rospy.logerr(f"메시지 생성 중 알 수 없는 오류: {e}")

                        except socket.timeout:
                            # recv() 타임아웃. 루프를 계속 돌며 rospy.is_shutdown() 확인
                            continue
                        except (socket.error, UnicodeDecodeError) as e:
                            rospy.logwarn(f"클라이언트({addr}) 수신 중 오류: {e}")
                            break # 이 클라이언트와의 연결 종료
                    
                    rospy.loginfo(f"{addr} 연결 종료")

            except socket.timeout:
                # accept() 타임아웃. 루프를 계속 돌며 rospy.is_shutdown() 확인
                continue
            except socket.error as e:
                if rospy.is_shutdown():
                    break # ROS가 종료되면 소켓 대기 중단
                rospy.logwarn(f"소켓 accept 오류: {e}")
            except Exception as e:
                if rospy.is_shutdown():
                    break
                rospy.logerr(f"메인 루프 오류: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 노드가 종료되었습니다.")
    finally:
        rospy.loginfo("서버를 종료합니다.")