#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

# --- [설정] 여기에 원하는 목표 좌표를 적으세요 ---
FIXED_LAT = 37.5786497   # 위도
FIXED_LON = 126.8873648  # 경도
# ----------------------------------------------

def main():
    # 1. 노드 이름 설정
    rospy.init_node('fixed_gps_publisher', anonymous=True)

    # 2. 퍼블리셔 생성 (토픽 이름: /destination_gps)
    pub = rospy.Publisher('/destination_gps', NavSatFix, queue_size=10)

    # 3. 발행 주기 설정 (1Hz = 1초에 1번, 필요하면 숫자를 바꾸세요)
    rate = rospy.Rate(1) 

    rospy.loginfo(f"고정 좌표 발행을 시작합니다: [{FIXED_LAT}, {FIXED_LON}]")

    while not rospy.is_shutdown():
        # 4. NavSatFix 메시지 채우기
        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = 'gps_link'  # 혹은 'map', 'base_link' 등 상황에 맞게
        
        gps_msg.latitude = FIXED_LAT
        gps_msg.longitude = FIXED_LON
        gps_msg.altitude = 0.0  # 고도 (필요 없으면 0)

        # GPS 상태 (정상 수신 상태로 가장)
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        # 5. 메시지 발행
        pub.publish(gps_msg)
        
        # 로그 출력 (잘 가고 있나 확인용)
        rospy.loginfo(f"Published /destination_gps: {FIXED_LAT}, {FIXED_LON}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass