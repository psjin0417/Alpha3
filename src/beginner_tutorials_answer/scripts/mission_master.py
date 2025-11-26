#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import utm  # pip install utm 필요
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from morai_msgs.msg import EgoVehicleStatus 

# --- [설정] Map Origin Offset (제공해주신 값 적용) ---
MAP_OFFSET_EAST = 313008.55819800857
MAP_OFFSET_NORTH = 4161698.628368007

class PathCommander:
    def __init__(self):
        # Publisher 설정
        self.pub_start = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    def send_coordinates(self, start_xy, goal_xy):
        """
        start_xy: [x, y] 리스트 (Ego Local)
        goal_xy : [x, y] 리스트 (Converted ENU)
        """
        rospy.loginfo("Waiting for Path Planner connection...")
        rate = rospy.Rate(10)
        while (self.pub_start.get_num_connections() == 0 or 
               self.pub_goal.get_num_connections() == 0):
            if rospy.is_shutdown(): return
            rate.sleep()
        
        rospy.loginfo("Connected! Sending coordinates...")

        # 2. 출발지 메시지 생성
        start_msg = PoseWithCovarianceStamped()
        start_msg.header.stamp = rospy.Time.now()
        start_msg.header.frame_id = "map"
        start_msg.pose.pose.position.x = start_xy[0]
        start_msg.pose.pose.position.y = start_xy[1]
        start_msg.pose.pose.orientation.w = 1.0

        # 3. 도착지 메시지 생성
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_xy[0]
        goal_msg.pose.position.y = goal_xy[1]
        goal_msg.pose.orientation.w = 1.0

        # 4. 전송
        self.pub_start.publish(start_msg)
        rospy.sleep(0.1) 
        self.pub_goal.publish(goal_msg)

        rospy.loginfo(f"Command Sent!")
        rospy.loginfo(f" -> Start (Ego): {start_xy}")
        rospy.loginfo(f" -> Goal (GPS->ENU): {goal_xy}")

if __name__ == '__main__':
    rospy.init_node('mission_master_node')
    
    commander = PathCommander()

    try:
        # --- [Step 1] 현재 차량 위치 수신 ---
        rospy.loginfo("Waiting for /Ego_topic message...")
        ego_msg = rospy.wait_for_message("/Ego_topic", EgoVehicleStatus)
        
        current_x = ego_msg.position.x
        current_y = ego_msg.position.y
        
        start_point = [current_x, current_y]
        rospy.loginfo(f"Ego Position Received: {start_point}")

        # --- [Step 2] GPS 목적지 수신 및 ENU 변환 ---
        rospy.loginfo("Waiting for /destination_gps message...")
        
        # /destination_gps 토픽 대기
        gps_msg = rospy.wait_for_message("/destination_gps", NavSatFix)
        
        lat = gps_msg.latitude
        lon = gps_msg.longitude
        
        rospy.loginfo(f"GPS Destination Received: Lat {lat}, Lon {lon}")

        # 1. 위도/경도 -> UTM 변환 (Zone 52 강제)
        # 반환값: (easting, northing, zone_number, zone_letter)
        utm_easting, utm_northing, zone_num, zone_letter = utm.from_latlon(lat, lon, force_zone_number=52)

        # 2. Global UTM -> Local ENU 변환 (Offset 적용)
        # Local East = UTM East - Map Offset East
        # Local North = UTM North - Map Offset North
        goal_x = utm_easting - MAP_OFFSET_EAST
        goal_y = utm_northing - MAP_OFFSET_NORTH
        
        # 최종 Goal Point 설정
        goal_point = [goal_x, goal_y]

        # 검증용 로그 출력
        rospy.loginfo(f"--- Coordinate Conversion ---")
        rospy.loginfo(f"UTM Raw   : E={utm_easting:.4f}, N={utm_northing:.4f} (Zone {zone_num})")
        rospy.loginfo(f"Offset    : E={MAP_OFFSET_EAST}, N={MAP_OFFSET_NORTH}")
        rospy.loginfo(f"Local ENU : x={goal_x:.4f}, y={goal_y:.4f}")
        
        # (참고) 입력하신 예시 GPS (37.57562065, 126.88978698)가 들어오면
        # 목표값인 [629.68, -858.12] 와 유사하게 나오는지 로그로 확인 가능합니다.

        # --- [Step 3] 좌표 전송 ---
        commander.send_coordinates(start_point, goal_point)

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error occurred: {e}")