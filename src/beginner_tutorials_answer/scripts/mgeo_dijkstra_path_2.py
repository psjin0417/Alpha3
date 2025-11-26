#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import copy
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point # Point 추가
from std_msgs.msg import Int32 # Int32 추가
from nav_msgs.msg import Path
from pathlib import Path as Saver

# MGeo 관련 경로 설정
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
from lib.mgeo.class_defs import *

class dijkstra_path_pub:
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)

        # 기존 경로 퍼블리셔
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        
        # [추가] 요청하신 2가지 퍼블리셔 설정
        self.start_flag_pub = rospy.Publisher('/Start_topic', Int32, queue_size=1)
        self.goal_point_pub = rospy.Publisher('/Goal_topic', Point, queue_size=1)

        # 맵 로딩
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/sangam'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)
        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes = node_set.nodes
        self.links = link_set.lines
        self.global_planner = Dijkstra(self.nodes, self.links)

        # 토픽 구독
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_callback)

        # 상태 플래그
        self.is_goal_pose = False
        self.is_init_pose = False

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        # [수정] 지속적인 Goal 전송을 위한 변수 초기화
        self.latest_goal_msg = None

        # 메인 루프
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # 두 좌표가 모두 들어왔는지 확인 (새로운 경로 생성 요청)
            if self.is_goal_pose and self.is_init_pose:
                rospy.loginfo(f"New Request Detected! Start: {self.start_node}, End: {self.end_node}")
                
                # 1. 경로 계산
                self.global_path_msg = self.calc_dijkstra_path_node(self.start_node, self.end_node)

                # 2. txt 파일 저장
                self.save_path_to_txt()

                # 3. 경로 발행
                self.global_path_pub.publish(self.global_path_msg)
                
                # ---------------------------------------------------------
                # [수정] Start Flag 발행 및 Goal 메시지 업데이트
                # ---------------------------------------------------------
                
                # (1) /Start_topic : 1 발행 (이벤트 트리거용으로 1회 발행)
                start_msg = Int32()
                start_msg.data = 1
                self.start_flag_pub.publish(start_msg)

                # (2) /Goal_topic : 메시지 생성 및 저장 (지속 전송을 위해 변수에 저장)
                goal_msg = Point()
                goal_msg.x = self.goal_x
                goal_msg.y = self.goal_y
                goal_msg.z = 0.0
                
                self.latest_goal_msg = goal_msg  # [중요] 최신 목표 지점 업데이트

                rospy.loginfo(f">>> Path Generated. Start_Flag=1 Sent. Goal Updated to ({self.goal_x:.2f}, {self.goal_y:.2f})")
                
                # 다음 요청을 위해 플래그 초기화
                self.is_goal_pose = False
                self.is_init_pose = False
                rospy.loginfo(">>> System Reset for Next Command (Goal will keep publishing).")

            # ---------------------------------------------------------
            # [수정] Goal Topic 지속 발행 로직 (루프 돌 때마다 항상 실행)
            # ---------------------------------------------------------
            if self.latest_goal_msg is not None:
                self.goal_point_pub.publish(self.latest_goal_msg)
                start_msg = Int32()
                start_msg.data = 1
                self.start_flag_pub.publish(start_msg)
            
            rate.sleep()
    
    def save_path_to_txt(self):
        full_path = os.path.expanduser("/home/autonav/alpha3/src/beginner_tutorials_answer/path/global.txt")

        dir_path = os.path.dirname(full_path)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

        with open(full_path, "w", encoding="utf-8") as f:
            for pos in self.global_path_msg.poses:
                f.write("{:.6f}\t{:.6f}\t0.0\n".format(pos.pose.position.x, pos.pose.position.y))
        
        print("\n" + "="*50)
        print(f"*** GLOBAL PATH SAVED SUCCESS ***")
        print(f"File Location: {full_path}")
        print("="*50 + "\n")

    def init_callback(self, msg):
        start_min_dis = float('inf')
        self.init_x = msg.pose.pose.position.x
        self.init_y = msg.pose.pose.position.y
        
        for node_idx in self.nodes:
            node_pose_x = self.nodes[node_idx].point[0]
            node_pose_y = self.nodes[node_idx].point[1]
            start_dis = sqrt(pow(self.init_x - node_pose_x, 2) + pow(self.init_y - node_pose_y, 2))
            if start_dis < start_min_dis:
                start_min_dis = start_dis
                self.start_node = node_idx

        self.is_init_pose = True
        rospy.loginfo(f"Start Point Received: ({self.init_x:.2f}, {self.init_y:.2f}) -> Map Node: {self.start_node}")

    def goal_callback(self, msg):
        goal_min_dis = float('inf')
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        for node_idx in self.nodes:
            node_pose_x = self.nodes[node_idx].point[0]
            node_pose_y = self.nodes[node_idx].point[1]
            goal_dis = sqrt(pow(self.goal_x - node_pose_x, 2) + pow(self.goal_y - node_pose_y, 2))

            if goal_dis < goal_min_dis:
                goal_min_dis = goal_dis
                self.end_node = node_idx

        self.is_goal_pose = True
        rospy.loginfo(f"Goal Point Received: ({self.goal_x:.2f}, {self.goal_y:.2f}) -> Map Node: {self.end_node}")

    def calc_dijkstra_path_node(self, start_node, end_node):
        result, path = self.global_planner.find_shortest_path(start_node, end_node)
        out_path = Path()
        out_path.header.frame_id = '/map'

        if not result:
            rospy.logwarn("Path finding failed!")
            return out_path

        for waypoint in path["point_path"]:
            read_pose = PoseStamped()
            read_pose.pose.position.x = waypoint[0]
            read_pose.pose.position.y = waypoint[1]
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)   

        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()

    def get_weight_matrix(self):
        weight = dict() 
        for from_node_id, from_node in self.nodes.items():
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            weight[from_node_id][from_node_id] = 0
            for to_node in from_node.get_to_nodes():
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
                weight[from_node_id][to_node.idx] = min_cost           
        return weight
    
    def find_shortest_link_leading_to_node(self,from_node, to_node):
        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)
        if len(to_links) == 0: raise BaseException('[ERROR] No link')
        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link
        return shortest_link, min_cost

    def find_nearest_node_idx(self, distance, s):        
        idx_list = list(self.nodes.keys())
        min_value = float('inf')
        min_idx = idx_list[-1]
        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx): 
        s = dict()
        from_node = dict() 
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx
        s[start_node_idx] = True
        distance = copy.deepcopy(self.weight[start_node_idx])
        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s)
            s[selected_node_idx] = True            
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx
        tracking_idx = end_node_idx
        node_path = [end_node_idx]
        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)     
        node_path.reverse()
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]
            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]
            shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
            link_path.append(shortest_link.idx)
        point_path = []        
        for link_id in link_path:
            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])
        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}

if __name__ == '__main__':
    dijkstra_path_pub = dijkstra_path_pub()