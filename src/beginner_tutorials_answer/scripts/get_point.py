#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def path_maker():
    # [중요 1] anonymous=True 제거 (런치파일 파라미터 읽기 위해 필수)
    rospy.init_node('path_maker_node')

    # latch=True: 늦게 오는 구독자에게도 마지막 메시지 보여주기
    pub_init = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

    # 런치파일 파라미터 읽기
    start_x = rospy.get_param('~start_x', 548.89) 
    start_y = rospy.get_param('~start_y', -811.16)
    goal_x = rospy.get_param('~goal_x', 640.03)
    goal_y = rospy.get_param('~goal_y', -698.76)

    # [중요 2] 받는 놈(다익스트라)이 준비될 때까지 기다리기
    rospy.loginfo("Waiting for mgeo_dijkstra_path node...")
    
    # 연결될 때까지 무한 대기 (Ctrl+C 누르면 종료)
    while pub_init.get_num_connections() == 0 or pub_goal.get_num_connections() == 0:
        if rospy.is_shutdown(): return
        rospy.sleep(0.5) # CPU 과부하 방지
    
    rospy.loginfo("Connected! Dijkstra node is ready.")

    # --- 메시지 생성 ---
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.stamp = rospy.Time.now()
    init_msg.header.frame_id = "map"
    init_msg.pose.pose.position.x = start_x
    init_msg.pose.pose.position.y = start_y
    init_msg.pose.pose.orientation.w = 1.0

    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"
    goal_msg.pose.position.x = goal_x
    goal_msg.pose.position.y = goal_y
    goal_msg.pose.orientation.w = 1.0

    # --- 메시지 발행 ---
    # Rviz처럼 확실하게 보내기 위해 약간의 텀을 두고 발행
    rospy.sleep(0.5)
    pub_init.publish(init_msg)
    rospy.loginfo(f"--> Sent Start: ({start_x}, {start_y})")
    
    rospy.sleep(0.5)
    pub_goal.publish(goal_msg)
    rospy.loginfo(f"--> Sent Goal : ({goal_x}, {goal_y})")

    rospy.loginfo(">> Coordinates sent. Keeping node alive to maintain message...")
    
    # [핵심] 노드가 죽지 않게 잡고 있어야 함!
    # 그래야 latch된 메시지가 메모리에 남아서 다익스트라 노드로 전달됨
    rospy.spin()

if __name__ == '__main__':
    try:
        path_maker()
    except rospy.ROSInterruptException:
        pass