

1. 에고토픽 연결, IMU GPS 센서 연결 
2. roslaunch beginner_tutorials dijkstra_path_pub -> rviz
3. rosrun beginner_tutorials mission_master.py -> 
4. rosrun package main -> 전역경로 주행


목적지 생성 
5-1 destination_receiver.py -> 실행해서 라즈베리파이로부터 gps정보받아서 gps 토픽 퍼블리쉬 -> port ip 설정 필요
5-2 test_gps.py -> 임의적으로 목적지 gps토픽 퍼블리쉬 
