# 자율주행 경진대회 대비 ROS 교육

### ROS 관련 명령어

1. `roscore` : ROS MASTER를 실행하는 명령어
   > `roscore` -> `rosrun ~~`
2. `rosrun <<PACKAGE NAME>> <<FILE NAME>>` : PACKAGE NAME에 있는 단일 FILE NAME을 실행하는 명령어
3. `roslaunch <<PACKAGE_NAME>><<LAUNCH_FILE_NAME>>` : PAPACKAGE_NAME에 있는 단일 LAUNCH_FILE_NAME을 실행하는 명령어
4. `rostopic list, info, echo, pub` : 현재 ROS MASTER내에 토픽에 대해 리스트, 정보, 메세지, 수신, 발신하는 명령에
   - `rostopic list` : ROS MASTER 내에 topic list를 확인하는 명령어
   - `rostopic info <<TOPIC_NAME>>` : TOPIC_NAME의 메세지(데이터)를 확인하는 명령어
   - `rostopic echo <<TOPIC_NAME>>` : TOPIC_NAME의 정보를 수신하는 명령어
   - `rostopic pub <<TOPIC_NAME>> <<MESSAGE_TYPE>> <<ARGS>>` : TOPIC_NAME의 메세지 (데이터)를 발신하는 명령어

### ROS Tools

1. rat_graph : ros의 시스템 구조 그래프 확인
2. rviz
3. rat_image_view
