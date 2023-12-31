# 자율주행 경진대회 대비 ROS 교육

> ‼️ 주의사항
>
> 1.  명령어 확인
> 2.  경로 확인
> 3.  적용 반영 확인
> 4.  파일 권한 확인

### ROS 통신 방식

<table>
   <tr>
      <th/>
      <th>방향성</th>
      <th>동기성</th>
      <th>지속성</th>
   </tr>
   <tr>
      <th>Topic</th>
      <td>단방향</td>
      <td>비동기</td>
      <td>지속적</td>
   </tr>
   <tr>
      <th>Service</th>
      <td>양방향</td>
      <td>동기</td>
      <td>일시적</td>
   </tr>
   <tr>
      <th>Action</th>
      <td>양방향</td>
      <td>비동기</td>
      <td>지속적, 일시적</td>
   </tr>
</table>

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
4. rqt

### ROS Topic을 확인 목록

1. `<<TOPIC_NAME>>`
2. `<ESSAGE_TYPE>>`
3. `<<MESSAGE의 항목>>`
