`roslaunch rosbridge_server rosbridge_websocket.launch`

- roscore와 함께 작동함

`rostopic echo /Ego_topic`

- ego 차량 상태정보 확인하는 topic

`rostopic echo /GetTrafficLightStatus`

- 카메라를 통해 획득한 신호등 정보 확인

`rviz`

- 라이다 시각화로 정호 확인 가능 (미션 내 blackout, noise 구역 있음)
