### TurtleBot Control

```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

```bash
# turn on the TB3's camera
$ roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```

### Run ROS Simulate

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Run SLAM Mode

```bash
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### BringUp Mode

```bash
# SBC
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

![img](./img/slam_mode.png)

bingup robot

### Log 분석하기

- Topic log message 읽고 분석하기

### 멤모

- in -> ex -> calib -> action
- SBC에 bringup 미리해도 된다
- mission에서는 ex없음 (roscore에서 켜줌)
  - `mission:=traffic_light`
  - `rostopic ...` 으로 topic 전송
    - data 3: 미션 실행해라, 2: lane 따라가
