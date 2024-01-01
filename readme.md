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
3. `roslaunch <<PACKAGE_NAME>> <<LAUNCH_FILE_NAME>>` : PAPACKAGE_NAME에 있는 단일 LAUNCH_FILE_NAME을 실행하는 명령어
4. `rostopic list | info | echo | pub` : 현재 ROS MASTER내에 토픽에 대해 리스트, 정보, 메세지, 수신, 발신하는 명령에
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

### source code 유의사항

1. 공통사항
   1. ros_node 이름 설정
   2. ros_node 역할 설정
2. pub (active)
   1. publish
   2. rate time 설정
   3. rate 실행
3. subse (passive)
   1. callback function

### 소스 코드 예제

```py
# 01pub.py
#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
# msg type
from std_msgs.msg import Int32

# setting node name
rospy.init_node('edu_pub_node')
# node role
pub = rospy.Publisher("/counter", Int32, queue_size=1)
int_msg = Int32()
# limit ros rate (Hz)
rospy.Rate(10)

num = 0
while not rospy.is_shutdown():
    num = num + 1
    int_msg.data = num
    # sned msg
    pub.publish(int_msg)
```

```py
# 02.sub.py
#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
# msg type
from std_msgs.msg import Int32

# sub's callback fn
def CB(msg):
    print(msg)

# setting node name
rospy.init_node('edu_sub_node')
# node role
rospy.Subscriber("counter", Int32, callback=CB)
rospy.spin()
```

### 클래스 사용 시

```py
# 00.class_form.py
#!/usr/bin/env python3
#-*-codingutf-8 -*-

class Class_ex:
    def __init__(self):
        self.data = 0

    def func(self):
        pass

def main():
    try :
        class_name = Class_ex()
        class_name.func()
    except :
        pass

# main fn
if __name__ == "__main__":
    main()
```

```py
# 03.class_pub.py
#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
from std_msgs.msg import Int32

class Class_pub:
    def __init__(self):
        # name
        rospy.init_node('node_pub')
        # role
        self.pub = rospy.Publisher('/counter', Int32, queue_size=1)
        # type
        self.int_msg = Int32()
        # rate (Hz)
        self.rate = rospy.Rate(11)

    def func(self):
        num = 0
        while not rospy.is_shutdown():
            num = num + 1
            self.int_msg.data = num
            # sned msg
            self.pub.publish(self.int_msg)
            # set rate
            self.rate.sleep()

def main():
    try :
        class_pub = Class_pub()
        class_pub.func()
    except rospy.ROSInterruptException:
        pass

# main fn
if __name__ == "__main__":
    main()
```

```py
# 04.class_sub.py
#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
from std_msgs.msg import Int32

class Class_sub:
    def __init__(self):
        # name
        rospy.init_node('node_sub')
        # role
        rospy.Subscriber('/counter', Int32, callback=self.CB)

    # sub's callback fn
    def CB(msg):
        print(msg)

def main():
    try :
        class_sub = Class_sub()
        # run
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# main fn
if __name__ == "__main__":
    main()
```
