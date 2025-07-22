#! /usr/bin/env python3
import rospy,sys,os
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.linker_range_arc import *

#这个是接收到角度的函数
# def left_hand(msg):
#     p = list(msg.position)
#     if len(p) > 10:
#         p[11:11] = [0.0,0.0,0.0,0.0]#这个是将那四个预留关节插入进去
#         pos = arc_to_range_left(p)
#         left_joint_msg = create_joint_state_msg(position=pos)
#         left_pub.publish(left_joint_msg)#这个是发布范围
#     else:
#         pos = arc_to_range_left_10(p)
#         left_joint_msg = create_joint_state_msg(position=pos)
#         left_pub.publish(left_joint_msg)

#这个是接收到角度的函数
def left_hand(msg):
    p = list(msg.position)
    if len(p) > 19:
        #p[11:11] = [0.0,0.0,0.0,0.0]#这个是将那四个预留关节插入进去....我在pub之前已经插入了，将其变为了25个关节
        pos = arc_to_range_left_l25(p)
        left_joint_msg = create_joint_state_msg(position=pos)
        left_pub.publish(left_joint_msg)#这个是发布范围
    elif len(p)>10:
        p[11:11] = [0.0,0.0,0.0,0.0]#这个是将那四个预留关节插入进去
        pos = arc_to_range_left(p)
        left_joint_msg = create_joint_state_msg(position=pos)
        left_pub.publish(left_joint_msg)#这个是发布范围
    else:
        pos = arc_to_range_left_10(p)
        left_joint_msg = create_joint_state_msg(position=pos)
        left_pub.publish(left_joint_msg)
#这个是将范围转化为弧度的函数
def left_hand_state(msg):
    p = list(msg.position)
    if len(p) > 19:
        pos = range_to_arc_left_l25(p)
        left_state_msg = create_joint_state_msg(position=pos)
        left_state_pub.publish(left_state_msg)#这个是发布范围
    elif len(p)>10:
        p[11:11] = [0.0,0.0,0.0,0.0]
        pos = range_to_arc_left(p)
        left_state_msg = create_joint_state_msg(position=pos)
        left_state_pub.publish(left_state_msg)
    else:
        pos = range_to_arc_left_10(p)
        left_state_msg = create_joint_state_msg(position=pos)
        left_state_pub.publish(left_state_msg)


# def left_hand_state(msg):
#     p = list(msg.position)
#     if len(p) > 10:
#         p[11:11] = [0.0,0.0,0.0,0.0]
#         pos = range_to_arc_left(p)
#         left_state_msg = create_joint_state_msg(position=pos)
#         left_state_pub.publish(left_state_msg)
#     else:
#         pos = range_to_arc_left_10(p)
#         left_state_msg = create_joint_state_msg(position=pos)
#         left_state_pub.publish(left_state_msg)

def right_hand_state(msg):
    p = list(msg.position)
    if len(p)>19:
        pos = range_to_arc_right_l25(p)
        right_state_msg = create_joint_state_msg(position=pos)
        right_state_pub.publish(right_state_msg)
    elif len(p) > 10:
        pos = range_to_arc_right(p)
        right_state_msg = create_joint_state_msg(position=pos)
        right_state_pub.publish(right_state_msg)
    else:
        pos = range_to_arc_right_10(p)
        right_state_msg = create_joint_state_msg(position=pos)
        right_state_pub.publish(right_state_msg)
def right_hand(msg):
    p = list(msg.position)
    if len(p)>19:
        pos = arc_to_range_right_l25(p)
        right_state_msg = create_joint_state_msg(position=pos)
        #print(right_state_msg)                                                    #调试信息
        right_pub.publish(right_state_msg)
    elif len(p) > 10:
        p[11:11] = [0.0,0.0,0.0,0.0]
        pos = arc_to_range_right(p)
        right_joint_msg = create_joint_state_msg(position=pos)
        right_pub.publish(right_joint_msg)
    else:
        pos = arc_to_range_right_10(p)
        right_joint_msg = create_joint_state_msg(position=pos)
        right_pub.publish(right_joint_msg)

def create_joint_state_msg(position, names=[]):
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = names
    msg.position = list(map(float, position))
    msg.velocity = [0.0] * len(position)
    msg.effort = [0.0] * len(position)
    return msg

if __name__ == "__main__":
    rospy.init_node("hand_range_arc_conversion", anonymous=True)
    print("范围与弧度转换节点已启动")
    # 接收到角度话题
    left_sub = rospy.Subscriber("/cb_left_hand_control_cmd_arc",JointState,left_hand,queue_size=10)
    right_sub = rospy.Subscriber("/cb_right_hand_control_cmd_arc",JointState,right_hand,queue_size=10)
    # 将弧度转为范围后发布
    left_pub = rospy.Publisher("/cb_left_hand_control_cmd",JointState,queue_size=1)#这个是发布位置信息命令
    right_pub = rospy.Publisher("/cb_right_hand_control_cmd", JointState,queue_size=1)
    # 接收到状态话题
    left_state_sub = rospy.Subscriber("/cb_left_hand_state",JointState,left_hand_state,queue_size=10)
    right_state_sub = rospy.Subscriber("/cb_right_hand_state",JointState,right_hand_state,queue_size=10)
    # 将状态范围转为弧度后发布
    left_state_pub = rospy.Publisher("/cb_left_hand_state_arc",JointState,queue_size=1)
    right_state_pub = rospy.Publisher("/cb_right_hand_state_arc",JointState,queue_size=1)
    

    rospy.spin()