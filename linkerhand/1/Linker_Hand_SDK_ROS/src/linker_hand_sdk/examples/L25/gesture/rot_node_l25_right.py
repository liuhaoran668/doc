#!/usr/bin/env python3
import rospy,rospkg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import std_msgs.msg
import time
import threading
import signal
import sys,os
import numpy as np


class linker:
    def __init__(self):
        rospy.init_node('dong_test_sender', anonymous=True)
        self.joint_state = JointState()
        self.hand_joint = "L25" 
        self.hand_type = "right" 
        self.prev_pos = self.pos = self.curr_pos=np.zeros(20)#20个关节位置
        
        self.pub1 = rospy.Publisher('/cb_right_hand_control_cmd_arc', JointState, queue_size=10)#将关节角度转化为255的


        #需要一个sub来接收收到的位置信息，，，，，这个sub是接受智能体发来的位置信息。在回调中发送给机械手
        rospy.Subscriber("/linker_node/cmd_ones", JointState, self._receive_ones)#他发送的是这个。这个就是接收isaac中位置信息的订阅，在这里将其发送给can通信
        

        rate = rospy.Rate(20)  # 设置频率为50Hz
        while not rospy.is_shutdown():  # 持续1秒
            rate.sleep()

    #this goes from [-1, 1] to [lower, upper]
    def scale(self,x, lower, upper):
        return (0.5 * (x + 1.0) * (upper - lower) + lower)
    #this goes from [lower, upper] to [-1, 1]
    def unscale(self,x, lower, upper):
        return (2.0 * x - upper - lower)/(upper - lower)



    ###Sim linker hand to real linker hand  Sim is allegro-like but all 16 joints are usable.  这个是将中心角度进行统一，sim和real的中心角度应该不同
    def linkersim_to_linkerhand(self,joints):
        joints = np.array(joints)
        ret_joints = joints + 3.14159   #在leaphand中是差180度
        return ret_joints

    #在这里需要将手部的限制给标出来，换成l25的
    def linker_limits(self,type = "regular"):
        sim_min = np.array([0,    0,   0,   0,  0, -1.3,-0.18,   0 ,    0,    -0.26,  -1.57,   0,     0,   0,    0,    -1.57,    0,    0,    0,    0])
        sim_max = np.array([0.9,1.57,1.57,1.57,1.57, 0 ,  0,    0.18,  0.18,   0.61,    0    ,1.57,  1.57,1.57, 1.57,     0, 1.57, 1.57, 1.57, 1.57])
        return sim_min, sim_max

    #Isaac has custom ranges from -1 to 1 so we convert that to linkerHand real world
    def sim_ones_to_linkerhand(self,joints, hack_thumb = False):
        sim_min, sim_max = self.linker_limits(type = hack_thumb)#将关节的限制存起来，就是在urdf中显示出来的关节限制,在这里就是ros中的
        print(sim_min)
        joints = self.scale(joints, sim_min, sim_max)####将输出的-1，1关节空间转换为lower和high之间的
        #joints = self.linkersim_to_linkerhand(joints)#这个是中心角度的设计，，查看sim和real的中心角度差多少,,,没搞懂这个中心角度是啥意思呢，暂时先不考虑
        return joints



    def _receive_ones(self, pose):####这个就是将收到的归一化的动作空间进行反归一化
        #pose就是接收到的智能体动作，在handcon里面已经将位置索引转化为real了。。  。。。。所以这个linker_limits应该是l25接收时的限制
        pose = self.sim_ones_to_linkerhand(np.array(pose.position))#
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        #下一步就是将位置发送给机械手让其动起来,,,这些位置坐标都需要修改，，智能体发送过来的是20个关节角度，但是can接收的是25个
        #我需要添加增加将这些发送回来的数据索引成真实手的关节索引（在handcon进行修改这个索引），！！！并且添加预留关节！！！
        
        #我现在想的是都在pub之前将预留的加上或者去掉
        add_pos = np.copy(self.curr_pos)
        # add_pos[7:7]=[0]
        # add_pos[11:11]=[0,0,0,0]
        add_pos = np.insert(add_pos, 7, 0)
        add_pos = np.insert(add_pos, 11, [0, 0, 0,0])
        # 创建JointState并添加头信息
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.position = add_pos.tolist()
         #print(joint_msg.position)                       #-------------------------------------------------调试信息
        self.pub1.publish(joint_msg)
        #pub1.publish(add_pos)##pub1是发送弧度给arc，然后arc转化为255发送出去






def signal_handler(sig, frame):

    print('You pressed Ctrl+C!')

    sys.exit(0)  # 0表示正常退出
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

    try:
        print("测试中")
        linker_node = linker()
    except KeyboardInterrupt:
         print("Caught KeyboardInterrupt, exiting gracefully.")
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
    finally:
         print("Cleaning up...")