# --------------------------------------------------------
# LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning
# https://arxiv.org/abs/2309.06440
# Copyright (c) 2023 Ananye Agarwal
# Licensed under The MIT License [see LICENSE for details]
# --------------------------------------------------------
# Based on:
# https://github.com/HaozhiQi/hora/blob/main/hora/algo/deploy/robots/leap.py
# --------------------------------------------------------

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import numpy as np
import threading
# from leap_hand.srv import *

class LeapHand(object):
    def __init__(self):
        """ Simple python interface to the leap Hand.

        The leapClient is a simple python interface to an leap
        robot hand.  It enables you to command the hand directly through
        python library calls (joint positions, joint torques, or 'named'
        grasps).

        The constructors sets up publishers and subscribes to the joint states
        topic for the hand.
        """

        # Topics (that can be remapped) for named graps
        # (ready/envelop/grasp/etc.), joint commands (position and
        # velocity), joint state (subscribing), and envelop torque. Note that
        # we can change the hand topic prefix (for example, to leapHand_0)
        # instead of remapping it at the command line.

        # 初始化 joint state 缓存和线程锁
        self.joint_state = None
        self._lock = threading.Lock()

        topic_joint_command = '/linker_node/cmd_ones'

        # Publishers for above topics.
        self.pub_joint = rospy.Publisher(topic_joint_command, JointState, queue_size=10)###将智能体的动作进行pub
        #rospy.Subscriber(topic_joint_state, JointState, self._joint_state_callback)
        #self._joint_state = None
        #self.joint_state = None

        #-----
        #self.leap_position = rospy.ServiceProxy('/leap_position', leap_position)#这个接受硬件传来的位姿，
        
        rospy.Subscriber("/cb_right_hand_state_arc", JointState, self.joint_state_callback)#这个是接收到arc转换后的机械手位置，有25个
        #------
        
        
        
        #self.leap_effort = rospy.ServiceProxy('/leap_effort', leap_effort)

    #def _joint_state_callback(self, data):
    #self._joint_state = data
    
    #--------------
    def joint_state_callback(self, msg):
           #"""后台回调函数：持续接收来自硬件的关节状态"""------都利用arc进行中转，中转之后就会得到想要的弧度和范围
        with self._lock:
            self.joint_state = np.array(msg.position)###
    #--------------
    
    def sim_to_real(self, values):
        return values[self.sim_to_real_indices]#sim和real的索引不同

    def command_joint_position(self, desired_pose):
        """
            Takes as input self.cur_targets (raw joints) in numpy array
        """

        # Check that the desired pose can have len() applied to it, and that
        # the number of dimensions is the same as the number of hand joints.
        if (not hasattr(desired_pose, '__len__') or
                len(desired_pose) != 20):
            rospy.logwarn('Desired pose must be a {}-d array: got {}.'
                          .format(20, desired_pose))
            return False

        msg = JointState()  # Create and publish

        # convert desired_pose to ros_targets     将希望的pose进行归一化
        desired_pose = (2 * desired_pose - self.leap_dof_lower - self.leap_dof_upper) / (self.leap_dof_upper - self.leap_dof_lower)
        
        ###我可以在这里添加5个预留关节的位置，然后按照索引顺序进行重排列，不用在linkernode里面弄了。。。。。
        #desired_pose=np.append(desired_pose, [0.0] * 5)#这个是加上五个预留关节

        desired_pose = self.sim_to_real(desired_pose) #下面就是将索引给修改一下
        try:
            msg.position = desired_pose
            #print(msg.position)
            self.pub_joint.publish(msg)
            #print('1')
            rospy.logdebug('Published desired pose.')
            return True
        except rospy.exceptions.ROSSerializationException:
            rospy.logwarn('Incorrect type for desired pose: {}.'.format(
                desired_pose))
            return False

    def real_to_sim(self, values):
        return values[self.real_to_sim_indices]

    def poll_joint_position(self):
        """ Get the current joint positions of the hand.

        :param wait: If true, waits for a 'fresh' state reading.
        :return: Joint positions, or None if none have been received.
        """


        with self._lock:
            if self.joint_state is None:
                rospy.logwarn_throttle(2.0, "Joint state not received yet.")
                return None, None

            joint_position = self.joint_state.copy()
        #joint_position = np.array(self.leap_position().position)#他从服务接口中获取位置，，，，leaphand是只获取了位置，没有对effort进行设置
        #joint_effort = np.array(self.leap_effort().effort)
        #print(joint_position)
        #在这里将接收到的位置关节进行剔除,,,接收到的是25个关节角度的数组
        indices_to_remove = [7, 11, 12, 13, 14]
        joint_position=np.delete(joint_position, indices_to_remove)##将预留的位置关节进行剔除
        #print(joint_position)#确实是裁减之后的
        joint_position = self.LEAPhand_to_sim_ones(joint_position)
        joint_position = self.real_to_sim(joint_position)#这个会根据real2sim索引顺序将joint_position数据进行重新排列，
        joint_position = (self.leap_dof_upper - self.leap_dof_lower) * (joint_position + 1) / 2 + self.leap_dof_lower#还原到真实的上面？

        return (joint_position, None)

    def LEAPsim_limits(self):
        sim_min = self.sim_to_real(self.leap_dof_lower)
        sim_max = self.sim_to_real(self.leap_dof_upper)
        
        return sim_min, sim_max

    def LEAPhand_to_LEAPsim(self, joints):
        joints = np.array(joints)
        ret_joints = joints - 3.14
        return ret_joints

    def LEAPhand_to_sim_ones(self, joints):
        #joints = self.LEAPhand_to_LEAPsim(joints)  #这个是将sim和real的中心角度进行更改的，等测试之后在看
        sim_min, sim_max = self.LEAPsim_limits()
        joints = unscale_np(joints, sim_min, sim_max)#这个在我理解的应该是将超出关节限制的那些关节强制映射到-1，1.。然后在后面在映射成物理角度，这样就没有了超出限制的了
        
        return joints

def unscale_np(x, lower, upper):    #将其转化为-1到1的值
    return (2.0 * x - upper - lower)/(upper - lower)


