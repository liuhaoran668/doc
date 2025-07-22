#!/usr/bin/env python3
import can
import time,sys,os
import threading
import numpy as np
from enum import Enum
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)
from utils.color_msg import ColorMsg

class FrameProperty(Enum):
    INVALID_FRAME_PROPERTY = 0x00  # 无效的can帧属性 | 无返回
    # 并行指令区域
    ROLL_POS = 0x01  # 横滚关节位置 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度 [10,11,12,13,14]
    YAW_POS = 0x02  # 航向关节位置 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度 [5,6,7,8,9]
    ROOT1_POS = 0x03  # 指根1关节位置 | 最接近手掌的指根关节 [0,1,2,3,4]
    ROOT2_POS = 0x04  # 指根2关节位置 | 最接近手掌的指根关节  [15, 16,17,18,19]
    ROOT3_POS = 0x05  # 指根3关节位置 | 最接近手掌的指根关节 暂无
    TIP_POS = 0x06  # 指尖关节位置 | 最接近手掌的指根关节 [20,21,22,23,24]

    ROLL_SPEED = 0x09  # 横滚关节速度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_SPEED = 0x0A  # 航向关节速度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_SPEED = 0x0B  # 指根1关节速度 | 最接近手掌的指根关节
    ROOT2_SPEED = 0x0C  # 指根2关节速度 | 最接近手掌的指根关节
    ROOT3_SPEED = 0x0D  # 指根3关节速度 | 最接近手掌的指根关节
    TIP_SPEED = 0x0E  # 指尖关节速度 | 最接近手掌的指根关节

    ROLL_TORQUE = 0x11  # 横滚关节扭矩 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_TORQUE = 0x12  # 航向关节扭矩 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_TORQUE = 0x13  # 指根1关节扭矩 | 最接近手掌的指根关节
    ROOT2_TORQUE = 0x14  # 指根2关节扭矩 | 最接近手掌的指根关节
    ROOT3_TORQUE = 0x15  # 指根3关节扭矩 | 最接近手掌的指根关节
    TIP_TORQUE = 0x16  # 指尖关节扭矩 | 最接近手掌的指根关节

    ROLL_FAULT = 0x19  # 横滚关节故障码 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_FAULT = 0x1A  # 航向关节故障码 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_FAULT = 0x1B  # 指根1关节故障码 | 最接近手掌的指根关节
    ROOT2_FAULT = 0x1C  # 指根2关节故障码 | 最接近手掌的指根关节
    ROOT3_FAULT = 0x1D  # 指根3关节故障码 | 最接近手掌的指根关节
    TIP_FAULT = 0x1E  # 指尖关节故障码 | 最接近手掌的指根关节

    ROLL_TEMPERATURE = 0x21  # 横滚关节温度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    YAW_TEMPERATURE = 0x22  # 航向关节温度 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度
    ROOT1_TEMPERATURE = 0x23  # 指根1关节温度 | 最接近手掌的指根关节
    ROOT2_TEMPERATURE = 0x24  # 指根2关节温度 | 最接近手掌的指根关节
    ROOT3_TEMPERATURE = 0x25  # 指根3关节温度 | 最接近手掌的指根关节
    TIP_TEMPERATURE = 0x26  # 指尖关节温度 | 最接近手掌的指根关节
    # 并行指令区域

    # 串行指令区域
    THUMB_POS = 0x41  # 大拇指指关节位置 | 返回本类型数据
    INDEX_POS = 0x42  # 食指关节位置 | 返回本类型数据
    MIDDLE_POS = 0x43  # 中指关节位置 | 返回本类型数据
    RING_POS = 0x44  # 无名指关节位置 | 返回本类型数据
    LITTLE_POS = 0x45  # 小拇指关节位置 | 返回本类型数据

    THUMB_SPEED = 0x49  # 大拇指速度 | 返回本类型数据
    INDEX_SPEED = 0x4A  # 食指速度 | 返回本类型数据
    MIDDLE_SPEED = 0x4B  # 中指速度 | 返回本类型数据
    RING_SPEED = 0x4C  # 无名指速度 | 返回本类型数据
    LITTLE_SPEED = 0x4D  # 小拇指速度 | 返回本类型数据

    THUMB_TORQUE = 0x51  # 大拇指扭矩 | 返回本类型数据
    INDEX_TORQUE = 0x52  # 食指扭矩 | 返回本类型数据
    MIDDLE_TORQUE = 0x53  # 中指扭矩 | 返回本类型数据
    RING_TORQUE = 0x54  # 无名指扭矩 | 返回本类型数据
    LITTLE_TORQUE = 0x55  # 小拇指扭矩 | 返回本类型数据

    THUMB_FAULT = 0x59  # 大拇指故障码 | 返回本类型数据
    INDEX_FAULT = 0x5A  # 食指故障码 | 返回本类型数据
    MIDDLE_FAULT = 0x5B  # 中指故障码 | 返回本类型数据
    RING_FAULT = 0x5C  # 无名指故障码 | 返回本类型数据
    LITTLE_FAULT = 0x5D  # 小拇指故障码 | 返回本类型数据

    THUMB_TEMPERATURE = 0x61  # 大拇指温度 | 返回本类型数据
    INDEX_TEMPERATURE = 0x62  # 食指温度 | 返回本类型数据
    MIDDLE_TEMPERATURE = 0x63  # 中指温度 | 返回本类型数据
    RING_TEMPERATURE = 0x64  # 无名指温度 | 返回本类型数据
    LITTLE_TEMPERATURE = 0x65  # 小拇指温度 | 返回本类型数据
    # 串行指令区域

    # 合并指令区域，同一手指非必要单控数据合并
    FINGER_SPEED = 0x81  # 手指速度 | 返回本类型数据
    FINGER_TORQUE = 0x82  # 转矩 | 返回本类型数据
    FINGER_FAULT = 0x83  # 手指故障码 | 返回本类型数据

    # 指尖传感器数据组
    HAND_NORMAL_FORCE = 0x90  # 五指法向压力
    HAND_TANGENTIAL_FORCE = 0x91  # 五指切向压力
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # 五指切向方向
    HAND_APPROACH_INC = 0x93  # 五指接近感应

    THUMB_ALL_DATA = 0x98  # 大拇指所有数据
    INDEX_ALL_DATA = 0x99  # 食指所有数据
    MIDDLE_ALL_DATA = 0x9A  # 中指所有数据
    RING_ALL_DATA = 0x9B  # 无名指所有数据
    LITTLE_ALL_DATA = 0x9C  # 小拇指所有数据
    # 动作指令 ·ACTION
    ACTION_PLAY = 0xA0  # 动作

    # 配置命令·CONFIG
    HAND_UID = 0xC0  # 设备唯一标识码
    HAND_HARDWARE_VERSION = 0xC1  # 硬件版本
    HAND_SOFTWARE_VERSION = 0xC2  # 软件版本
    HAND_COMM_ID = 0xC3  # 设备id
    HAND_FACTORY_RESET = 0xCE  # 恢复出厂设置
    HAND_SAVE_PARAMETER = 0xCF  # 保存参数

    WHOLE_FRAME = 0xF0  # 整帧传输 | 返回一字节帧属性+整个结构体485及网络传输专属

class LinkerHandL25Can:
    def __init__(self, config, can_channel='can0', baudrate=1000000, can_id=0x28):
        self.config = config
        self.can_id = can_id
        self.running = True
        self.last_thumb_pos, self.last_index_pos,self.last_ring_pos,self.last_middle_pos, self.last_little_pos = None,None,None,None,None
        self.x01, self.x02, self.x03, self.x04,self.x05,self.x06,self.x07, self.x08,self.x09,self.x0A,self.x0B,self.x0C,self.x0D,self.x0E,self.speed = [],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
        self.last_root1,self.last_yaw,self.last_roll,self.last_root2,self.last_tip = None,None,None,None,None
        # 速度
        self.x49, self.x4a, self.x4b, self.x4c, self.x4d,self.xc1 = [],[],[],[],[],[]
        self.x41,self.x42,self.x43,self.x44,self.x45 = [],[],[],[],[]
        # 扭矩
        self.x51, self.x52, self.x53, self.x54,self.x55 = [],[],[],[],[]
        # 故障码
        self.x59,self.x5a,self.x5b,self.x5c,self.x5d = [],[],[],[],[]
        # 温度阈值
        self.x61,self.x62,self.x63,self.x64,self.x65 = [],[],[],[],[]
        # 根据操作系统初始化 CAN 总线
        if sys.platform == "linux":
            self.bus = can.interface.Bus(
                channel=can_channel, interface="socketcan", bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        elif sys.platform == "win32":
            self.bus = can.interface.Bus(
                channel='PCAN_USBBUS1', interface='pcan', bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        else:
            raise EnvironmentError("Unsupported platform for CAN interface")

        # 根据 can_id 初始化 publisher 和相关参数
        if can_id == 0x28:  # 左手
            self.hand_exists = config['LINKER_HAND']['LEFT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['LEFT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['LEFT_HAND']['NAME']
        elif can_id == 0x27:  # 右手

            self.hand_exists = config['LINKER_HAND']['RIGHT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['RIGHT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['RIGHT_HAND']['NAME']


        # 启动接收线程
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def send_command(self, frame_property, data_list):
        """
        发送命令到 CAN 总线
        :param frame_property: 数据帧属性
        :param data_list: 数据载荷
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            #print(f"Message sent: ID={hex(self.can_id)}, Data={data}")
        except can.CanError as e:
            print(f"Failed to send message: {e}")
        time.sleep(0.003)

    def receive_response(self):
        """
        接收并处理 CAN 总线的响应消息
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)  # 阻塞接收，1 秒超时
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")
    

    def set_joint_positions(self, joint_ranges):
        if len(joint_ranges) == 25:
            l25_pose = self.joint_map(joint_ranges)
            # 使用列表推导式将列表每6个元素切成一个子数组
            chunks = [l25_pose[i:i+6] for i in range(0, 30, 6)]
            # if self._list_d_value(self.last_thumb_pos, chunks[0]):
            #     self.last_thumb_pos=chunks[0]
            #     self.send_command(FrameProperty.THUMB_POS, chunks[0])
            # if self._list_d_value(self.last_index_pos, chunks[1]):
            #     self.last_index_pos=chunks[1]
            #     self.send_command(FrameProperty.INDEX_POS, chunks[1])
            # if self._list_d_value(self.last_middle_pos, chunks[2]):
            #     self.last_middle_pos = chunks[2]
            #     self.send_command(FrameProperty.MIDDLE_POS, chunks[2])
            # if self._list_d_value(self.last_ring_pos, chunks[3]):
            #     self.last_ring_pos = chunks[3]
            #     self.send_command(FrameProperty.RING_POS, chunks[3])
            # if self._list_d_value(self.last_little_pos, chunks[4]):
            #     self.last_little_pos = chunks[4]
            #     self.send_command(FrameProperty.LITTLE_POS, chunks[4])

            self.send_command(FrameProperty.THUMB_POS, chunks[0])
            time.sleep(0.001)
            self.send_command(FrameProperty.INDEX_POS, chunks[1])
            time.sleep(0.001)
            self.send_command(FrameProperty.MIDDLE_POS, chunks[2])
            time.sleep(0.001)
            self.send_command(FrameProperty.RING_POS, chunks[3])
            time.sleep(0.001)
            self.send_command(FrameProperty.LITTLE_POS, chunks[4])
            time.sleep(0.001)

    def set_joint_positions_by_topic(self, joint_ranges):
        if len(joint_ranges) == 25:
            #ROLL_POS = 0x01  # 横滚关节位置 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度 [10,11,12,13,14]
            #YAW_POS = 0x02  # 航向关节位置 | 坐标系建在每个手指的指根部位，按手指伸直的状态去定义旋转角度 [5,6,7,8,9]
            #ROOT1_POS = 0x03  # 指根1关节位置 | 最接近手掌的指根关节 [0,1,2,3,4]
            #ROOT2_POS = 0x04  # 指根2关节位置 | 最接近手掌的指根关节  [15, 16,17,18,19]
            #ROOT3_POS = 0x05  # 指根3关节位置 | 最接近手掌的指根关节 暂无
            #TIP_POS = 0x06  # 指尖关节位置 | 最接近手掌的指根关节 [20,21,22,23,24]
            # ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]
            
            l25_pose = self.slice_list(joint_ranges,5)
            if self._list_d_value(self.last_root1, l25_pose[0]):
                self.set_root1_positions(l25_pose[0])
                self.last_root1 = l25_pose[0]
            if self._list_d_value(self.last_yaw, l25_pose[1]):  
                self.set_yaw_positions(l25_pose[1])
                self.last_yaw = l25_pose[1]
            if self._list_d_value(self.last_roll, l25_pose[2]):
                self.set_roll_positions(l25_pose[2])
                self.last_roll = l25_pose[2]
            if self._list_d_value(self.last_root2, l25_pose[3]):
                self.set_root2_positions(l25_pose[3])
                self.last_root2 = l25_pose[3]
            if self._list_d_value(self.last_tip, l25_pose[4]): 
                self.set_tip_positions(l25_pose[4])
                self.last_tip = l25_pose[4]
            

    def slice_list(self, input_list, slice_size):
        """
        将一个列表按指定大小切片。
    
        参数:
        input_list (list): 需要切片的列表。
        slice_size (int): 每个切片的元素个数。
    
        返回:
        list of lists: 切片后的列表。
        """
        # 使用列表推导式进行切片
        sliced_list = [input_list[i:i + slice_size] for i in range(0, len(input_list), slice_size)]
        return sliced_list

    def _list_d_value(self,list1, list2):
        if list1 == None:
            return True
        for a, b in zip(list1, list2):
            if abs(b - a) > 2:
                return True
                break
        return False
    # 设置所有手指横滚关节位置
    def set_roll_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    # 设置所有手指航向关节位置
    def set_yaw_positions(self, joint_ranges):
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    # 设置所有手指指根1关节位置
    def set_root1_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    # 设置所有手指指根2关节位置
    def set_root2_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    # 设置所有手指指根3关节位置
    def set_root3_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    # 设置所有手指指尖关节位置
    def set_tip_positions(self, joint_ranges=[80]*5):
        self.send_command(FrameProperty.TIP_POS, joint_ranges)
    # 设置大拇指扭矩
    def set_thumb_torque(self, j=[]):
        self.send_command(FrameProperty.THUMB_TORQUE, j)
    # 设置食指扭矩
    def set_index_torque(self, j=[]):
        self.send_command(FrameProperty.INDEX_TORQUE, j)
    # 设置中指扭矩
    def set_middle_torque(self, j=[]):
        self.send_command(FrameProperty.MIDDLE_TORQUE, j)
    # 设置无名指扭矩
    def set_ring_torque(self, j=[]):
        self.send_command(FrameProperty.RING_TORQUE, j)
    # 设置小拇指扭矩
    def set_little_torque(self, j=[]):
        self.send_command(FrameProperty.LITTLE_TORQUE, j)

    # 获取大拇指指关节位置
    def get_thumb_positions(self,j=[0]):
        self.send_command(FrameProperty.THUMB_POS, j)
    # 获取食指关节位置
    def get_index_positions(self, j=[0]):
        self.send_command(FrameProperty.INDEX_POS,j)
    # 获取中指关节位置
    def get_middle_positions(self, j=[0]):
        self.send_command(FrameProperty.MIDDLE_POS,j)
    # 获取无名指关节位置
    def get_ring_positions(self, j=[0]):
        self.send_command(FrameProperty.RING_POS,j)
    # 获取小拇指关节位置
    def get_little_positions(self, j=[0]):
        self.send_command(FrameProperty.LITTLE_POS, j)
    # 大拇指所有电机故障码
    def get_thumbn_fault(self,j=[]):
        self.send_command(FrameProperty.THUMB_FAULT,j)
    # 食指指所有电机故障码
    def get_index_fault(self,j=[]):
        self.send_command(FrameProperty.INDEX_FAULT,j)
    # 中指所有电机故障码
    def get_middle_fault(self,j=[]):
        self.send_command(FrameProperty.MIDDLE_FAULT,j)
    # 无名指所有电机故障码
    def get_ring_fault(self,j=[]):
        self.send_command(FrameProperty.RING_FAULT,j)
    # 小拇指所有电机故障码
    def get_little_fault(self,j=[]):
        self.send_command(FrameProperty.LITTLE_FAULT,j)
    # 大拇指温度阈值
    def get_thumb_threshold(self,j=[]):
        self.send_command(FrameProperty.THUMB_TEMPERATURE, '')
    # 食指指温度阈值
    def get_index_threshold(self,j=[]):
        self.send_command(FrameProperty.INDEX_TEMPERATURE, j)
    # 中指温度阈值
    def get_middle_threshold(self,j=[]):
        self.send_command(FrameProperty.MIDDLE_TEMPERATURE, j)
    # 无名指温度阈值
    def get_ring_threshold(self,j=[]):
        self.send_command(FrameProperty.RING_TEMPERATURE, j)
    # 小拇指温度阈值
    def get_little_threshold(self,j=[]):
        self.send_command(FrameProperty.LITTLE_TEMPERATURE, j)

    # 失能01模式
    def set_disability_mode(self, j=[1,1,1,1,1]):
        self.send_command(0x85,j)
    # 使能00模式
    def set_enable_mode(self, j=[00,00,00,00,00]):
        self.send_command(0x85,j)
    
    # 设置所有手指扭矩
    def set_torque(self,j=[250]*5):
        t = j[0]
        i = j[1]
        m = j[2]
        r = j[3]
        l = j[4]
        self.set_thumb_torque(j=[t]*5)
        self.set_index_torque(j=[i]*5)
        self.set_middle_torque(j=[m]*5)
        self.set_ring_torque(j=[r]*5)
        self.set_little_torque(j=[l]*5)
    
    def set_speed(self, speed):
        self.speed = speed
        ColorMsg(msg=f"L25设置速度为:{self.speed}", color="yellow")
        thumb_speed = [self.speed[0]]*5
        index_speed = [self.speed[1]]*5
        middle_speed = [self.speed[2]]*5
        ring_speed = [self.speed[3]]*5
        little_speed = [self.speed[4]]*5
        self.send_command(FrameProperty.THUMB_SPEED, thumb_speed)
        self.send_command(FrameProperty.INDEX_SPEED, index_speed)
        self.send_command(FrameProperty.MIDDLE_SPEED, middle_speed)
        self.send_command(FrameProperty.RING_SPEED, ring_speed)
        self.send_command(FrameProperty.LITTLE_SPEED, little_speed)
        
    def set_finger_torque(self, torque):
        self.send_command(0x42, torque)

    def request_device_info(self):
        self.send_command(0xC0, [0])
        self.send_command(0xC1, [0])
        self.send_command(0xC2, [0])

    def save_parameters(self):
        self.send_command(0xCF, [])
    def process_response(self, msg):
        if msg.arbitration_id == self.can_id:
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            if frame_type == 0x01:
                self.x01 = list(response_data)
            elif frame_type == 0x02:
                self.x02 = list(response_data)
            elif frame_type == 0x03:
                self.x03 = list(response_data)
            elif frame_type == 0x04:
                self.x04 = list(response_data)
            elif frame_type == 0x05:
                self.x05 = list(response_data)
            elif frame_type == 0x06:
                self.x06 = list(response_data)
            elif frame_type == 0xC0:
                print(f"Device ID info: {response_data}")
                if self.can_id == 0x28:
                    self.right_hand_info = response_data
                elif self.can_id == 0x27:
                    self.left_hand_info = response_data
            elif frame_type == 0x08:
                self.x08 = list(response_data)
            elif frame_type == 0x09:
                self.x09 = list(response_data)
            elif frame_type == 0x0A:
                self.x0A = list(response_data)
            elif frame_type == 0x0B:
                self.x0B = list(response_data)
            elif frame_type == 0x0C:
                self.x0C = list(response_data)
            elif frame_type == 0x0D:
                self.x0D = list(response_data)
            elif frame_type == 0x22:
                #ColorMsg(msg=f"五指切向压力方向：{list(response_data)}")
                d = list(response_data)
                self.tangential_force_dir = [float(i) for i in d]
            elif frame_type == 0x23:
                #ColorMsg(msg=f"五指接近度：{list(response_data)}")
                d = list(response_data)
                self.approach_inc = [float(i) for i in d]
            elif frame_type == 0x41: # 拇指关节位置返回值
                self.x41 = list(response_data)
            elif frame_type == 0x42: # 食指关节位置返回值
                #print("食指所有关节状态",list(response_data))
                self.x42 = list(response_data)
            elif frame_type == 0x43: # 中指关节位置返回值
                self.x43 = list(response_data)
            elif frame_type == 0x44: # 无名指关节位置返回值
                
                self.x44 = list(response_data)
            elif frame_type == 0x45: # 小拇指关节位置返回值
                self.x45 = list(response_data)
            elif frame_type == 0x49: # 拇指速度返回值
                self.x49 = list(response_data)
            elif frame_type == 0x4a: # 食指速度返回值
                self.x4a = list(response_data)
            elif frame_type == 0x4b: # 中指速度返回值
                self.x4b = list(response_data)
            elif frame_type == 0x4c: # 无名指速度返回值
                self.x4c = list(response_data)
            elif frame_type == 0x4d: # 小拇指速度返回值
                self.x4d = list(response_data)
            elif frame_type == 0xc1: # 版本号
                self.xc1 = list(response_data)
            elif frame_type == 0x59:
                self.x59 = list(response_data)
            elif frame_type == 0x5a:
                self.x5a = list(response_data)
            elif frame_type == 0x5b:
                self.x5b = list(response_data)
            elif frame_type == 0x5c:
                self.x5c = list(response_data)
            elif frame_type == 0x5d:
                self.x5d = list(response_data)
            elif frame_type == 0x61:
                self.x61 = list(response_data)
            elif frame_type == 0x62:
                self.x62 = list(response_data)
            elif frame_type == 0x63:
                self.x63 = list(response_data)
            elif frame_type == 0x64:
                self.x64 = list(response_data)
            elif frame_type == 0x65:
                self.x65 = list(response_data)

    # topic映射L25
    def joint_map(self, pose):
        # L25 CAN数据默认接收30个数据
        l25_pose = [0.0] * 30  # 初始化l25_pose为30个0.0

        # 映射表，通过字典简化映射关系
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  4: None,  5: 20,
            6: None, 7: 6,   8: 1,   9: 16,  10: None, 11: 21,
            12: None, 13: None, 14: 2,  15: 17, 16: None, 17: 22,
            18: None, 19: 8,  20: 3,   21: 18, 22: None, 23: 23,
            24: None, 25: 9,  26: 4,   27: 19, 28: None, 29: 24
        }

        # 遍历映射字典，进行值的映射
        for l25_idx, pose_idx in mapping.items():
            if pose_idx is not None:
                l25_pose[l25_idx] = pose[pose_idx]

        return l25_pose

    # 将L25的状态值转换为CMD格式的状态值
    def state_to_cmd(self, l25_state):
        # L25 CAN默认接收30个数据，初始化pose为25个0.0
        pose = [0.0] * 25  # 原来控制L25的指令数据为25个

        # 映射关系，字典中存储l25_state索引和pose索引之间的映射关系
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  5: 20,  7: 6,
            8: 1,   9: 16,  11: 21, 14: 2,  15: 17, 17: 22,
            19: 8,  20: 3,  21: 18, 23: 23, 25: 9,   26: 4,
            27: 19, 29: 24
        }
        # 遍历映射字典，更新pose的值
        for l25_idx, pose_idx in mapping.items():
            pose[pose_idx] = l25_state[l25_idx]
        return pose
    def action_play(self):
        self.send_command(0xA0,[])
    # 获取所有关节数据
    def get_current_status(self, j=''):
        self.send_command(FrameProperty.THUMB_POS, j)
        time.sleep(0.005)
        self.send_command(FrameProperty.INDEX_POS,j)
        time.sleep(0.005)
        self.send_command(FrameProperty.MIDDLE_POS,j)
        time.sleep(0.005)
        self.send_command(FrameProperty.RING_POS,j)
        time.sleep(0.005)
        self.send_command(FrameProperty.LITTLE_POS, j)
        time.sleep(0.005)
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l25_state = self.state_to_cmd(l25_state=state)
            return l25_state
    def get_current_state_topic(self):
        self.send_command(0x01,[])
        time.sleep(0.001)
        self.send_command(0x02,[])
        time.sleep(0.001)
        self.send_command(0x03,[])
        time.sleep(0.001)
        self.send_command(0x04,[])
        time.sleep(0.001)
        self.send_command(0x06,[])
        time.sleep(0.001)
        state = self.x03+self.x02+self.x01+self.x04+self.x06
        return state
    def get_speed(self,j=''):
        self.send_command(FrameProperty.THUMB_SPEED, j) # 大拇指速度
        time.sleep(0.01)
        self.send_command(FrameProperty.INDEX_SPEED, j) # 食指速度
        time.sleep(0.01)
        self.send_command(FrameProperty.MIDDLE_SPEED, j) # 中指速度
        time.sleep(0.01)
        self.send_command(FrameProperty.RING_SPEED, j) # 无名指速度
        time.sleep(0.01)
        self.send_command(FrameProperty.LITTLE_SPEED, j) # 小拇指速度
        time.sleep(0.01)
        speed = self.x49+ self.x4a+ self.x4b+ self.x4c+ self.x4d
        if len(speed) == 30:
            l25_speed = self.state_to_cmd(l25_state=speed)
            return l25_speed
    
    def get_finger_torque(self):
        return self.finger_torque
    # def get_current(self):
    #     return self.x06
    def get_fault(self):
        self.get_thumbn_fault()
        time.sleep(0.001)
        self.get_index_fault()
        time.sleep(0.001)
        self.get_middle_fault()
        time.sleep(0.001)
        self.get_ring_fault()
        time.sleep(0.001)
        self.get_little_fault()
        time.sleep(0.001)
        return [self.x59]+[self.x5a]+[self.x5b]+[self.x5c]+[self.x5d]
    def get_threshold(self):
        self.get_thumb_threshold()
        self.get_index_threshold()
        self.get_middle_threshold()
        self.get_ring_threshold()
        self.get_little_threshold()
        return [self.x61]+[self.x62]+[self.x63]+[self.x64]+[self.x65]
    def get_version(self):
        if self.xc1 == []:
            self.send_command(FrameProperty.HAND_HARDWARE_VERSION,[])
        return self.xc1
    def close_can_interface(self):
        if self.bus:
            self.bus.shutdown()  # 关闭 CAN 总线
    
    '''
    这个方法只用于展示数据关系映射，使用的话最好使用上面的方法
    '''
    def joint_map_2(self, pose):
        l25_pose = [0.0]*30 #L25 CAN默认接收30个数据 pose控制L25发送的指令数据默认25个，这里进行映射
        '''
        需要进行映射
        # L25 CAN数据格式
        #["拇指横摆0-10", "拇指侧摆1-5", "拇指根部2-0", "拇指中部3-15", "预留4-", "拇指指尖5-20", "预留6-", "食指侧摆7-6", "食指根部8-1", "食指中部9-16", "预留10-", "食指指尖11-21", "预留12-", "预留13-", "中指根部14-2", "中指中部15-17", "预留16-", "中指指尖17-22", "预留18-", "无名指侧摆19-8", "无名指根部20-3", "无名指中部21-18", "预留22-", "无名指指尖23-23", "预留24-", "小指侧摆25-9", "小指根部26-4", "小指中部27-19", "预留28-", "小指指尖29-24"]
        # CMD 接收到的数据格式
        #["拇指根部0", "食指根部1", "中指根部2", "无名指根部3","小指根部4","拇指侧摆5","食指侧摆6","中指侧摆","无名指侧摆8","小指侧摆9","拇指横摆10","预留","预留","预留","预留","拇指中部15","食指中部16","中指中部17","无名指中部18","小指中部19","拇指指尖20","食指指尖21","中指指尖22","无名指指尖23","小指指尖24"]
        '''
        l25_pose[0] = pose[10]
        l25_pose[1] = pose[5]
        l25_pose[2] = pose[0]
        l25_pose[3] = pose[15]
        l25_pose[4] = 0.0
        l25_pose[5] = pose[20]
        l25_pose[6] = 0.0
        l25_pose[7] = pose[6]
        l25_pose[8] = pose[1]
        l25_pose[9] = pose[16]
        l25_pose[10] = 0.0
        l25_pose[11] = pose[21]
        l25_pose[12] = 0.0
        l25_pose[13] = 0.0
        l25_pose[14] = pose[2]
        l25_pose[15] = pose[17]
        l25_pose[16] = 0.0
        l25_pose[17] = pose[22]
        l25_pose[18] = 0.0
        l25_pose[19] = pose[8]
        l25_pose[20] = pose[3]
        l25_pose[21] = pose[18]
        l25_pose[22] = 0.0
        l25_pose[23] = pose[23]
        l25_pose[24] = 0.0
        l25_pose[25] = pose[9]
        l25_pose[26] = pose[4]
        l25_pose[27] = pose[19]
        l25_pose[28] = 0.0
        l25_pose[29] = pose[24]
        return l25_pose