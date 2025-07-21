////这个是五个电机的版本，五个电机都加上有问题的版本


#include <unistd.h>
#include <stdio.h>
#include <cmath> // For M_PI, fmod, fabs
#include <algorithm> // For clamp
#include <limits> // For numeric_limits

#include <iostream>
#include <vector>
#include <memory>
#include <optional>

// Dynamixel SDK includes
#include "dynamixel_sdk.h"

// Pinocchio includes
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp" // Just for namespace reference, actual model is built below

// Control table addresses for XL330-M288-T (Protocol 2.0)
#define ADDR_TORQUE_ENABLE          64 // 1 Byte
#define ADDR_OPERATING_MODE         11 // 1 Byte
#define ADDR_GOAL_CURRENT          102 // 2 Bytes (signed)
#define ADDR_PRESENT_POSITION      132 // 4 Bytes
#define ADDR_PRESENT_CURRENT       126 // 2 Bytes (signed)
#define ADDR_PRESENT_TEMPERATURE   146 // 1 Byte
// #define ADDR_MOVING_STATUS         123 // 1 Byte (Optional: check if moving)

// Data Byte Length
#define LEN_TORQUE_ENABLE           1
#define LEN_OPERATING_MODE          1
#define LEN_GOAL_CURRENT            2
#define LEN_PRESENT_POSITION        4
#define LEN_PRESENT_CURRENT         2
#define LEN_PRESENT_TEMPERATURE     1

// Protocol version
#define PROTOCOL_VERSION            2.0

// Default setting
#define DXL1_ID                     1     //设置电流，负下，正上
#define DXL2_ID                     2     //设置电流，负上，正下
#define DXL3_ID                     3
#define DXL4_ID                     4     //设置电流，根据3的位置进行调整
#define DXL5_ID                     5


#define BAUDRATE                    3000000 // Or higher, e.g., 4000000, check your adapter
#define DEVICENAME                  "/dev/ttyUSB0" // Or "/dev/ttyACM0" or Windows COM port e.g. "COM3"

#define TORQUE_ENABLE               1
#define TORQUE_DISABLE              0
#define OPERATING_MODE_CURRENT      0 // Current Control Mode

// Conversion factor: DXL Position units to Radians
// XL330-M288-T has 4096 steps per revolution (0-4095)
#define DXL_POSITION_TO_RADIANS     (2.0 * M_PI / 4096.0)

// Conversion factor: Goal Current DXL units to Amps.
// According to XL330-M288-T manual, Goal Current unit is ~3.36mA/unit.
// So 1 unit = 0.00336 A.
// #define DXL_GOAL_CURRENT_TO_AMPS    0.00336 // Not directly needed for this code

// Conversion factor: Torque (Nm) to Goal Current DXL units
// Based on Rated Torque (0.2 Nm) and Rated Current (0.5 A).
// Theoretical Torque constant approx = 0.2 Nm / 0.5 A = 0.4 Nm/A.
// Required Amps = Torque (Nm) / (Torque Constant)
// Goal Current DXL = Required Amps / DXL_GOAL_CURRENT_TO_AMPS
// Goal Current DXL = Torque (Nm) / (Torque Constant * DXL_GOAL_CURRENT_TO_AMPS)
// Goal Current DXL approx = Torque (Nm) / (0.4 * 0.00336) = Torque (Nm) / 0.001344
// Goal Current DXL approx = Torque (Nm) * 744.04
// Use a conversion factor. This might need tuning. Let's use 745 for now.
#define TORQUE_TO_GOAL_CURRENT_DXL_UNITS 2825

// Max/Min Goal Current values for XL330 (signed 11 bits)
// Range is -2047 to 2047
#define GOAL_CURRENT_MAX_DXL        1700
#define GOAL_CURRENT_MIN_DXL       -1700

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <condition_variable>
// void getid()
// {
//     std::cout << "Attempting to read present positions..." << std::endl;
//     // 7. 发送同步读取指令包
//     int dxl_comm_result = groupSyncRead.txRxPacket();
//     //printCommStatus(packetHandler, dxl_comm_result);
//     if (dxl_comm_result != COMM_SUCCESS) {
//         // // 如果通信失败，后续的检查和数据获取可能无意义
//         // groupSyncRead.clearParam(); // 清理参数
//         // portHandler->closePort();
//         // delete packetHandler;
//         // delete portHandler;
//         // return -1;
//         std::cout<<"sadsa"<<std::endl;
//     }
//     // 8. 检查每个电机是否成功返回数据并获取数据
//     std::vector<uint32_t> present_positions(6);
//     bool all_motors_responded = true;
//     for (uint8_t id=0;id<6;++id) {
//         // // 8a. 检查电机是否有硬件错误 (可选，但推荐)
//         // uint8_t dxl_error = 0;
//         // dxl_comm_result = groupSyncRead.getError(id, &dxl_error); // 注意：此处的dxl_comm_result只检查内部状态，不是总线通信
//         // if (dxl_comm_result != COMM_SUCCESS) {
//         //      std::cerr << "Failed to get error status for DXL ID " << (int)id << " (internal SDK issue)." << std::endl;
//         // }
//         // if (dxl_error != 0) {
//         //     printRxPacketError(packetHandler, dxl_error);
//         //     std::cerr << "Motor ID " << (int)id << " reported hardware error." << std::endl;
//         //     all_motors_responded = false; // 标记一下，即使数据可能存在，也应注意错误
//         // }
//         // 8b. 检查数据是否可用
//         bool is_available = groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
//         if (is_available) {
//             present_positions[id] = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
//             std::cout << "[ID:" << (int)id << "] Present Position: " << present_positions[id] << std::endl;
//         } else {
//             std::cerr << "No data available from DXL ID: " << (int)id << std::endl;
//             all_motors_responded = false;
//         }
//     }
//     if (!all_motors_responded) {
//         std::cout << "Warning: Not all motors responded or reported errors." << std::endl;
//     }
//     // 9. 清理GroupSyncRead的参数 (如果要在循环中重复使用或更改参数，这很重要)
//     groupSyncRead.clearParam();
// }


// 共享数据和同步原语
std::vector<double> g_latest_motor_positions(6);
std::mutex g_positions_mutex;
std::condition_variable g_cv_positions_ready;
bool g_new_data_available = false;
bool g_terminate_threads = false; // 用于优雅地关闭线程

// ROS发布线程函数
void rosPublishThread(ros::NodeHandle& nh) {
    try {

        ////////////////////////////////
        // for (uint8_t id = 0; id < 6; ++id) {
        //     dxl_ids.push_back(id);
        // }
        // // 6. 将要读取的电机ID添加到GroupSyncRead的参数列表中
        // //    这一步告诉SDK我们要从哪些电机读取数据
        // bool addparam_result = false;
        // for (uint8_t id : dxl_ids) {
        //     addparam_result = groupSyncRead.addParam(id);
        //     if (!addparam_result) {
        //         std::cerr << "Failed to add param for DXL ID: " << (int)id << std::endl;
        //         // 通常这里失败意味着内部参数列表已满或ID无效，但对于少量电机不太可能
        //         portHandler->closePort();
        //         delete packetHandler;
        //         delete portHandler;
        //         return -1;
        //     }
        // }
        //////////////////////////////////////////


        // 创建发布者
        ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/motion_target/target_joint_state_arm", 10);
  
        
        // 设置发布频率
        ros::Rate rate(200);  // 50Hz
        
        // 准备消息
        sensor_msgs::JointState joint_state_msg;

        
        // 配置消息结构
        joint_state_msg.position.resize(6);
        //joint_state_msg.velocity.resize(num_motors);  // 虽然我们没有速度信息，但保留该字段
        for(size_t i = 0; i < joint_state_msg.position.size(); ++i)
        {
          joint_state_msg.position[i] = 0.0;
        }
        // 主循环
        ROS_INFO("ROS发布线程已启动");
        while (ros::ok() && !g_terminate_threads) {

                std::unique_lock<std::mutex> lock(g_positions_mutex);
                g_cv_positions_ready.wait_for(lock, std::chrono::milliseconds(100), []{ return g_new_data_available || g_terminate_threads; });
                if (g_terminate_threads) {
                    lock.unlock(); // 必须先解锁再break，或者让unique_lock自动析构
                    break;
                }
                if (g_new_data_available) {
                    // 复制数据到ROS消息
                    for (int i = 0; i < 6; ++i) {
                        // 注意: Dynamixel原始值是 0-4095 (或类似)。你可能需要转换为弧度
                        // 假设 XL330: 0-4095 -> 0-360 degrees
                        // joint_state_msg.position[i] = (static_cast<double>(g_latest_motor_positions[i]) / 4095.0) * (2.0 * M_PI);
                        // 暂时直接用原始值，你需要根据实际情况转换
                        joint_state_msg.position[i] = static_cast<double>(g_latest_motor_positions[i]);
                    }

                    g_new_data_available = false; // 标记数据已被消费
                    lock.unlock(); // 尽快释放锁，在发布之前
                    joint_state_msg.header.stamp = ros::Time::now();
                    
                    // for(auto we:g_latest_motor_positions)
                    // {
                    //     std::cout<<"-------we-------"<<we<<std::endl;
                    // }
                    //std::cout<<"2222222222222222222222222222222222====="<<g_latest_motor_positions[2]<<std::endl;
                    joint_state_msg.position[1]+=1.896;
                    joint_state_msg.position[2]-=1.8;
                    //joint_state_msg.position[2]=-joint_state_msg.position[2];
                    //std::cout<<"2222222222222222222222222222222222====="<<joint_state_msg.position[2]<<std::endl;
                    for(int i=3;i<6;i++)
                    {
                        joint_state_msg.position[i]=0;
                    }
                    // for(auto we:joint_state_msg.position)
                    // {
                    //     std::cout<<"-------pppppppppppppppppppppppppppppppppppppppppppppp    "<<we<<std::endl;
                    // }

                     // 发布消息
                    joint_state_pub.publish(joint_state_msg);
                        // 填充消息
                }else{
                    lock.unlock(); // 如果是超时唤醒且没有新数据
                }
            //在这里将读取到电机信息进行发送
            /////////////////////////////////////////-------------------


            //     // 7. 发送同步读取指令包
            // int dxl_comm_result = gsr_ptr->txRxPacket();
            // //printCommStatus(packetHandler, dxl_comm_result);
            // if (dxl_comm_result != COMM_SUCCESS) {
            //     // // 如果通信失败，后续的检查和数据获取可能无意义
            //     // groupSyncRead.clearParam(); // 清理参数
            //     // portHandler->closePort();
            //     // delete packetHandler;
            //     // delete portHandler;
            //     // return -1;
            //     std::cout<<"sadsa"<<std::endl;
            // }
            // // 8. 检查每个电机是否成功返回数据并获取数据
            // std::vector<uint32_t> present_positions(6);
            // //bool all_motors_responded = true;
            // for (uint8_t id=0;id<6;++id) {
            //     // // 8a. 检查电机是否有硬件错误 (可选，但推荐)
            //     // uint8_t dxl_error = 0;
            //     // dxl_comm_result = groupSyncRead.getError(id, &dxl_error); // 注意：此处的dxl_comm_result只检查内部状态，不是总线通信
            //     // if (dxl_comm_result != COMM_SUCCESS) {
            //     //      std::cerr << "Failed to get error status for DXL ID " << (int)id << " (internal SDK issue)." << std::endl;
            //     // }
            //     // if (dxl_error != 0) {
            //     //     printRxPacketError(packetHandler, dxl_error);
            //     //     std::cerr << "Motor ID " << (int)id << " reported hardware error." << std::endl;
            //     //     all_motors_responded = false; // 标记一下，即使数据可能存在，也应注意错误
            //     // }
            //     // 8b. 检查数据是否可用
            //     bool is_available = gsr_ptr->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            //     if (is_available) {
            //         present_positions[id] = gsr_ptr->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            //         std::cout << "[ID:" << (int)id << "] Present Position: " << present_positions[id] << std::endl;
            //     } else {
            //         std::cout << "[ID:" << (int)id << "] Present Position: " << present_positions[id] << std::endl;
            //         std::cerr << "No data available from DXL ID: " << (int)id << std::endl;
            //         //all_motors_responded = false;
            //     }
            // }
            // // if (!all_motors_responded) {
            // //     std::cout << "Warning: Not all motors responded or reported errors." << std::endl;
            // // }
            // // 9. 清理GroupSyncRead的参数 (如果要在循环中重复使用或更改参数，这很重要)
            // gsr_ptr->clearParam();


            //////////////////-------------------------------------------------------------

           
            
            // 处理回调并遵循设定的频率
            ros::spinOnce();
            rate.sleep();
        }
        for(int i=0;i<6;i++)
        {
            joint_state_msg.position[i]=0.0;
        }
        joint_state_pub.publish(joint_state_msg);
    }
    catch (const std::exception& e) {
        ROS_ERROR("ROS发布线程异常: %s", e.what());
    }
    catch (...) {
        ROS_ERROR("ROS发布线程发生未知异常");
    }
    
    ROS_INFO("ROS发布线程已退出");
}







namespace { // Anonymous namespace for helper functions and model building

template <typename Scalar, int Options,
  template <typename, int> class JointCollectionTpl>
void BuildModel(pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>* model) {
  using namespace pinocchio;



  constexpr double link_length1 = 0.20;
  constexpr double link_length2 = 0.088;
  constexpr double link_length3 = 0.216;
  constexpr double link_length4 = 0.056;
  constexpr double link_length5 = 0.04;
  SE3 Tlink1 (SE3::Matrix3::Identity(), SE3::Vector3(0, 0, 0)); 
  SE3 Tlink2 (SE3::Matrix3::Identity(), SE3::Vector3(0, 0, link_length1));
  SE3 Tlink3 (SE3::Matrix3::Identity(), SE3::Vector3(0, 0, link_length2));    /// Tlink 变换描述了每个新关节的参考系相对于其父关节参考系的位置和姿态
  SE3 Tlink4 (SE3::Matrix3::Identity(), SE3::Vector3(link_length3, 0,0 ));
  SE3 Tlink5 (SE3::Matrix3::Identity(), SE3::Vector3(link_length4, 0,0 ));
  SE3 Tlink6 (SE3::Matrix3::Identity(), SE3::Vector3(link_length5, 0,0 ));
  constexpr double kFudge = 0.15;
  Inertia Ilink1(kFudge*0.15, Tlink2.translation(), Inertia::Matrix3::Identity() * 0.001);
  Inertia Ilink2(kFudge*0.05, Tlink3.translation(), Inertia::Matrix3::Identity() * 0.001);///Ilink是连杆
  Inertia Ilink3(kFudge*0.15, Tlink4.translation(), Inertia::Matrix3::Identity() * 0.001);
  Inertia Ilink4(kFudge*0.05, Tlink5.translation(), Inertia::Matrix3::Identity() * 0.001);
  Inertia Ilink5(kFudge*0.03, Tlink6.translation(), Inertia::Matrix3::Identity() * 0.001);

  //Inertia Ilink6(kFudge*0.03, Tlink6.translation(), Inertia::Matrix3::Identity() * 0.001);
  // Joint limits (placeholders)
  using CV = typename JointCollectionTpl<Scalar, Options>::JointModelRY::ConfigVector_t;
  using TV = typename JointCollectionTpl<Scalar, Options>::JointModelRY::TangentVector_t;
  CV qmin = CV::Constant(-M_PI);
  CV qmax = CV::Constant(M_PI);
  TV vmax = TV::Constant(M_PI * 4);
  TV taumax = TV::Constant(0.5); // XL330 Rated Torque

  Model::JointIndex idx = 0; // Index of the parent joint

  // Add first joint: Revolute around Y axis. Offset by Tlink from world origin.
  idx = model->addJoint(idx, typename JointCollectionTpl<Scalar, Options>::JointModelRY(), Tlink1,
                        "link1_joint", taumax, vmax, qmin, qmax);
  model->appendBodyToJoint(idx, Ilink1); // Attach link 1 inertia
  model->addJointFrame(idx);
  model->addBodyFrame("link1_body", idx);

  // Add second joint: Revolute around Y axis. Offset by Tlink from previous joint frame.
  idx = model->addJoint(idx, typename JointCollectionTpl<Scalar, Options>::JointModelRY(), Tlink2,
                        "link2_joint", taumax, vmax, qmin, qmax);
  model->appendBodyToJoint(idx, Ilink2); // Attach link 2 inertia
  model->addJointFrame(idx);
  model->addBodyFrame("link2_body", idx);

    //-----------------------
    idx = model->addJoint(idx, typename JointCollectionTpl<Scalar, Options>::JointModelRX(), Tlink3,
    "link3_joint", taumax, vmax, qmin, qmax);
    model->appendBodyToJoint(idx, Ilink3); // Attach link 2 inertia
    model->addJointFrame(idx);
    model->addBodyFrame("link3_body", idx);

    idx = model->addJoint(idx, typename JointCollectionTpl<Scalar, Options>::JointModelRY(), Tlink4,
    "link4_joint", taumax, vmax, qmin, qmax);
    model->appendBodyToJoint(idx, Ilink4); // Attach link 2 inertia
    model->addJointFrame(idx);
    model->addBodyFrame("link4_body", idx);

    idx = model->addJoint(idx, typename JointCollectionTpl<Scalar, Options>::JointModelRX(), Tlink5,
    "link5_joint", taumax, vmax, qmin, qmax);
    model->appendBodyToJoint(idx, Ilink5); // Attach link 2 inertia
    model->addJointFrame(idx);
    model->addBodyFrame("link5_body", idx);


  // Set gravity direction (e.g., negative Z in the base frame)
  model->gravity.linear(Eigen::Vector3d(0, 0, -9.81));

  // --- End Model Building ---
}

int32_t getWrappedDXLDelta(int32_t current_pos, int32_t zero_pos) {   //这个是将位置转换为-派到派，我的这个还需要修改
    int32_t delta = current_pos - 2048;//zero_pos;
    // Normalize delta to [-2048, 2047] range considering 4096 as a full circle
    if (delta > 2047) delta -= 2048;
    else if (delta < -2048) delta += 4096;
    return delta;
}

} // end anonymous namespace

int kbhit(void);
int main(int argc, char** argv) {

//-----
        // ROS 初始化
        ros::init(argc, argv, "dynamixel_position_reader_publisher");
        ros::NodeHandle nh;

//----

  // Dynamixel SDK objects
  std::unique_ptr<dynamixel::PortHandler> portHandler(dynamixel::PortHandler::getPortHandler(DEVICENAME));
  if (!portHandler) {
       fprintf(stderr, "Failed to get port handler for %s\n", DEVICENAME);
       return 1;
  }
  std::unique_ptr<dynamixel::PacketHandler> packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
    if (!packetHandler) {
       fprintf(stderr, "Failed to get packet handler for Protocol %.1f\n", PROTOCOL_VERSION);
       return 1;
  }


  
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWriteCurrent(
      new dynamixel::GroupSyncWrite(portHandler.get(), packetHandler.get(), ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT));

  // GroupSyncRead object for Present Position
  std::unique_ptr<dynamixel::GroupSyncRead> groupSyncReadPosition(
      new dynamixel::GroupSyncRead(portHandler.get(), packetHandler.get(), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));

    // GroupSyncRead for Status (Current, Temp - optional)
    // Let's add a SyncRead for Present Current and Temperature for monitoring
    std::unique_ptr<dynamixel::GroupSyncRead> groupSyncReadStatus(
      new dynamixel::GroupSyncRead(portHandler.get(), packetHandler.get(), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT + LEN_PRESENT_TEMPERATURE)); // Read Current + Temp in one go


  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error code
  int dxl_present_position[5] = {0, 0,0,0,0};           // Present position (DXL units)
  int32_t initial_dxl_position[5] = {0, 0,0,0,0};       // Initial position (DXL units) for zeroing
  double torque_command_nm[5] = {0.0, 0.0,0.0,0.0,0.0};       // Calculated torque commands (Nm)
  int16_t goal_current_dxl[5] = {0, 0,0,0,0};           // Goal Current commands (DXL units)
  // For sync write, we need a buffer for each motor's data
  uint8_t param_goal_current_dxl1[LEN_GOAL_CURRENT];
  uint8_t param_goal_current_dxl2[LEN_GOAL_CURRENT];
  uint8_t param_goal_current_dxl3[LEN_GOAL_CURRENT];
  uint8_t param_goal_current_dxl4[LEN_GOAL_CURRENT];
  uint8_t param_goal_current_dxl5[LEN_GOAL_CURRENT];

  uint8_t* param_buffers[5] = {param_goal_current_dxl1, param_goal_current_dxl2,param_goal_current_dxl3,param_goal_current_dxl4,param_goal_current_dxl5};


  // Pinocchio setup
  pinocchio::Model model;
  BuildModel(&model); 
  pinocchio::Data data(model); // Create data structure for computations

  // Pinocchio joint state vectors
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq); // Joint positions (radians). nq=2 for 2R.
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // Joint velocities (0)
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // Joint accelerations (0)

  // Open port
  if (!portHandler->openPort()) {
    fprintf(stderr, "Failed to open port! %s\n", DEVICENAME);
    return 1;
  }
  printf("Port opened successfully: %s\n", DEVICENAME);

  // Set baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    fprintf(stderr, "Failed to set baudrate! %d\n", BAUDRATE);
  }
  printf("Baudrate set to %d\n", BAUDRATE);

  // Enable Torque, Set Operating Mode, and Add to Sync Read
  int dxl_ids[5] = {DXL1_ID, DXL2_ID,DXL3_ID,DXL4_ID,DXL5_ID};
  bool motors_ready = true;
  for (int i = 0; i < 5; ++i) {//只对235进行更改使能
    int id = dxl_ids[i];
    uint16_t model_number;

    // Ping motor to check connectivity
    dxl_comm_result = packetHandler->ping(portHandler.get(), id, &model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Ping failed for DXL ID %d: %s\n", id, packetHandler->getTxRxResult(dxl_comm_result));
        motors_ready = false;
        break; // Exit loop if any motor fails ping
    } else if (dxl_error != 0) {
        fprintf(stderr, "Ping error for DXL ID %d: %s\n", id, packetHandler->getRxPacketError(dxl_error));
        motors_ready = false;
        break; // Exit loop on error
    } else {
        printf("DXL ID %d ping successful. Model Number: %d\n", id, model_number);
    }

    // Disable torque temporarily to change operating mode，，先让电机失能
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler.get(), id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Torque Disable failed for DXL ID %d: %s\n", id, packetHandler->getTxRxResult(dxl_comm_result));
        motors_ready = false; break;
    } else if (dxl_error != 0) {
        fprintf(stderr, "Torque Disable error for DXL ID %d: %s\n", id, packetHandler->getRxPacketError(dxl_error));
        // Continue? Or break? Let's break for safety.
        motors_ready = false; break;
    } else {
        printf("DXL ID %d torque disabled.\n", id);
    }

    // Set Operating Mode to Current Control
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler.get(), id, ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT, &dxl_error);
     if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Set Operating Mode failed for DXL ID %d: %s\n", id, packetHandler->getTxRxResult(dxl_comm_result));
        motors_ready = false; break;
    } else if (dxl_error != 0) {
        fprintf(stderr, "Set Operating Mode error for DXL ID %d: %s\n", id, packetHandler->getRxPacketError(dxl_error));
        motors_ready = false; break;
    } else {
         printf("DXL ID %d operating mode set to Current Control.\n", id);
    }

    // Enable Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler.get(), id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Torque Enable failed for DXL ID %d: %s\n", id, packetHandler->getTxRxResult(dxl_comm_result));
        motors_ready = false; break;
    } else if (dxl_error != 0) {
        fprintf(stderr, "Torque Enable error for DXL ID %d: %s\n", id, packetHandler->getRxPacketError(dxl_error));
        motors_ready = false; break;
    } else {
        printf("DXL ID %d torque enabled.\n", id);
    }

    // Add motor to Sync Read group for Present Position
    if (!groupSyncReadPosition->addParam(id)) {
        fprintf(stderr, "Failed to add DXL ID %d to SyncRead group for position.\n", id);
        motors_ready = false; break;
    }
     // Add motor to Sync Read group for Status (Current, Temp)
    if (!groupSyncReadStatus->addParam(id)) {
         fprintf(stderr, "Failed to add DXL ID %d to SyncRead group for status.\n", id);
        motors_ready = false; break;
    }
  }
//   groupSyncReadPosition->addParam(0);
//   std::thread ros_thread = std::thread(rosPublishThread,std::move(groupSyncReadPosition));
  // Exit if any motor failed to initialize
  if (!motors_ready) {
      fprintf(stderr, "Motor initialization failed. Exiting.\n");
      //goto cleanup;
  }

  // --- Read initial positions for zeroing ---
  // Assuming the arm is in the desired q=0 pose when this code starts.
  
//   printf("Reading initial positions... Ensure arm is in the desired zero pose.\n");
//   dxl_comm_result = groupSyncReadPosition->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS) {
//       fprintf(stderr, "SyncRead initial position failed: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
//       //goto cleanup;
//   }
//   for (int i = 0; i < 5; ++i) {
//       int id = dxl_ids[i];
//       if (groupSyncReadPosition->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
//           initial_dxl_position[i] = groupSyncReadPosition->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
//           printf("DXL ID %d initial position: %d\n", id, (int)initial_dxl_position[i]);
//       } else {
//           fprintf(stderr, "Failed to read initial position for DXL ID %d\n", id);
//           //goto cleanup;
//       }
//   }
//   return 0;
//   groupSyncReadPosition->clearParam(); // Clear params for next read cycle

//   // --- Main Control Loop ---
//   printf("\nStarting gravity compensation loop. Press any key to exit.\n");


////-----
std::thread ros_thread(rosPublishThread, std::ref(nh));
ROS_INFO("Main_Thread: ROS publish thread started.");
/////-----------


//   int status_count = 0;
//   constexpr int kStatusPeriod = 100; // Print status every N cycles

ros::Time start_time = ros::Time::now(); 
int loop_count = 0;                      // Counter for loop iterations within the interval
ros::Duration measurement_interval(1.0);



  while (ros::ok()) {
loop_count++;
if (ros::Time::now() - start_time >= measurement_interval) {
  double elapsed_seconds = (ros::Time::now() - start_time).toSec();
  double actual_frequency = (double)loop_count / elapsed_seconds;
  ROS_INFO("Actual loop frequency: %.2f Hz ()", actual_frequency); // Print the frequency
  // Reset for the next measurement interval
  start_time = ros::Time::now();
  loop_count = 0;
}
    std::vector<double> current_positions_buffer(6);
    //读取第0个电机
    uint32_t position1 = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler.get(), 0, ADDR_PRESENT_POSITION, &position1, &dxl_error);
        int32_t dxl_delta1=0;
        dxl_delta1 = getWrappedDXLDelta(position1, 2048);
        double qqq=0.0;
        qqq = (double)dxl_delta1* DXL_POSITION_TO_RADIANS;
    current_positions_buffer[0]=qqq;
    //std::cout<<"--------------------------0000000------"<<current_positions_buffer[0]<<std::endl;
//---


    for (int i = 0; i < 5; ++i) {
        int id = dxl_ids[i];
        uint32_t position = 0;
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler.get(), id, ADDR_PRESENT_POSITION, &position, &dxl_error);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            fprintf(stderr, "Read position failed for ID %d: %s\n", 
                    id, packetHandler->getTxRxResult(dxl_comm_result));
            continue; // 继续尝试读取下一个电机
        } else if (dxl_error != 0) {
            fprintf(stderr, "Read position error for ID %d: %s\n", 
                    id, packetHandler->getRxPacketError(dxl_error));
            continue;
        }
        

        dxl_present_position[i] = position;
        //std::cout<<i<<"位置："<<position<<std::endl;
        // --- Convert DXL Position to Pinocchio Angle (Radians) ---
        ///////////////////////////////=====
        int32_t dxl_delta=0;
        if(i!=2)
        {
            dxl_delta = getWrappedDXLDelta(dxl_present_position[i], initial_dxl_position[i]);
        }else{
            dxl_delta=dxl_present_position[i];
        }
  
        ///////////////////////////////=====
        //std::cout<<"dxl_delta:"<<dxl_delta<<std::endl;//697应该不对
        // 根据电机ID调整方向
        if(i == 1) {
            q(i) = -(double)dxl_delta * DXL_POSITION_TO_RADIANS; // 电机1是反向安装的，，，，将他们的角度方向都统一
            //std::cout<<"q(1):"<<q(i)<<std::endl;
        } else {
            q(i) = (double)dxl_delta * DXL_POSITION_TO_RADIANS;  // 正向
            //std::cout<<"q(0):"<<q(i)<<std::endl;
        }
              //--------将位置存进当前位置中，可以不用这个，先实验一下，利用dxl_delta传给共享内存
              current_positions_buffer[i+1]=q(i);
              //std::cout<<"------------------curren--"<<current_positions_buffer[i+1]<<std::endl;
              ///----
    }
    ////------------------
                    { // 大括号创建作用域，锁能尽快释放,作用域结束，锁就释放
                        std::lock_guard<std::mutex> lock(g_positions_mutex);
                        g_latest_motor_positions = current_positions_buffer; // std::vector 的赋值是深拷贝
                        g_new_data_available = true;
                    }
                    g_cv_positions_ready.notify_one(); // 通知ROS线程数据已准备好
////-----------------
    // groupSyncReadPosition->clearParam(); // Clear params for next read cycle

    // Exit loop on position read error
    if (dxl_comm_result != COMM_SUCCESS) break;



    ////////////////是否可以这样，在主线程里面读取位置，将位置的信息储存到共享内存，一个线程进行计算重力补偿，一个线程进行发送位置topic？？？？？？


    const Eigen::VectorXd& tau = pinocchio::rnea(model, data, q, v, a); // tau is in Nm,,,,,tau就是计算出来的力矩，就是抵消重力所需要的力矩
    //std::cout<<"tau:"<<tau<<std::endl;
    // --- Convert Torque (Nm) to DXL Goal Current units ---
    //double compensation_factor = 0.8;
    torque_command_nm[0] = tau(0);
    torque_command_nm[1] = -tau(1);//这个是希望电机输出的力矩
    torque_command_nm[2] = tau(2);
    torque_command_nm[3] = tau(3);
    torque_command_nm[4] = tau(4);
    //std::cout<<"torque_command_nm[0]:"<<torque_command_nm[0]<<std::endl;
    //std::cout<<"torque_command_nm[1]:"<<torque_command_nm[1]<<std::endl;
    for (int i = 0; i < 5; ++i) {
        // Convert Nm to DXL units
        double raw_goal_current = torque_command_nm[i] * TORQUE_TO_GOAL_CURRENT_DXL_UNITS;

        // Clamp to XL330's valid range [-1700, 1700]
        goal_current_dxl[i] = static_cast<int16_t>(
            std::clamp(raw_goal_current, (double)GOAL_CURRENT_MIN_DXL, (double)GOAL_CURRENT_MAX_DXL)
        );
        //std::cout<<"goal_current_dxl"<<goal_current_dxl[i]<<std::endl;
        // Prepare parameter buffer for SyncWrite
        int id = dxl_ids[i];
        //uint8_t* param_buffer = (i == 0) ? param_goal_current_dxl1 : param_goal_current_dxl2;

        uint8_t* param_buffer = param_buffers[i];


        // if(i==0)
        // {
        //     param_buffer[0] = DXL_LOBYTE(-goal_current_dxl[i]);
        //     param_buffer[1] = DXL_HIBYTE(-goal_current_dxl[i]);
        // }else{
        //     param_buffer[0] = DXL_LOBYTE(goal_current_dxl[i]);
        //     param_buffer[1] = DXL_HIBYTE(goal_current_dxl[i]);
        // }
        param_buffer[0] = DXL_LOBYTE(goal_current_dxl[i]);
        param_buffer[1] = DXL_HIBYTE(goal_current_dxl[i]);

        // Add parameter for this motor to SyncWrite group
         groupSyncWriteCurrent->addParam(id, param_buffer);
    }
    //std::cout<<"asdasdsad"<<std::endl;
    // --- Send Goal Current Commands ---
    dxl_comm_result = groupSyncWriteCurrent->txPacket();//这个应该是将目标电流进行发送的函数
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "SyncWrite Goal Current failed: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        // Handle error: exit, retry? Exit for now.
        break;
    }
     groupSyncWriteCurrent->clearParam(); // Clear params for next write cycle
    

    // --- Read Status (Optional: Current, Temp) ---
    // int16_t present_current_dxl[5] = {0, 0,0,0,0};
    // uint8_t present_temp[5] = {0, 0,0,0,0};
        // --- 读取状态 (Current, Temp) 也改为单独读取 ---
            //     for (int i = 0; i < 5; ++i) {
            //         int id = dxl_ids[i];
            //         int16_t current = 0;
            //         uint8_t temp = 0;
                    
            //         // 读取电流
            //         dxl_comm_result = packetHandler->read2ByteTxRx(
            //             portHandler.get(), id, ADDR_PRESENT_CURRENT, (uint16_t*)&current, &dxl_error);
            //         if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
            //             present_current_dxl[i] = current;
            //            // std::cout<<"电流:"<<current<<std::endl;
            //         }
                    
            //         // 读取温度
            //         dxl_comm_result = packetHandler->read1ByteTxRx(
            //             portHandler.get(), id, ADDR_PRESENT_TEMPERATURE, &temp, &dxl_error);
            //         if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
            //             present_temp[i] = temp;
            //         }
            //     }
            //     //----------------------------------------
            // groupSyncReadStatus->clearParam(); // Clear params for next read cycle


    // --- Print Status ---
    // status_count++;
    // if (status_count >= kStatusPeriod) {
    //   // Convert DXL Current to Amps (optional)
    //   // double present_current_amps[2];
    //   // present_current_amps[0] = present_current_dxl[0] * DXL_GOAL_CURRENT_TO_AMPS;
    //   // present_current_amps[1] = present_current_dxl[1] * DXL_GOAL_CURRENT_TO_AMPS;

    //   printf("Pos DXL: %4d/%4d | Pos Rad: %+6.3f/%+6.3f | Torque Calc (Nm): %+6.3f/%+6.3f | Goal Current DXL: %+-4d/%+-4d | Temp (°C): %3d/%3d  \r",
    //          dxl_present_position[0], dxl_present_position[1],
    //          q(0), q(1),
    //          torque_command_nm[0], torque_command_nm[1],
    //          goal_current_dxl[0], goal_current_dxl[1],
    //          present_temp[0], present_temp[1]);
    //   fflush(stdout);

    //   status_count = 0;
    // }

    // --- Loop Delay ---
    // Adjust delay based on desired control frequency and communication speed.
    // 10 us is very fast. Start with a larger delay like 1000 us (1ms) or more
    // and reduce if communication/processing can keep up.
    // Minimum delay is dictated by communication time for Tx/Rx packets.
    //::usleep(1000); // 1000 microseconds = 1 millisecond

  } // end while(true)

    // --- Cleanup: Disable Torque on Exit ---
cleanup:
  printf("\nDisabling torque on motors...\n");
  for (int i = 0; i < 5; ++i) {
    int id = dxl_ids[i];
    uint8_t torque_error = 0;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler.get(), id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &torque_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Torque Disable failed for DXL ID %d: %s\n", id, packetHandler->getTxRxResult(dxl_comm_result));
    } else if (torque_error != 0) {
        fprintf(stderr, "Torque Disable error for DXL ID %d: %s\n", id, packetHandler->getRxPacketError(torque_error));
    } else {
        printf("DXL ID %d torque disabled.\n", id);
    }
  }


    ////-----这个是线程销毁机制
    {
        std::lock_guard<std::mutex> lock(g_positions_mutex);
        g_terminate_threads = true;
    }
    g_cv_positions_ready.notify_all(); // 唤醒所有等待的线程（这里只有一个）
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    ROS_INFO("Main_Thread: ROS publish thread joined.");

///------------


  // Close port
  portHandler->closePort();
  printf("Port closed.\n");

  return 0;
}

