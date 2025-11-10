#include <CustomInterface.h>
#include <iostream>  // 用于std::cout
#include <thread>   // 必需包含 <thread>
#include <chrono>   // 必需包含 <chrono>
#include <unistd.h>  // 用于sleep
#include <sys/timerfd.h>  // 用于timerfd
#include <cstring>   // 用于memset (如果需要)
#include <cstdlib>
#include <tuple>
#include <iomanip>
#include <cmath>     // 用于std::atan2等数学函数
#include "socketcan_driver.hpp"  // SocketCan驱动头文件
#include "yesense_driver.hpp"    // Yesense IMU驱动头文件
#include "socketcan/se_motor.hpp"

#define M_PI 3.14159265358979323846

int motor_dir[12] = {-1, 1, 1,    // 1,2,3 右前腿
                     -1,-1,-1,    // 4,5,6 左前腿
                      1, 1, 1,    // 7,8,9 右后腿    
                      1,-1,-1};   // A,B,C 左后腿

float motor_zeroOffset[12] = {-0.78, -1.57, -0.8,   // 1,2,3 右前腿
                               0.78, -1.57, -0.8,   // 4,5,6 左前腿
                              -0.78, -1.57, -0.8,   // 7,8,9 右后腿
                               0.78, -1.57, -0.8 }; // A,B,C 左后腿


// 辅助函数：从四元数计算RPY (roll, pitch, yaw)
std::tuple<double, double, double> quat_to_rpy(double w, double x, double y, double z) 
{
    // 计算 roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // 计算 pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    double pitch;
    if (std::abs(sinp) >= 1) 
    {
        pitch = std::copysign(M_PI / 2, sinp);  // 使用 +/- pi/2 如果超出范围
    } 
    else 
    {
        pitch = std::asin(sinp);
    }
    
    // 计算 yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return {roll, pitch, yaw};
}

CustomInterface::CustomInterface(const double &loop_rate)
    : _drivers{
        SocketCanDriver("can0", [this](const can_frame& frame){ this->handleCanFrame(0, frame); }),
        SocketCanDriver("can1", [this](const can_frame& frame){ this->handleCanFrame(1, frame); }),
        SocketCanDriver("can2", [this](const can_frame& frame){ this->handleCanFrame(2, frame); }),
        SocketCanDriver("can3", [this](const can_frame& frame){ this->handleCanFrame(3, frame); })
    },
    _imu("/dev/yesenseIMU", 460800)
{
    running_ = true;
    all_thread_done_ = false;
    mode_state = false;
    if (loop_rate > 0) 
    {
        dt_ = 1.0 / loop_rate;
    } 
    else 
    {
        std::cout << "Loop rate should be more than zero! Set to default 500Hz." << std::endl;
        dt_ = 1.0 / 500;
    }

    // 初始化IMU,设置参数
    _imu.setCovariances(0.01, 0.01, 0.01);
    _imu.setFrameID("imu_link");

    // 打开IMU串口,打不开就嘎了
    if (!_imu.open()) 
    {
        std::cerr << "Failed to open IMU on port /dev/yesenseIMU" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "======= Yesense has already been loaded! ======= " << std::endl;

    // 启动所有SocketCan驱动，启动不了就嘎了
    for (auto& driver : _drivers) 
    {
        if (!driver.start()) 
        {
            std::cerr << "Failed to start SocketCanDriver." << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    std::cout << "====== SocketCAN has already been loaded! ====== " << std::endl;

    // 所有电机使能
    can_frame enableCANFrame;
    enableCANFrame.can_dlc = 8;
    uint8_t enableData[8] = {0x80,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFD};
    memcpy(enableCANFrame.data, enableData, sizeof(enableData));
    for (int i = 0; i < 12; i++)
    {
        enableCANFrame.can_id = i+1;
        int canDeviceNum = i/3;
        // std::cout<< "Sending Through Device:    "<<canDeviceNum<<std::endl;

        if (!_drivers[canDeviceNum].send(enableCANFrame)) 
        {
            std::cerr << "Send failed for motor " << enableCANFrame.can_id << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // 休眠500微秒

    }
    std::cout << "====== All Motors has been enabled! ====== " << std::endl;

    // 假设启动后模式激活（如果需要特定条件，可以修改）
    mode_state = true;

    // 只启动用户控制线程
    _user_code_ControlThread = std::thread(&CustomInterface::Control, this);
}

void CustomInterface::Control()
{
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    int seconds = (int) dt_;
    int nanoseconds = (int) (1e9 * std::fmod(dt_, 1.f));

    itimerspec timerSpec;
    memset(&timerSpec, 0, sizeof(timerSpec));  // 清零
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;

    timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    unsigned long long missed = 0;
    while (running_) 
    {
        if (!mode_state) 
        {
            sleep(1);
            std::cout << "Motor control mode has not been activated successfully" << std::endl;
            continue;
        }

        // 调用自定义接收函数更新数据
        receiveMotorData();       // 更新q, qd, tau (如果需要同步；否则callback已处理)
        receiveRobotState();      // 更新omega, rpy, acc, quat

        UserCode();  // 用户自定义代码，基于robot_data设置motor_cmd

        motor_cmd_send();  // 准备并发送motor_cmd
        
        int m = read(timerFd, &missed, sizeof(missed));
        (void) m;
    }
    close(timerFd);  // 关闭timerfd
}

void CustomInterface::Spin()
{
    while (!all_thread_done_) 
    {
        sleep(1.0);
    }
    printf("~ Exit ~\n");
}

void CustomInterface::Stop()
{
    running_ = false;
    _user_code_ControlThread.join();

    // 停止IMU和SocketCan驱动
    _imu.close();
    for (auto& driver : _drivers) 
    {
        driver.stop();
    }

    all_thread_done_ = true;
    std::cout<< "See you!" <<std::endl;
}

// Over
void CustomInterface::motor_cmd_send()
{
    for (int i = 0; i < 12; i++) 
    {
        // 从模型 → 电机
        _motor_ctrl.q_des[i] = (motor_cmd.q_des[i] - motor_zeroOffset[i]) * motor_dir[i];
        _motor_ctrl.qd_des[i] = motor_cmd.qd_des[i] * motor_dir[i];  // 速度通常只乘 dir，不需偏移（假设零速无偏）
        _motor_ctrl.tau_des[i] = motor_cmd.tau_des[i] * motor_dir[i];  // 扭矩类似
        _motor_ctrl.kp_des[i] = motor_cmd.kp_des[i];  // KP/KD 无需调整
        _motor_ctrl.kd_des[i] = motor_cmd.kd_des[i];

        struct can_frame ctrlFrame = {};  // 初始化清零
        MotorCmd ctrlCmd = {
            .q_des = static_cast<float>(_motor_ctrl.q_des[i]),
            .qd_des = static_cast<float>(_motor_ctrl.qd_des[i]),
            .kp_des = static_cast<float>(_motor_ctrl.kp_des[i]),
            .kd_des = static_cast<float>(_motor_ctrl.kd_des[i]),
            .tau_des = static_cast<float>(_motor_ctrl.tau_des[i])
        };

        uint32_t motor_id = i+1;  // 示例 CAN ID，调整为实际
        int motor_model = i%3;  // 简化：i 从0开始，直接 %3 得0,1,2
        SE_packCommand(ctrlFrame, motor_id, motor_model, ctrlCmd);

        int canDeviceNum = i / 3;
        // std::cout<< "Sending Through Device:    "<<canDeviceNum<<std::endl;

        if (!_drivers[canDeviceNum].send(ctrlFrame)) 
        {
            std::cerr << "Send failed for motor " << motor_id << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // 休眠500微秒
    }
}

// 自定义函数：接收电机数据（替换原LCM处理）
void CustomInterface::receiveMotorData()
{
    // TODO: 如果需要同步读取，可以在这里添加逻辑
    // 但由于SocketCan使用异步callback更新robot_data，此函数可以为空
}

// 自定义函数：接收机器人状态/IMU数据（替换原LCM处理）
void CustomInterface::receiveRobotState()
{
    if (_imu.readAndParse()) 
    {
        IMUData data = _imu.getIMU();
        // 更新quat (假设orientation_x/y/z/w 是quat的x/y/z/w)
        robot_data.quat[0] = data.orientation_x;
        robot_data.quat[1] = data.orientation_y;
        robot_data.quat[2] = data.orientation_z;
        robot_data.quat[3] = data.orientation_w;  // 假设w是最后一个

        // 更新omega (角速度)
        robot_data.omega[0] = data.angular_velocity_x;
        robot_data.omega[1] = data.angular_velocity_y;
        robot_data.omega[2] = data.angular_velocity_z;

        // 更新acc (线性加速度)
        robot_data.acc[0] = data.linear_acceleration_x;
        robot_data.acc[1] = data.linear_acceleration_y;
        robot_data.acc[2] = data.linear_acceleration_z;

        // 计算rpy from quat (w, x, y, z)
        auto [roll, pitch, yaw] = quat_to_rpy(robot_data.quat[3], robot_data.quat[0], robot_data.quat[1], robot_data.quat[2]);
        robot_data.rpy[0] = roll;
        robot_data.rpy[1] = pitch;
        robot_data.rpy[2] = yaw;
    }
}

// CAN帧接收回调处理函数
void CustomInterface::handleCanFrame(int driver_idx, const can_frame& frame)
{
    if(mode_state)
    {
        // std::cout << "Received CAN frame from driver " << driver_idx << ": ID=0x" << std::hex << frame.can_id
        //       << ", DLC=" << std::dec << static_cast<int>(frame.can_dlc) << std::endl;
        // 更新robot_data

        uint32_t motor_id;
        PVTData pvtStatus;

        SE_unpackStatus(frame, motor_id, pvtStatus);
        robot_data.q[motor_id-1]    = pvtStatus.position * motor_dir[motor_id-1] + motor_zeroOffset[motor_id-1];
        // robot_data.q[motor_id-1]    = pvtStatus.position ;
        robot_data.qd[motor_id-1]   = pvtStatus.velocity * motor_dir[motor_id-1];
        robot_data.tau[motor_id-1]  = pvtStatus.torque   * motor_dir[motor_id-1];
    }
}