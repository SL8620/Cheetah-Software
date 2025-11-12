#ifndef CUSTOM_INTERFACE_H
#define CUSTOM_INTERFACE_H

#include <thread>
#include <array>
#include <cstdint>  // 用于uint8_t等
#include "socketcan/socketcan_driver.hpp"
#include "yesense/yesense_driver.hpp"
#include <mutex>  // 添加mutex头文件

// 前向声明（如果socketcan_driver.hpp未包含）
struct can_frame;

// 结构体定义（基于原代码推断）
struct RobotData {
    double q[12];
    double qd[12];
    double tau[12];
    double omega[3];
    double rpy[3];
    double acc[3];
    double quat[4];
    int err_flag;
    int ctrl_topic_interval;

    // 1. 添加默认构造函数
    RobotData() : err_flag(0), ctrl_topic_interval(0) {
        // 安全初始化所有数组
        memset(q, 0, sizeof(q));
        memset(qd, 0, sizeof(qd));
        memset(tau, 0, sizeof(tau));
        memset(omega, 0, sizeof(omega));
        memset(rpy, 0, sizeof(rpy));
        memset(acc, 0, sizeof(acc));
        memset(quat, 0, sizeof(quat));
    }
    
    // 2. 修复拷贝构造函数
    RobotData(const RobotData& other) : RobotData() {  // 委托默认构造函数
        *this = other;  // 现在this是已初始化的
    }
    
    // 3. 修复赋值运算符（保持不变）
    RobotData& operator=(const RobotData& other) {
        if (this != &other) {
            memcpy(q, other.q, sizeof(q));
            memcpy(qd, other.qd, sizeof(qd));
            memcpy(tau, other.tau, sizeof(tau));
            memcpy(omega, other.omega, sizeof(omega));
            memcpy(rpy, other.rpy, sizeof(rpy));
            memcpy(acc, other.acc, sizeof(acc));
            memcpy(quat, other.quat, sizeof(quat));
            err_flag = other.err_flag;
            ctrl_topic_interval = other.ctrl_topic_interval;
        }
        return *this;
    }
    
    // 4. 添加调试方法
    void print() const {
        std::cout << "RobotData - 错误标志: " << err_flag 
                  << ", 间隔: " << ctrl_topic_interval << std::endl;
    }
};

struct MotorCtrl {
    double q_des[12];
    double qd_des[12];
    double kp_des[12];
    double kd_des[12];
    double tau_des[12];

    // 1. 默认构造函数
    MotorCtrl() {
        clear();
    }
    
    // 2. 修复拷贝构造函数 - 避免重复初始化
    MotorCtrl(const MotorCtrl& other) {
        // 直接拷贝，不调用默认构造函数
        memcpy(q_des, other.q_des, sizeof(q_des));
        memcpy(qd_des, other.qd_des, sizeof(qd_des));
        memcpy(kp_des, other.kp_des, sizeof(kp_des));
        memcpy(kd_des, other.kd_des, sizeof(kd_des));
        memcpy(tau_des, other.tau_des, sizeof(tau_des));
    }
    
    // 3. 赋值运算符
    MotorCtrl& operator=(const MotorCtrl& other) {
        if (this != &other) {
            memcpy(q_des, other.q_des, sizeof(q_des));
            memcpy(qd_des, other.qd_des, sizeof(qd_des));
            memcpy(kp_des, other.kp_des, sizeof(kp_des));
            memcpy(kd_des, other.kd_des, sizeof(kd_des));
            memcpy(tau_des, other.tau_des, sizeof(tau_des));
        }
        return *this;
    }
    
    // 4. 添加清空方法
    void clear() {
        memset(q_des, 0, sizeof(q_des));
        memset(qd_des, 0, sizeof(qd_des));
        memset(kp_des, 0, sizeof(kp_des));
        memset(kd_des, 0, sizeof(kd_des));
        memset(tau_des, 0, sizeof(tau_des));
    }
    
    // 5. 添加调试方法
    void print() const {
        std::cout << "MotorCtrl - 第一个关节: q=" << q_des[0] 
                  << ", qd=" << qd_des[0] << std::endl;
    }
};

class CustomInterface {
public:
    CustomInterface(const double &loop_rate);
    void Spin();
    void Stop();

    // 用户需实现的函数（假设为虚函数，用户继承实现）
    virtual void UserCode() = 0;

protected:
    // 保护成员函数
    void Control();
    void motor_cmd_send();

    void receiveMotorData();
    void receiveRobotState();
    void handleCanFrame(int driver_idx, const can_frame& frame);

    // 成员变量
    bool running_;
    bool all_thread_done_;
    bool mode_state;
    double dt_;
    std::thread _user_code_ControlThread;
    mutable std::mutex _dataMutex;  // 保护共享数据的互斥锁
    mutable std::mutex _commandMutex;  // 可选：单独保护命令数据


    // 硬件驱动
    std::array<SocketCanDriver, 4> _drivers;
    YesenseIMU _imu;

    // 数据结构体
    RobotData robot_data;
    MotorCtrl motor_cmd;     // 用户设置的命令
    MotorCtrl _motor_ctrl;   // 处理后的发送命令
};

#endif  // CUSTOM_INTERFACE_H