#ifndef CUSTOM_INTERFACE_H
#define CUSTOM_INTERFACE_H

#include <thread>
#include <array>
#include <cstdint>  // 用于uint8_t等
#include "socketcan/socketcan_driver.hpp"
#include "yesense/yesense_driver.hpp"

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
    int ctrl_topic_interval;  // 假设为int，根据原代码可调整
};

struct MotorCtrl {
    double q_des[12];
    double qd_des[12];
    double kp_des[12];
    double kd_des[12];
    double tau_des[12];
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

    // 硬件驱动
    std::array<SocketCanDriver, 4> _drivers;
    YesenseIMU _imu;

    // 数据结构体
    RobotData robot_data;
    MotorCtrl motor_cmd;     // 用户设置的命令
    MotorCtrl _motor_ctrl;   // 处理后的发送命令
};

#endif  // CUSTOM_INTERFACE_H