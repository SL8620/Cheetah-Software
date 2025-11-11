#include <CyberdogInterface.h>

void CyberdogInterface::UserCode()
{
    cyberdogData = robot_data;  // 底层电机imu状态->上层
    motor_cmd = cyberdogCmd;    // 上层电机控制信息->底层
}
