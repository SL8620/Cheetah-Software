#include <CyberdogInterface.h>

void CyberdogInterface::UserCode()
{
    {
        std::lock_guard<std::mutex> lock(_dataMutex);
        cyberdogData = robot_data;  // 现在这是安全的
        motor_cmd = cyberdogCmd;    // 现在这是安全的
    }
}