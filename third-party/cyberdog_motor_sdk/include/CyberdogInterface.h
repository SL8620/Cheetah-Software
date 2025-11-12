#ifndef CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
#define CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H

#include <CustomInterface.h>
#include <mutex>  // 添加mutex头文件

#define CYBERDOG
#define USE_SIM
#define USE_RC
#define USE_KEYBOARD

/*!
 * Data from Cyberdog
 */

using CyberdogData = RobotData;
using CyberdogCmd = MotorCtrl;

class CyberdogInterface : public CustomInterface
{
public:
    CyberdogInterface(const double &loop_rate) : CustomInterface(loop_rate)
    {};
    
    ~CyberdogInterface()
    {
        // 所有电机失能
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
        std::cout << "====== All Motors has been disabled! ====== " << std::endl;

        // imu关闭
        _imu.close();

        //SocketCan关闭
        for (auto& driver : _drivers) 
        {
            driver.~SocketCanDriver();
        }

        std::cout<< "===== Dog Disabled, See U Next Time! =====" <<std::endl;
    };
    
    
    CyberdogData cyberdogData;
    CyberdogCmd cyberdogCmd;


private:
    bool first_run = true;
    long long count = 0;
    
    void UserCode();
};

#endif //CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
