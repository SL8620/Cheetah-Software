#ifndef CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
#define CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H

#include <CustomInterface.h>


#define CYBERDOG
//#define USE_SIM
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
    {};
    
    CyberdogData cyberdogData;
    CyberdogCmd cyberdogCmd;


private:
    bool first_run = true;
    long long count = 0;
    
    void UserCode();
};

#endif //CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
