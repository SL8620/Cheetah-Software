#include <CyberdogInterface.h>

void CyberdogInterface::UserCode()
{
    cyberdogData = robot_data;  // 底层电机imu状态->上层
    motor_cmd = cyberdogCmd;    // 上层电机控制信息->底层

    // bool first_run = true;
    // long long count = 0;
    // float init_q[12];
    // float target1_q[3] = {0 / 57.3, -80 / 57.3, 135 / 57.3};
    // float target2_q[3] = {0 / 57.3, -45 / 57.3, 90 / 57.3};

    // //  vertical view
    //     // leg 1 | | leg 0
    //     //       | |
    //     // leg 3 | | leg 2
    //     //
    //     // Right hand coordinate system
    //     //   ______    zero angle   ______
    //     //   \     \     -->        |     |
    //     //   /     /                |     |
    //     float t = (count / 1500.0) > 2 ? 2 : (count / 1500.0);
    //     if(first_run == true)
    //     {
    //         for(int i = 0; i < 12; i++)
    //             init_q[i] = robot_data.q[i];
    //         if(init_q[2] > 0.1 && init_q[5] > 0.1 && init_q[8] > 0.1 && init_q[11] > 0.1)
    //         {
    //             first_run = false;
    //             count = 0;
    //         }
    //     }
    //     else
    //     {
    //         for(int i = 0; i < 12; i++)
    //         {
    //             if(t < 1.0)
    //                 motor_cmd.q_des[i] = target1_q[i % 3] * t + init_q[i] * (1 - t);
    //             else
    //                 motor_cmd.q_des[i] = target2_q[i % 3] * (t - 1) + target1_q[i % 3] * (2 - t);
    //             motor_cmd.kp_des[i] = 100;
    //             motor_cmd.kd_des[i] = 2;
    //             motor_cmd.qd_des[i] = 0;
    //             motor_cmd.tau_des[i] = 0;
    //         }
    //     }
    //     if((count++) % 1000 == 0)
    //     {
    //         printf("interval:---------%.4d-------------\n", robot_data.ctrl_topic_interval);
    //         printf("rpy [3]:");
    //         for(int i = 0; i < 3; i++)
    //             printf(" %.2f", robot_data.rpy[i]);
    //         printf("\nacc [3]:");
    //         for(int i = 0; i < 3; i++)
    //             printf(" %.2f", robot_data.acc[i]);
    //         printf("\nquat[4]:");
    //         for(int i = 0; i < 4; i++)
    //             printf(" %.2f", robot_data.quat[i]);
    //         printf("\nomeg[3]:");
    //         for(int i = 0; i < 3; i++)
    //             printf(" %.2f", robot_data.omega[i]);
    //         printf("\nq  [12]:");
    //         for(int i = 0; i < 12; i++)
    //             printf(" %.2f", robot_data.q[i]);
    //         printf("\nqd [12]:");
    //         for(int i = 0; i < 12; i++)
    //             printf(" %.2f", robot_data.qd[i]);
    //         printf("\ntau[12]:");
    //         for(int i = 0; i < 12; i++)
    //             printf(" %.2f", robot_data.tau[i]);
    //         printf("\nctrl[12]:");
    //         for(int i = 0; i < 12; i++)
    //             printf(" %.2f", motor_cmd.q_des[i]);
    //         printf("\n\n");
    //     }

}
