#ifndef ROBOT_H
#define ROBOT_H

#include "pose_2d.h"

// 机器人参数类 
class Robot{
public:
    Robot(const double& kl, const double& kr, const double& b, const Pose2d& T_r_l):
    kl_(kl), kr_(kr), b_(b), T_r_l_(T_r_l)
    {}
    // 轮子和车身参数                   主要用于轮式里程计结算 
    double kl_, kr_, b_;
    // 激光-里程计外参 
    Pose2d T_r_l_;         
}; //class Robot

#endif