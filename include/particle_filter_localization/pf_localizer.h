
/**
 * @brief 基于粒子滤波的定位框架    
 * @author wenhao li, congping li 
 * @date 2020/9/26
 * 
 **/

#ifndef _PF_LOCALIZER_H
#define _PF_LOCALIZER_H

#include "robot.h"
#include "pose_2d.h"
#include "2d_GridMap.hpp"
#include "particle_filter_localization/motion_Model.h"
#include "particle_filter_localization/measurement_Model.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>



/**
 * @brief 粒子滤波的定位, 实现 运动学模型,  观测模型, 重采样
 * TODO: 基于KLD的重采样
 **/

class pfLocalizer
{
public:
    // 粒子的结构体
    struct Particle  
    {
       // 位姿
       Pose2d p_pose_;
       // 权重
       long double weight_;    
    };

public:
    pfLocalizer(Robot* robot, GridMap* map, MotionModel* motion_model, MeasurementModel* measurement_model, size_t nParticles);
    /**
     * @brief 粒子运动学状态转移 
     **/   
    void motionUpdate(const Pose2d& odom);  
    /**
     * @brief 测量更新 
     * TODO: 如何处理动态障碍  
     **/
    void measurementUpdate(const sensor_msgs::LaserScanConstPtr& scan );
    /**
     * @brief 重采样
     **/
    void reSample();
    void particles2RosPoseArray(geometry_msgs::PoseArray& pose_array);
    /**
     * @brief 权重归一化 
     **/
    void normalization();
    
private:
    // 机器人的参数类  
    Robot* robot_;
    // 栅格  
    GridMap* map_;
    // 运动模型
    MotionModel* motion_model_;
    // 测量模型 
    MeasurementModel* measurement_model_;
    
    size_t nParticles_;
    std::vector<Particle> particles_;       // 保存全部粒子的vector  
    /* sys statues */
    bool is_init_;
    Pose2d last_odom_pose_;
};




#endif