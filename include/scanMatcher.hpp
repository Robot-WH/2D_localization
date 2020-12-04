#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include "map.hpp"
#include "pose_2d.h"
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

 
// 匹配算法抽象类 
class ScanMatcher
{
public:
    ScanMatcher()
    {}
    
    virtual ~ScanMatcher()  
    {}
    
    /**
    * @brief 纯虚函数   匹配的求解 
    * 
    **/
    virtual void Solve(Eigen::Vector3d& init_pose, int maxIteration) = 0;

    /**
     * @brief 纯虚函数 设置匹配源  
     **/
    virtual void SetInputSource(sensor_msgs::LaserScanConstPtr laser) = 0;

    /**
     * @brief 纯虚函数 设置匹配地图   
     **/
    virtual void SetInputTarget(Map const& map) = 0;

    /**
     * @brief 获取目标地图  
     **/
    virtual Map* GetTargetMap() = 0;
    
    /**
     * @brief 求解匹配得分
     */
    virtual double Score(Eigen::Vector3d const& now_pose) = 0;  

    /********************* @TODO 下面这些static 函数 放入到 commom 文件中 **********************************/

    /**
     * @brief 单纯的数据类型转换， 
     * @param[in] msg ros massage 
     * @param[out] eigen_pts 转换结果    (x,y)
     **/ 
    static void ConvertLaserScanToEigenPointCloud(sensor_msgs::LaserScanConstPtr const& msg,
        std::vector<Eigen::Vector2d>& eigen_pts)
    {
        // double max_range = 0;  
        eigen_pts.clear();
        // 遍历全部点   将级坐标式 (l, theta) => (x, y, theta) 
        for (int i = 0; i < msg->ranges.size(); i++) 
        {
            if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;
            // 转换为 (x, y, theta)
            double angle = msg->angle_min + msg->angle_increment * i;
            double lx = msg->ranges[i] * std::cos(angle);
            double ly = msg->ranges[i] * std::sin(angle);

            // if(msg->ranges[i] > max_range)
            //    max_range = msg->ranges[i];  

            if (std::isnan(lx) || std::isinf(ly) || std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx, ly));
        }

        // std::cout<<"max_range: "<<max_range<<std::endl; 

    }

    
    /**
     * @brief 正规化角度
     * TODO static函数 public private权限不同有什么区别? 
     **/
    static double NormalizationAngle(double & angle)
    {
        if (angle > PI)
            angle -= 2 * PI;
        else if (angle < -PI)
            angle += 2 * PI;

        return angle;
    }

    /**
     * @brief (x, y , theta) 转换为变换矩阵  
     **/
    static Eigen::Matrix3d Vector2Transform(Eigen::Vector3d const& vec)
    {
        Eigen::Matrix3d T;
        T << cos(vec(2)), -sin(vec(2)), vec(0),
            sin(vec(2)), cos(vec(2)), vec(1),
            0, 0, 1;
        return T;
    }

    /**
     * @brief 对某一个点进行转换．
     * @return (x,y)
     **/
    static Eigen::Vector2d TransformPoint(Eigen::Vector2d const& pt, Eigen::Matrix3d const& T)
    {
        Eigen::Vector3d tmp_pt(pt(0), pt(1), 1);     // x,y 
        tmp_pt = T * tmp_pt;
        return Eigen::Vector2d(tmp_pt(0), tmp_pt(1));  
    }


protected:
    

}; //class ScanMatcher

#endif