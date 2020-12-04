
#ifndef MAP_HPP
#define MAP_HPP

#include "pose_2d.h"
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>

// 地图抽象类 
class Map 
{
public:
    Map()
    {
    }

    Map(double origin_x, double origin_y, int size_x, int size_y):_origin_x(origin_x), _origin_y(origin_y), _size_x(size_x), _size_y(size_y)
    {
    }

    virtual ~Map()
    {
    }

    // 地图的清空    
    virtual void MapClear() = 0;

    // 将一帧点云数据添加到map
    virtual void AddScan(Eigen::Vector3d const& scan_origin_pt, sensor_msgs::LaserScanConstPtr laser) = 0;

    // 将一帧点云数据移除map
    virtual void RemoveScan() = 0;

    // 设置地图中心在世界坐标系下的坐标
    virtual void SetOriginCoord(Eigen::Vector3d const origin_coord) = 0;

    virtual int const& GetWidth() const = 0; 

    virtual int const& GetHeight() const = 0;

    virtual double const& GetOriginX() const = 0; 

    virtual double const& GetOriginY() const = 0; 


    // 2D地图的可视化
    virtual cv::Mat Visualization() const
    {
        return cv::Mat{};  
    }

    virtual double const& GetResolution() const
    {
        return 0.0;  
    }

public:
    // Map origin; map初始位置在世界坐标下的位置 
    double _origin_x, _origin_y;
    // Map dimensions (number of cells) X Y方向的栅格数
    int _size_x, _size_y;

}; //class Map

#endif