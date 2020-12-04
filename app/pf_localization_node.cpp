/**
 * @brief 基于粒子滤波的蒙特卡洛定位节点  参考 AMCL   
 * @details 实现基于KLD采样的粒子滤波定位  接收 传感器融合的里程计数据作为运动传播, 激光的观测进行观测更新   
 * @author wenhao li, congping li 
 **/


#include "particle_filter_localization/pf_localizer.h"
#include <ros/ros.h>

// 数据发布 
ros::Publisher map_pub, particles_pub;   // 地图和粒子  

// 里程计回调函数 
void odometryCallback ( const nav_msgs::OdometryConstPtr& odom )
{
 

}

// 激光的回调函数 
void laserCallback ( const sensor_msgs::LaserScanConstPtr& scan )
{
   
}



int main ( int argc, char **argv )
{
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "pf_localization_node" );
    ros::NodeHandle nh_private("~");
    
    // 加载地图 
    std::string map_img_dir, map_cfg_dir;
    int initx, inity;
    double cell_size;
 
    nh_private.getParam ( "map/map_img_dir", map_img_dir );
    // 初始位置  
    nh_private.getParam ( "map/initx", initx );
    nh_private.getParam ( "map/inity", inity );
    nh_private.getParam ( "map/cell_size", cell_size );
    ROS_INFO_STREAM("map_img_dir: "<<map_img_dir<<" initx: "<<initx<<" inity: "
                     <<inity<<" cell_size: "<<cell_size );
    GridMap *map = new GridMap();
    // 加载地图   
    map->loadMap ( map_img_dir, initx, inity, cell_size );
    /*
    // 验证地图的加载 
    map->SaveMap("/home/mini/code/localization_ws/src/indoor_localization/map/map2.png",
                 "/home/mini/code/localization_ws/src/indoor_localization/map/mapInfo.txt" );
    */
    // 初始化观测模型
    double sigma, rand_weight, gaussion_weight, scale;
    nh_private.getParam ( "measurement_model/sigma", sigma );    // 高斯分布方差
    nh_private.getParam ( "measurement_model/rand_weight", rand_weight );          // 随机概率权重 
    nh_private.getParam ( "measurement_model/gaussion_weight", gaussion_weight );  // 高斯概率权重 
    nh_private.getParam ( "measurement_model/scale", scale );      // 分辨率与栅格地图的比值  
    std::cout << "\n\n正在计算似然域模型, 请稍后......\n\n";
    ROS_INFO_STREAM("sigma: "<<sigma<<" rand_weight: "<<rand_weight<<" gaussion_weight: "
                     <<gaussion_weight<<" scale: "<<scale );
    MeasurementModel* measurement_model = new MeasurementModel ( map, sigma, rand_weight, gaussion_weight, scale );
    std::cout << "\n\n似然域模型计算完毕.  开始接收数据.\n\n";

    // 初始化粒子滤波定位器  
    
    
    /* 初始Topic */
    ros::Subscriber odom_sub = nh_private.subscribe ( "/odometry", 10, odometryCallback );
    ros::Subscriber laser_sub = nh_private.subscribe ( "/scan", 10, laserCallback );
    map_pub = nh_private.advertise<nav_msgs::OccupancyGrid> ( "pf_localization/grid_map", 10 );
    particles_pub = nh_private.advertise<geometry_msgs::PoseArray> ( "pf_localization/particles", 10 );
    
    ros::spin();

}


