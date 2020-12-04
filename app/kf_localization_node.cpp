
/**
 * @brief 基于滤波器的松耦合融合马尔可夫定位节点   参考cartographer  
 * @details 融合多传感器 轮速计 + IMU +激光匹配    
 * @author wenhao li, congping li 
 * @date 2020/9/26
 * 
 **/


#include <ros/ros.h>


int main ( int argc, char **argv )
{
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "kf_localization_node" );
    ros::NodeHandle nh;

}