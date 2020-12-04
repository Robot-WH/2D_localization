
/**
 * @brief 融合多传感器 轮速计 + IMU +激光里程计 实现激光里程计   
 * @author wenhao li, congping li 
 * @date 2020/9/26
 **/

#include <ros/ros.h>
#include <image_transport/image_transport.h>   // find_package() 要包含这个  
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

#include "tic_toc.h"
#include "map.hpp"
#include "scanMatcher.hpp"
#include "ScanMatchers/GaussionNewton.h"
#include "ScanMatchers/imls_icp.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#define USE_MAP_DEBUG 1

/**
 * @brief 多传感器融合激光里程计类   
 * @details 策略模式
 * @details 轮速计 与 IMU 作为预测, 激光匹配作为观测    
 **/  
class MultiSensorFusionLaserOdometry
{

public:  

    // 地图更新方法的枚举 
    enum _LocalmapUpdateMethod
    {
        singleScan,            // scan-scan的 更新方法
        LimitScanNumber,       // 局部地图由最近的若干帧组成
        LimitScanNumberAndArea // 局部地图由一定区域内的若干帧组成
    };
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    MultiSensorFusionLaserOdometry( ros::NodeHandle const& nh, ScanMatcher *p_scan_matcher,
                                    _LocalmapUpdateMethod localmap_update_method = LimitScanNumberAndArea)
    :_nh(nh), _p_scan_matcher(p_scan_matcher), _localmap_update_method(localmap_update_method)
    {
        // 订阅laser的数据
        _laserscan_Sub = _nh.subscribe("/scan", 5, &MultiSensorFusionLaserOdometry::rosLaserScanCallback, this);
        //_wheelSpeed_Sub = _nh.subscribe("--", 5, &MultiSensorFusionLaserOdometry::rosWheelSpeedsCallback, this);
        // 发布激光里程记 
        _odom_Pub = _nh.advertise<nav_msgs::Odometry>("odom", 1, true);
        // 发布path  
        _path_Pub = _nh.advertise<nav_msgs::Path>("path", 1, true);

        odom_frame_id = "odom";
        laser_frame_id = "laser";  
        // 发布地图 img
        image_transport::ImageTransport transport(_nh); 
        map_img_pub = transport.advertise("map/image", 1);

        _last_pose = {0.,0.,0.};    // 上一时刻的pose
        _curr_pose = {0.,0.,0.};    // 当前时刻的pose
        _ref_pose = {0.,0.,0.};     // 上一个关键帧pose
        _delta_pose = {0.,0.,0.};   // 增量  

        _isFirstFrame = true;  
    }
   
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ~MultiSensorFusionLaserOdometry()
   {
       delete _p_scan_matcher;  
   }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 动态设置匹配方法
    void SetScanMatcherMethod(ScanMatcher* p_scan_matcher)
    {
        _p_scan_matcher = p_scan_matcher; 
    }
    
private:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 激光的回调函数 
    void rosLaserScanCallback(sensor_msgs::LaserScanConstPtr const& msg)
    {
        //static int i = 0;  
        // 是第一帧数据
        if (_isFirstFrame == true) 
        {
            std::cout << "First Frame" << std::endl;
            
            updataLaserLocalMap(_curr_pose, msg);                // 更新用于匹配的局部地图 
            // m_odomPath.push_back(nowPose);
            // m_gaussianNewtonPath.push_back(nowPose);
            _isFirstFrame = false;
            _ref_pose = _curr_pose;  
            //i++;
            // _p_scan_matcher->SetInputSource(msg);
            // debug
            //cv::Mat map_img = _p_scan_matcher->GetTargetMap()->Visualization();   // 获取匹配的地图的图像
            //cv::imwrite("/home/mini/code/localization_ws/src/indoor_localization/map/frist.png", map_img);
            return;
        }
        // TODO: 计算预测值
        _curr_pose += _delta_pose;  
        // if(i>1)  return;
        // 进行匹配  求得相对运动
        _p_scan_matcher->SetInputSource(msg);
        TicToc tt;
        tt.tic();  
        _p_scan_matcher->Solve(_curr_pose, 50);
        std::cout << "scanMatching time: " << tt.toc() << std::endl;
        std::cout << "curr_pose: " << _curr_pose.transpose() << std::endl;
        _delta_pose = _curr_pose - _last_pose;  
        _last_pose = _curr_pose;  
        updataLaserLocalMap(_curr_pose, msg); // 更新用于匹配的局部地图

        /***************************************** 地图进行可视化 *********************************************/ 
        cv::Mat map_img = _p_scan_matcher->GetTargetMap()->Visualization();   // 获取匹配的地图的图像
        //std::cout << "map_img row: "<<map_img.rows<<" col: "<< map_img.cols << std::endl;

        if(map_img.empty())
        {
            std::cout << "map is empty!" << std::endl;
            return; 
        }
        
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", map_img).toImageMsg();
        map_img_pub.publish(img_msg);

        #if USE_MAP_DEBUG == 1
            cv::imshow("0", map_img);
            cv::waitKey(10);  
        #endif
        
        /***************************************** 里程计发布 *********************************************/ 
        // 发布odom的数据   发布path数据
        publish_odometry(msg->header.stamp, _curr_pose);  
        
    }

    /*
    // 轮速计的回调函数 
    void rosWheelSpeedsCallback()
    {

    }
    */

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 更新局部匹配地图  
    void updataLaserLocalMap(Eigen::Vector3d const& laser_Pose, sensor_msgs::LaserScanConstPtr laser_msg) 
    {
        TicToc tt;
        tt.tic();  
        // scan - scan 
        if(_localmap_update_method == singleScan)                    // 局部地图为单帧  
        {   
            // 首先判断运动是否充分  否则不更新匹配地图 
            if(!isKeyframe())
                return;  
            Map *target_map = _p_scan_matcher->GetTargetMap(); // 获取匹配算法的地图对象
            // 如果非第一帧  那么需要清空
            if(!_isFirstFrame)
            {
                // 由于是scan-scan的匹配 所以首先要清除之前的地图
                target_map->MapClear();
            }

            // 设置地图中心在参考坐标系下的坐标
            target_map->SetOriginCoord(laser_Pose);  
            target_map->AddScan(laser_Pose, laser_msg);                  // 给地图添加一帧
            _last_pose = laser_Pose;  
            std::cout << "map update time: " << tt.toc() << std::endl; 

        }
        else if(_localmap_update_method == LimitScanNumberAndArea)   // 局部地图
        {
            // 首先判断运动是否充分
            if(1)
            {
                std::cout << "map update time: " << tt.toc() << std::endl; 
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 判断当前运动是否充分  是否需要更新关键帧 
    bool isKeyframe()
    {
        Eigen::Vector3d delta_motion = _curr_pose - _ref_pose;
        std::cout << "delta_motion: " << delta_motion.transpose() << std::endl;
        // 如果是第一帧的话   那么是关键帧 
        if(_isFirstFrame)
            return true;  
        // 如果运动平移的距离大于1   旋转 大于 30度  也是关键帧 
        if(delta_motion[0]*delta_motion[0] + delta_motion[1]*delta_motion[1] > 1 || std::fabs(delta_motion[2]) > M_PI / 6)
            return true;
        return false;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief publish odometry
     * @param stamp  timestamp
     * @param pose   odometry pose to be published
     */
    void publish_odometry(const ros::Time& stamp, const Eigen::Vector3d& curr_pose) 
    {
        // 由角度创建四元数    pitch = 0, roll = 0, yaw = curr_pose[2];  
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(curr_pose[2]);
        // publish the odom                发布当前帧里程计到/odom话题                 
        nav_msgs::Odometry odom;                            
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id;        // odom坐标    /odom
        odom.child_frame_id = laser_frame_id;        // /lidar_odom
        odom.pose.pose.position.x = curr_pose[0];
        odom.pose.pose.position.y = curr_pose[1];
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;  
        _odom_Pub.publish(odom);
        // 发布轨迹  
        geometry_msgs::PoseStamped laserPose;    
        laserPose.header = odom.header;
        laserPose.pose = odom.pose.pose;                // 和laserOdometry的pose相同  
        static nav_msgs::Path laserPath;                    // 记录轨迹  
        laserPath.header.stamp = odom.header.stamp;
        laserPath.poses.push_back(laserPose);
        laserPath.header.frame_id = odom_frame_id;      // odom坐标     /odom
        _path_Pub.publish(laserPath);
        // 发布TF变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = laser_frame_id;
        
        odom_trans.transform.translation.x = curr_pose[0];
        odom_trans.transform.translation.y = curr_pose[1];
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        static tf::TransformBroadcaster odom_broadcaster;
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
    }

    /**
     * TODO  匹配算法类只负责完成匹配运算 , 地图的维护在这个类中完成 !!
     * 对于 地图, 也需要实现成 基类 - 派生类的模式 
     **/

private:
    bool _isFirstFrame;  
    // 地图构造方法的枚举
    _LocalmapUpdateMethod _localmap_update_method;  

    Eigen::Vector3d _last_pose;
    Eigen::Vector3d _ref_pose;
    Eigen::Vector3d _curr_pose;
    Eigen::Vector3d _delta_pose;         // 运动增量    

    std::string odom_frame_id;   // 里程计坐标
    std::string laser_frame_id;  // 激光坐标 

    // 匹配算法基类指针
    ScanMatcher* _p_scan_matcher;  

    ros::NodeHandle _nh;

    ros::Subscriber _laserscan_Sub;        // laser的订阅者
    ros::Subscriber _wheelSpeed_Sub;       // 轮速的订阅者
    ros::Subscriber _imu_Sub;              // imu的订阅者

    ros::Publisher _odom_Pub;              // 激光里程计发布  
    ros::Publisher _path_Pub;              // 路径发布
    // 发布地图 img
    image_transport::Publisher map_img_pub; // 地图 图像的发布
};



int main ( int argc, char **argv )
{
    std::string ADDR = "/home/mini/code/localization_ws/src/indoor_localization"; 
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = ADDR + "/Log";
    FLAGS_logtostderr = 1;      //是否打印到控制台
    FLAGS_alsologtostderr = 1;  //打印到日志同时是否打印到控制台 
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "msf_odometry_node" );
    ros::NodeHandle private_nh("~");

    // 局部, 匹配方法
    MultiSensorFusionLaserOdometry msf_odometry(private_nh, new GaussionNewtonScanMatcher(0.01, 2001, 2001, 0.3, 0.5, false), 
                                                MultiSensorFusionLaserOdometry::singleScan);    

    ros::spin();

    return 0;  
}
