#ifndef GAUSSION_NEWTON_H
#define GAUSSION_NEWTON_H

#include "../gn_map.hpp"
#include "../scanMatcher.hpp"
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
 
// 高斯牛顿匹配  
class GaussionNewtonScanMatcher : public ScanMatcher
{
public:
    GaussionNewtonScanMatcher();
    GaussionNewtonScanMatcher(double resolution, int size_x, int size_y, double likelihood_sigma, double max_occ_dist, bool _debug = false);  

    ~GaussionNewtonScanMatcher();

    /**
    * @brief 高斯牛顿求解   重写纯虚函数   
    **/
    void Solve(Eigen::Vector3d& init_pose, int maxIteration);    

    /**
    * @brief 设置匹配源     重写纯虚函数 
    **/
    void SetInputSource(sensor_msgs::LaserScanConstPtr laser);  

    /**
     * @brief 纯虚函数 设置匹配地图   
     **/
    void SetInputTarget(Map const &map);
    
    /**
     * @brief 获取目标地图  
     **/
    Map* GetTargetMap(); 

    /**
     * @brief 求解匹配得分
     */
    double Score(Eigen::Vector3d const& now_pose);

private:

    /**
     * @brief ComputeCompleteHessianAndb
     * 计算H*dx = b中的H和b
     * @details 计算jacobian 与 b矩阵 需要求取地图的梯度以及变换的导数
     * @param[in] map 用于匹配的地图 
     * @param[in] now_pose 初始的位姿 
     * @param[in] laser_pts 用于匹配的激光点
     * @param[out] H 求取的H = JTJ
     * @param[out] b 
     */
    void computeHessianAndb(gn_map *map, Eigen::Vector3d now_pose,
                            std::vector<Eigen::Vector2d> &laser_pts,
                            Eigen::Matrix3d &H, Eigen::Vector3d &b);

    /**
     * @brief InterpMapValueWithDerivatives
     * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
     * 返回值为Eigen::Vector3d ans
     * ans(0)表示势场值
     * ans(1:2)表示梯度
     * @param map
     * @param coords
     * @return
     */
    Eigen::Vector3d interpMapValueWithDerivatives(gn_map const* map, Eigen::Vector2d const& coords);


    // 匹配的目标点云
    gn_map* _target_pts; 
    // 当前点云  
    std::vector<Eigen::Vector2d> _source_pts;
    // 是否进行调试
    bool _debug;  



}; //class GaussionNewtonScanMatcher

#endif