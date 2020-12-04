#ifndef IMLS_ICP_H
#define IMLS_ICP_H

#include "../scanMatcher.hpp"
#include <Eigen/Core>
 
// imls-icp匹配  
class ImlsIcp : public ScanMatcher
{
public:
    ImlsIcp();

    ~ImlsIcp();

    /**
    * @brief 求解 
    **/
    void Solve(Eigen::Vector3d& init_pose, int maxIteration);   


    /**
    * @brief 设置匹配源  
    **/
    void SetInputSource(sensor_msgs::LaserScanConstPtr laser);  


    /**
     * @brief 纯虚函数 设置匹配地图   
     **/
    virtual void SetInputTarget(Map const &map);



private:
    

}; //class GaussionNewtonScanMatcher

#endif