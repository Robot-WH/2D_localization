
#include "ScanMatchers/imls_icp.h"


ImlsIcp::ImlsIcp()
{

}

ImlsIcp::~ImlsIcp()
{

}

// 匹配求解 
void ImlsIcp::Solve(Eigen::Vector3d& init_pose, int maxIteration)
{
   
}


// 设置匹配源  
void ImlsIcp::SetInputSource(sensor_msgs::LaserScanConstPtr laser)
{

} 


// 纯虚函数 设置匹配地图   
void ImlsIcp::SetInputTarget(Map const& map)
{
    
}







