

#include <iostream>
#include "particle_filter_localization/pf_localizer.h"


// 主要完成粒子初始化 
pfLocalizer::pfLocalizer ( Robot* robot, GridMap* map, MotionModel* motion_model, MeasurementModel* measurement_model , size_t nParticles ) :
    robot_ ( robot ), map_ ( map ), motion_model_ ( motion_model ), measurement_model_ ( measurement_model ),nParticles_ ( nParticles )
{
    is_init_ = false;

    /* 初始化粒子 */
    const double minX = map_->minX();
    const double maxX = map_->maxX();
    const double minY = map_->minY();
    const double maxY = map_->maxY();
    cv::RNG rng ( cv::getTickCount() );
    size_t N = 0;
    double weight = 1.0 / nParticles_;

    while ( N < nParticles_ )
    {   
        // 随机撒粒子  
        // uniform()  返回指定范围内的随机数  
        double x = rng.uniform ( minX, maxX );
        double y = rng.uniform ( minY, maxY );
        double th = rng.uniform ( -PI, PI );

        /* 判断是否在地图内 & 判断是否在可行区域内 */
        double bel;
        if ( !map_->getGridBel ( x, y, bel ) )
        {
            continue;
        }
        if ( bel != 0.0 ) //0.0的区域是可行驶区域
        {
            continue;
        }

        /* 构造一个粒子 */
        N++;
        Particle pt;
        pt.p_pose_ = Pose2d(x, y, th);
        pt.weight_ = weight;
        particles_.push_back ( pt );

    }
} 


















