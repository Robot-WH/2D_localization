
#include "ScanMatchers/GaussionNewton.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
GaussionNewtonScanMatcher::GaussionNewtonScanMatcher(double resolution, int size_x, int size_y, double likelihood_sigma, double max_occ_dist, bool debug) 
: _debug(debug)
{
    // 初始化匹配用的地图
    _target_pts = new gn_map(resolution, likelihood_sigma, max_occ_dist, size_x, size_y);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
Map* GaussionNewtonScanMatcher::GetTargetMap()
{
    return _target_pts;  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
GaussionNewtonScanMatcher::~GaussionNewtonScanMatcher()
{
    delete _target_pts;  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 进行高斯牛顿优化．
 * @param[out] init_pose 传入初始姿态(当前激光source_pts相对与_target_pts的位姿), 输出优化的姿态  
 * @param maxIteration 迭代次数  
 */
void GaussionNewtonScanMatcher::Solve(Eigen::Vector3d& init_pose, int maxIteration)
{
    Eigen::Vector3d now_pose = init_pose;
    Eigen::Matrix3d H;
    Eigen::Vector3d b;
    // TODO 判断匹配目标laser点云与待匹配点云是否为空   
    if(0)
    {
        return; 
    }

    LOG(INFO) << "init Score: " << Score(now_pose);

    // 进行高斯牛顿迭代  
    for (int i = 0; i < maxIteration; i++) 
    {
        // 计算本次迭代的jacobian 与 b矩阵
        computeHessianAndb(_target_pts, now_pose, _source_pts, H, b);
        // std::cout << H(0, 0) << H(1, 1) << H(2, 2) << std::endl;
        if ((H(0, 0) != 0.0) && (H(1, 1) != 0.0)) 
        {
            Eigen::Vector3d searchDir(H.inverse() * b);         // 求Hx = b 的解  
            // 对角度限幅
            if (searchDir[2] > 0.2) 
            {
                searchDir[2] = 0.2;
                std::cout << "SearchDir angle change too large\n";
            } 
            else if (searchDir[2] < -0.2) 
            {
                searchDir[2] = -0.2;
                std::cout << "SearchDir angle change too large\n";
            }
            // TODO 如果 searchDir 足够小  直接break;   
            if(std::abs(searchDir[0])<1e-5&&std::abs(searchDir[1])<1e-5&&std::abs(searchDir[2])<1e-5)
            {
                LOG(INFO) << "step is so small! break! "<<" step: " << searchDir.transpose();;
                break;
            }
            // update
            now_pose += searchDir;

            LOG(INFO) << "iterater: " << i + 1 << " step: " << searchDir.transpose(); 
            // 计算得分
            LOG(INFO) << "Score: " << Score(now_pose);
            
        }
    }

    if(_debug == true)
    {
        std::cout << "solve down ! delta_pose: " << (now_pose - init_pose).transpose() << std::endl;
    }

    init_pose = now_pose;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 求解匹配得分
double GaussionNewtonScanMatcher::Score(Eigen::Vector3d const& now_pose)
{
    //  now_pose为当前帧到target的坐标变换    当前now_pose pose 转换为 变换矩阵   trans_now
    Eigen::Matrix3d trans_now;
    double sin_angle = sin(now_pose[2]);
    double cos_angle = cos(now_pose[2]);
    trans_now << cos_angle, -sin_angle, now_pose[0],
                 sin_angle, cos_angle,  now_pose[1],
                 0,         0,          1;
    double sum_score = 0;
    double score = 0;
    int outlier = 0; 
    // 遍历当前每一个激光点
    for (size_t i = 0; i < _source_pts.size(); i++) 
    {
        // 激光雷达坐标系下的激光点的坐标
        Eigen::Vector2d const& pt = _source_pts[i];                       // 激光点以及转换为 (x, y, theta) 了  
        Eigen::Vector3d pt_in_laser_frame(pt[0], pt[1], 1);

        // now此刻激光点云在世界坐标系下面的坐标
        Eigen::Vector3d pt_target = trans_now * pt_in_laser_frame;      // Twl*Pl
        // 找到对应地图的栅格  并获得得分 
        score = _target_pts->MapGetCell(pt_target[0], pt_target[1]).score;
        if(score<0.7)
            outlier++; 
        // 计算得分
        sum_score += 1 - score; 
    }

    LOG(INFO) << "match outlier :" << outlier;
    return sum_score / _source_pts.size();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 设置匹配源  
void GaussionNewtonScanMatcher::SetInputSource(sensor_msgs::LaserScanConstPtr laser)
{
    // 将 ros msg  laser转换为  vector  
    ScanMatcher::ConvertLaserScanToEigenPointCloud(laser, _source_pts);
    // 如果需要对每一个 激光帧可视化的话  
    if(_debug)
    {
        static int frame_i = 0;  
        int height = GetTargetMap()->GetHeight();   
        int width = GetTargetMap()->GetWidth();
        double origin_x = 0;
        double origin_y = 0;
        double resolution = GetTargetMap()->GetResolution();

        cv::Mat img_source(height, width, CV_8UC1, cv::Scalar(255));

        // std::cout << "OriginX: " << origin_x << "origin_y: " << origin_y << "resolution: "<< resolution << std::endl;
        
        for(Eigen::Vector2d& i:_source_pts)
        {
           int col = floor((i[0] - origin_x) / resolution + 0.5) + width / 2;
           int row = height / 2 - floor((i[1] - origin_y) / resolution + 0.5);
           // std::cout << "col: " << col << "row: " << row << std::endl;
           img_source.at<uint8_t>(row, col) = 0;
        }

        cv::imwrite("/home/mini/code/localization_ws/src/indoor_localization/map/" + std::to_string(frame_i++) + ".png", img_source);
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 纯虚函数 设置匹配地图   
void GaussionNewtonScanMatcher::SetInputTarget(Map const& map)
{
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 计算gaussion-newton的jacobian矩阵 与 b 矩阵  
void GaussionNewtonScanMatcher::computeHessianAndb(gn_map* target_pts, Eigen::Vector3d now_pose,
                            std::vector<Eigen::Vector2d> &laser_pts,
                            Eigen::Matrix3d &H, Eigen::Vector3d &b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //  now_pose为当前帧到target的坐标变换    当前now_pose pose 转换为 变换矩阵   trans_now
    Eigen::Matrix3d trans_now;
    double sin_angle = sin(now_pose[2]);
    double cos_angle = cos(now_pose[2]);
    trans_now << cos_angle, -sin_angle, now_pose[0],
                 sin_angle, cos_angle,  now_pose[1],
                 0,         0,          1;

    // 遍历每一个激光点    最后的结果是每一个激光点的结果叠加 
    for (size_t i = 0; i < laser_pts.size(); i++) 
    {
        // 激光雷达坐标系下的激光点的坐标
        const Eigen::Vector2d& pt = laser_pts[i];                       // 激光点以及转换为 (x, y, theta) 了     
        Eigen::Vector3d pt_in_laser_frame(pt[0], pt[1], 1);
        // std::cout << "pt_in_laser_frame: " << pt_in_laser_frame.transpose() << std::endl;

        // now此刻激光点云在世界坐标系下面的坐标
        Eigen::Vector3d pt_target = trans_now * pt_in_laser_frame;      // Twl*Pl  
        Eigen::Vector2d pt_in_target_frame(pt_target[0], pt_target[1]);
	
        // 求激光点在 栅格地图上的坐标      origin_x, origin_y 因该是地图中心 在世界坐标系的坐标    
        double cell_x_double, cell_y_double;
        cell_x_double = (pt_in_target_frame[0] - GetTargetMap()->GetOriginX()) / GetTargetMap()->GetResolution() 
                        + double(GetTargetMap()->GetWidth() / 2);
        cell_y_double = double(GetTargetMap()->GetHeight() / 2) - 
                        (pt_in_target_frame[1] - GetTargetMap()->GetOriginY()) / GetTargetMap()->GetResolution();
        Eigen::Vector2d coord(cell_x_double, cell_y_double);
        // std::cout << "coord col: " << coord[0] << "row: " << coord[1] << std::endl;

        // 插值 获得概率值    返回的是 (插值的概率值, 偏导数 )
        Eigen::Vector3d interpolated_value_gradient = interpMapValueWithDerivatives(target_pts, coord);    // 传入的 coord 的值是 栅格的坐标 
        //LOG(INFO) << "interpolated: " << interpolated_value_gradient.transpose();
        /*********************************************** 检查完毕 OK! *******************************************************/
        double funVal = 1.0 - interpolated_value_gradient[0];

        // 求b 参考论文 (12) 式子  
        b[0] += interpolated_value_gradient[1] * funVal;
        b[1] += interpolated_value_gradient[2] * funVal;
        double rotDeriv = (-sin_angle * pt[0] - cos_angle * pt[1]) * interpolated_value_gradient[1] 
                          + (cos_angle * pt[0] - sin_angle * pt[1]) * interpolated_value_gradient[2];
        // cout << "rotDeriv: " << rotDeriv << endl;
        b[2] += rotDeriv * funVal;

	    // 求H矩阵 参考 论文 (13) 式子   
        // cout << " H(0,0) " << H(0, 0) << " b[2] " << b[2] << endl;
        H(0, 0) += interpolated_value_gradient[1] * interpolated_value_gradient[1];
        H(1, 1) += interpolated_value_gradient[2] * interpolated_value_gradient[2];
        H(2, 2) += rotDeriv * rotDeriv;

        H(0, 1) += interpolated_value_gradient[1] * interpolated_value_gradient[2];
        H(0, 2) += interpolated_value_gradient[1] * rotDeriv;
        H(1, 2) += interpolated_value_gradient[2] * rotDeriv;
    }
    // 考虑对称性   
    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 通过插值求出地图某处的概率值与梯度 
 * @param[in] target_pts 待匹配的地图
 * @param[in] coords 注意这个是 栅格坐标 double型    
 **/
Eigen::Vector3d GaussionNewtonScanMatcher::interpMapValueWithDerivatives(gn_map const* target_pts, Eigen::Vector2d const& coords)
{
    Eigen::Vector3d ans;

    // 判断是否出界
    if (!target_pts->MapValid(coords[0], coords[1])) 
    {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    // 求左上角最近的一个整数点  
    Eigen::Vector2i int_coords(coords.cast<int>());                      // eigen  强制类型转换      
    // 求得小数部分  
    Eigen::Vector2d factors(coords - int_coords.cast<double>());         // x - x0   
    // std::cout << "-------" << std::endl;
    // std::cout << "coords: " << coords.transpose() << std::endl;
    // std::cout << "int_coords: " << int_coords.transpose() << std::endl;
    // std::cout << "factors : " << factors.transpose() << std::endl;

    Eigen::Vector4d intensities;
    // 求得双线性插值需要的四个点的概率值
    /**
     *    x00 - x01
     *     |     |
     *    x10 - x11
     **/
    /*
    // 有4种情况
    if(factors[0]<0.5&&factors[1]<0.5)       // 点击中左上区域  
    {
        intensities[0] = target_pts->_cells[int_coords[1]-1][int_coords[0]-1].score;    // 左上     (0,0)
        intensities[1] = target_pts->_cells[int_coords[1]-1][int_coords[0]].score;    // 右上   (1,0)  
        intensities[2] = target_pts->_cells[int_coords[1]][int_coords[0]-1].score;    // 左下   (0,1)
        intensities[3] = target_pts->_cells[int_coords[1]][int_coords[0]].score;    // 右下    (1,1)

        factors[0] = 0.5 + factors[0];
        factors[1] = 0.5 + factors[1];
    }
    else if(factors[0]>0.5&&factors[1]<0.5)       // 点击中右上区域  
    {
        intensities[0] = target_pts->_cells[int_coords[1]-1][int_coords[0]].score;    // 左上     (0,0)
        intensities[1] = target_pts->_cells[int_coords[1]-1][int_coords[0]+1].score;    // 右上   (1,0)  
        intensities[2] = target_pts->_cells[int_coords[1]][int_coords[0]].score;    // 左下   (0,1)
        intensities[3] = target_pts->_cells[int_coords[1]][int_coords[0]+1].score;    // 右下    (1,1)

        factors[0] = factors[0] - 0.5;
        factors[1] = 0.5 + factors[1];
    }
    else if(factors[0]<0.5&&factors[1]>0.5)       // 点击中左下区域  
    {
        intensities[0] = target_pts->_cells[int_coords[1]][int_coords[0]-1].score;    // 左上     (0,0)
        intensities[1] = target_pts->_cells[int_coords[1]][int_coords[0]].score;    // 右上   (1,0)  
        intensities[2] = target_pts->_cells[int_coords[1]+1][int_coords[0]-1].score;    // 左下   (0,1)
        intensities[3] = target_pts->_cells[int_coords[1]+1][int_coords[0]].score;    // 右下    (1,1)

        factors[0] = factors[0] + 0.5;
        factors[1] = factors[1] - 0.5;
    }
    else if(factors[0]>=0.5&&factors[1]>=0.5)  // 右下区域  
    {
        intensities[0] = target_pts->_cells[int_coords[1]][int_coords[0]].score;    // 左上     (0,0)
        intensities[1] = target_pts->_cells[int_coords[1]][int_coords[0] + 1].score;    // 右上   (1,0)  
        intensities[2] = target_pts->_cells[int_coords[1] + 1][int_coords[0]].score;    // 左下   (0,1)
        intensities[3] = target_pts->_cells[int_coords[1] + 1][int_coords[0] + 1].score;    // 右下    (1,1)

        factors[0] = factors[0] - 0.5;
        factors[1] = factors[1] - 0.5;
    }
    */
    
    intensities[0] = target_pts->_cells[int_coords[1]][int_coords[0]].score;    // 左上     (0,0)
    intensities[1] = target_pts->_cells[int_coords[1]][int_coords[0] + 1].score;    // 右上   (1,0)  
    intensities[2] = target_pts->_cells[int_coords[1] + 1][int_coords[0]].score;    // 左下   (0,1)
    intensities[3] = target_pts->_cells[int_coords[1] + 1][int_coords[0] + 1].score;    // 右下    (1,1)
    
    // std::cout << "intensities: " << intensities.transpose() << std::endl;
    // LOG(INFO) << "intensities: " << intensities.transpose();
    // 求偏导数需要  
    double dx1 = intensities[0] - intensities[1];       // (0,0) - (1,0)
    double dx2 = intensities[2] - intensities[3];       // (0,1) - (1,1)

    double dy1 = intensities[0] - intensities[2];       // (0,0) - (0,1)  
    double dy2 = intensities[1] - intensities[3];       // (1,0) - (1,1)
    
    double xFacInv = (1.0 - factors[0]);     // 1 - x + x0  = x1 - x , (x0 = 0, x1 = 1)
    double yFacInv = (1.0 - factors[1]);     // 1 - y + y0  = y1 - y 

    ans = Eigen::Vector3d
    (   // ( M(P(0,0))*(x1-x) + M(P(1,0))*(x-x0) ) * (y1 - y) + ( M(P(0,1))*(x1-x) + M(P(1,1))*(x-x0) ) * (y - y0)
        ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv))
         + ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),    // 插值出概率  
        // 下面是偏导数    WARNING map->resolution 是因为 这个导数实际上应该求解的是世界坐标系下x,y的导数  , factors的单位是栅格   所以相差一个  1/target_pts->resolution 
        -((dx1 * yFacInv) + (dx2 * factors[1])) / target_pts->_resolution ,  // -( M(P(0,0)) - M(P(1,0)) ) * (y1 - y) - ( M(P(0,1)) - M(P(1,1)) ) * (y - y0)
         ((dy1 * xFacInv) + (dy2 * factors[0])) / target_pts->_resolution   
    );

    return ans;
}









