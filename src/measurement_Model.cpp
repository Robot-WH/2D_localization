
#include "particle_filter_localization/measurement_Model.h"


MeasurementModel::MeasurementModel(GridMap* map, const double& sigma, const double& rand_weight, 
                                   const double& gaussion_weight, const int& scale):
map_(map), sigma_(sigma), rand_weight_(rand_weight), gaussion_weight_(gaussion_weight), scale_(scale)   
{
   // 计算似然场模型
   calculateLikelihoodField();
   // 验证似然场模型
   checkLikelihoodModel();  
}

MeasurementModel::~MeasurementModel()
{
}

// 计算似然场  
void MeasurementModel::calculateLikelihoodField()
{
    /* 初始化似然域模型, 预先计算的似然域模型的格子大小   与原栅格地图有scale的尺度变化  */
    size_width_ = scale_ * map_->GetWidth();
    size_height_ = scale_ * map_->GetHeight();
    init_x_ = scale_ * map_->GetInitX();
    init_y_ = scale_ * map_->GetInitY();
    cell_size_ = map_->GetCellsize() / scale_;
    // 初始化似然场模型空间 
    likelihood_data_.resize(size_height_, vector<double>(size_width_, 0)); 

    // 构造kdtree
    pcl::PointCloud<pcl::PointXY>::Ptr obstacle_tree (new pcl::PointCloud<pcl::PointXY>());
    // 遍历地图  将地图中的障碍物的map系下坐标记录在kdtree中 
    for (int row = 0; row < map_->GetHeight(); row++)
    {
        for (int col = 0; col < map_->GetWidth(); col++)
        {
            if(map_->GetMapBelData()[row][col]!=1.0)   // 如果非障碍物 则跳过  
                continue;  
            // 如果为障碍  保存坐标     
            pcl::PointXY point;
            point.x = (col - map_->GetInitX()) * map_->GetCellsize();
            point.y = (map_->GetInitY() - row) * map_->GetCellsize();
            obstacle_tree->push_back(point);
        }
    }

    kd_tree_.setInputCloud(obstacle_tree);
    std::cout << "measuremodel kdtree is done !!" << std::endl;

    // 提前计算高斯分布
    calculateGaussionIdxValue();  
    std::cout << " 高斯分布数据库以提前准备好  !!" << std::endl;

    // 遍历似然场每个位置
    for (int row = 0; row < size_height_; row++)
    {
        for (int col = 0; col < size_width_; col++)
        {
            likelihood_data_[row][col] = calulateLikelihoodGridValue(row, col); 
        }
    }

    std::cout << "likelihood model is done !!" << std::endl;

}

/**
 * @brief 求似然场 x,y坐标处的值
 * @param[in] row likelihood_data_ y坐标
 * @param[in] col likelihood_data_ x坐标
 * @return 似然场value 
 **/
double MeasurementModel::calulateLikelihoodGridValue(const double& row, const double& col)
{
    // 构造被搜索的点
    pcl::PointXY search_point;
    // 先根据似然场的栅格坐标计算世界坐标系下的真实坐标
    search_point.x = (col - init_x_) * cell_size_;
    search_point.y = (init_y_ - row) * cell_size_;  
    
    // kdtree 搜索 
    std::vector<int> k_indices;                                                                    // 搜索到的最近距离
    std::vector<float> k_sqr_distances;
    int nFound =  kd_tree_.nearestKSearch(search_point, 1, k_indices, k_sqr_distances);
    // 取最近的一个距离 
    double dist = k_sqr_distances.at(0);
    double gaussion_value = 0;
    /*
    // 加速求解   
    if(gaussion_idx_value.count(dist))
    {
        gaussion_value = gaussion_idx_value[dist];
        std::cout << "gaussion_value: " << gaussion_value << " dist: "<<dist<<std::endl;
    }
    else
        gaussion_value = 0; 
    */
    // 直接求解   
    gaussion_value = gaussion(0, sigma_, dist);
    // std::cout << "gaussion_value: " << gaussion_value << " dist: "<<dist<<std::endl;
        
    // 生成0-1随机分布  
    cv::RNG rng ( cv::getTickCount() );
    double rand_probability = rng.uniform ( 0., 1.);

    return gaussion_weight_ * gaussion_value + rand_weight_ * rand_probability;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// 提前准备高斯分布数据  
void MeasurementModel::calculateGaussionIdxValue()
{
    double dist = 0;
    double three_sigma = 3 * sigma_;      // 计算3 sigma 距离范围内的距离
    std::cout << "3 sigma距离: " << three_sigma << std::endl;

    while (dist<=three_sigma)
    {
        gaussion_values_.push_back(gaussion(0, sigma_, dist));  
        std::cout << "dist: " << dist << " value: " << gaussion_values_.back() << std::endl;
        dist += cell_size_;     // 求真实距离 
    }
}

const double PI = 3.1415926;  

/**
 * @brief 高斯函数采样
 * @param[in] mu 均值
 * @param[in] sigma 标准差 
 * @param[in] x 距离值
 * @return 概率值   注意 可以大于1 !!!
 **/
double MeasurementModel::gaussion ( const double& mu, const double& sigma, double x)
{
    return (1.0 / (sqrt( 2 * PI ) * sigma) ) * exp( -0.5 * (x-mu) * (x-mu) / (sigma* sigma) );
}

bool MeasurementModel::getIdx(const double& x, const double& y, Eigen::Vector2i& idx)
{
}

// 转换成图片输出 
void MeasurementModel::checkLikelihoodModel()
{
    cv::Mat likelihood_raw_img(size_height_, size_width_, CV_64FC1);
    double max_likelihood_value = 0;  
    // 获取最大像素值 
    for (int row = 0; row < size_height_; row++)
    {
        for (int col = 0; col < size_width_; col++)
        {   
            if(likelihood_data_[row][col] > max_likelihood_value)
                 max_likelihood_value = likelihood_data_[row][col];
            likelihood_raw_img.at<double>(row, col) = likelihood_data_[row][col];
        }
    }

    std::cout << "max_likelihood_value: " << max_likelihood_value << endl;
    likelihood_raw_img /= max_likelihood_value;    // 归一化
    // 构造opencv mat 
    cv::Mat likelihood_img(size_height_, size_width_, CV_8UC1);
    // 转为灰度
    for (int row = 0; row < size_height_; row++)
    {
        for (int col = 0; col < size_width_; col++)
        {
            likelihood_img.at<uchar>(row, col) = likelihood_raw_img.at<double>(row, col) * 255;
        }
    }
    
    if(!likelihood_img.empty())
    {
        cv::imshow("visualize", likelihood_img);
        cv::waitKey(); 
        cv::destroyWindow("visualize"); 
    }
    else
    {
        std::cout << " likelihood mode ERROR!" << std::endl;
    }

}



















