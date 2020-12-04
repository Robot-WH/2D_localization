
/**
 * @brief 基于似然场的观测模型   
 * @author wenhao li, congping li 
 * @date 2020/9/26
 * 
 **/

#ifndef _MEASUREMENT_MODEL_H
#define _MEASUREMENT_MODEL_H


#include "2d_GridMap.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_map>
#include <opencv2/opencv.hpp>

 
class MeasurementModel{
public:
    MeasurementModel(GridMap* map, const double& sigma, const double& rand_weight, const double& gaussion_weight, const int& scale);
    ~MeasurementModel();

    /**
     * @brief 计算似然场模型
     * @details 在初始化阶段提前计算好似然场模型
     **/
    void calculateLikelihoodField();  
    
    /**
     * @brief 计算似然场栅格坐标为x,y出的值
     * 
     **/
    double calulateLikelihoodGridValue(const double& x, const double& y); 
    /**
     * @brief 获取似然场的idx 
     */
    bool getIdx(const double& x, const double& y, Eigen::Vector2i& idx);
    /**
     * @brief 设置似然场的值
     */
    bool setGridLikelihood(const double& x, const double& y, const double& likelihood);
    /**
     * @brief 获取似然场的值
     */
    double getGridLikelihood ( const double& x, const double& y);

protected:

    /**
     * @brief 检查似然场模型计算情况
     * @details 似然场模型转换为图像输出  
     */
    void checkLikelihoodModel();  

    /**
     * @brief 提前计算高斯分布
     * @details 根据3sigma原则 计算
     */
    void calculateGaussionIdxValue(); 

private:

    /**
     * @brief 高斯函数采样
     * @param[in] mu 均值
     * @param[in] sigma 方差 
     * @param[in] x 距离值
     * @return 概率值  
     **/
    static double gaussion(const double& mu, const double& sigma, double x);
    
    GridMap* map_;              // 占据栅格地图
    double sigma_;              // 似然场的  标准差     
    double rand_weight_;        // 随机概率权重
    double gaussion_weight_;        // 高斯概率权重
     
    // kdtree 最近邻查找 方法
    pcl::KdTreeFLANN<pcl::PointXY> kd_tree_;
    
    /* 预先计算似然域的数据 */
    std::vector<vector<double>> likelihood_data_;
    // 似然场的size   
    int size_width_, size_height_, init_x_, init_y_;
    double cell_size_;
    double scale_;                  // 似然场分辨率 与 栅格地图的比值   
    // 计算高斯分布 dist从0到3sigma距离 
    std::vector<double> gaussion_values_;  

}; //class MeasurementModel



#endif 