
/**
 *@brief 用于高斯牛顿匹配的似然场地图 
 *
 **/

#ifndef GN_MAP_HPP
#define GN_MAP_HPP

#include "map.hpp"
#include "scanMatcher.hpp"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <queue>
#include <cstring>
#include <opencv2/opencv.hpp>

// Limits
#define MAP_WIFI_MAX_LEVELS 8
#define CELL_STATUS_FREE (-1)
#define CELL_STATUS_UNKNOWN (0)
#define CELL_STATUS_OCC (+1)

/*
 * 这里的CachedDistanceMap实际上是一个以(0,0)为中心的局部距离地图
 * 这个地图的每一个点都存储着该点的坐标到(0,0)的欧几里得距离。
 * 这里的CacheDistanceMap是用来计算栅格离障碍物距离的时候减少计算量的。
 * 相当于开始进行一次计算的话，后面只要查表就可以计算出来距离，而不需要反复的调用sqrt来求解
*/
class CachedDistanceMap 
{
    public:
    CachedDistanceMap(double len_pre_cell, double max_dist)
        : _distances(NULL)
        , _len_pre_cell(len_pre_cell)
        , _max_dist(max_dist)
    {
        // 最大距离对应的cell的个数   _cell_radius 是 int   所以 _cell_radius * _len_pre_cell <= _max_dist   因此至少需要 _cell_radius + 1 的栅格     
        _cell_radius = _max_dist / _len_pre_cell;
        // distances_ 是一个二维指针    
        _distances = new double*[_cell_radius + 2];   // 创建一个 double* 数组  +2 是因为 _cell_radius + 1 后还要包括障碍物本身的栅格  
	    // 计算以一个栅格为中心  向外扩展 _cell_radius + 2 个栅格的正方形区域的距离数据  
        for (int i = 0; i <= _cell_radius + 1; i++) 
        {
            _distances[i] = new double[_cell_radius + 2];
            for (int j = 0; j <= _cell_radius + 1; j++) 
            {
                _distances[i][j] = sqrt(i * i + j * j);
            }
        }
    }
    
    // 析构函数释放空间  
    ~CachedDistanceMap()
    {  
        // 注意 删除 数组空间要用 delete[]
        if (_distances) {
            // 现将每一个指针对应的空间清除 
            for (int i = 0; i <= _cell_radius + 1; i++)
            {
                delete[] _distances[i];
            }
            // 最后清除 指针数组 
            delete[] _distances;
        }
    }
    
    double** _distances;                  // 提前缓存栅格坐标中离中心的距离   二维指针   指向 double* 的指针  
    double _len_pre_cell;                  // 每个栅格的长度 
    double _max_dist;                      // 这个距离不包括障碍物栅格自身的大小   即从障碍物栅格的边界开始算 
    int _cell_radius;
};

/**
 *单独的地图cell类型
 */
typedef struct
{
    // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
    int occ_state = 0;
    // Distance to the nearest occupied cell
    double occ_dist=100;
    // cell的概率得分  
    double score=0;
    // Wifi levels
    //int wifi_levels[MAP_WIFI_MAX_LEVELS];
} map_cell;

class gn_map; 

class CellData 
{
public:
    gn_map* map_;                             // 为了获取map的其他信息  
    unsigned int i_, j_;                      // 该cell在Map上的坐标
    unsigned int src_i_, src_j_;              // 最近的障碍物栅格的位置 
};

/*
// 操作符号重载    距离比较
bool operator<(const CellData &a, const CellData &b);
*/

/**
 * 高斯牛顿优化匹配用的似然场地图的数据结构
 */
class gn_map : public Map
{
public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 默认构造
    gn_map():_resolution(0), _min_score(0), _likelihood_sigma(0)
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    gn_map(double resolution, double likelihood_sigma, double max_occ_dist, int size_x, int size_y): _resolution(resolution), 
    _likelihood_sigma(likelihood_sigma), _max_occ_dist(max_occ_dist), Map(0., 0. , size_x, size_y)
    {
        _cells = std::vector<std::vector<map_cell>>(_size_y, std::vector<map_cell>(_size_x));

        std::cout << "gn map initialized OK! "
                  << " size X: " << _cells[0].size() << " Y: " << _cells.size() << std::endl;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void MapClear()
    {
        // std::cout << "gn map clear !" << std::endl;
        /*
        _cells.clear();
        _cells.resize(_size_y, std::vector<map_cell>(_size_x));
        std::vector<std::vector<map_cell>>::iterator it;

        for (it = _cells.begin(); it != _cells.end(); it++)
        {
        //    it->resize(_size_y);
        } */
        
        _cells = std::vector<std::vector<map_cell>>(_size_y, std::vector<map_cell>(_size_x));
        // std::cout << "gn map col:" << _cells[0].size() << " row: "<< _cells.size() << std::endl;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Test to see if the given map coords lie within the absolute map bounds. 判断是否出界
    bool  MapValid(int col, int row) const 
    {
        return (col >= 0) && (col < _size_x) && (row >= 0) && (row < _size_y);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 线性存储时  找index 
    int MapIndex(int cols, int rows) const
    {
        return (cols + rows*_size_x);   
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 将一帧laser加入到地图
    void AddScan(Eigen::Vector3d const& scan_origin_pt, sensor_msgs::LaserScanConstPtr laser)
    {
        // 地图坐标系到世界坐标系的转换  
        Eigen::Matrix3d Trans = ScanMatcher::Vector2Transform(scan_origin_pt);
        std::vector<Eigen::Vector2d> laser_pts;
        // 将laser msg 数据转换为 Eigen::Vector2d 格式  
        ScanMatcher::ConvertLaserScanToEigenPointCloud(laser, laser_pts);

        // 设置障碍物   
        for (int i = 0; i < laser_pts.size(); i++) 
        {
            // 将地图系下的障碍物转换到世界坐标下 
            Eigen::Vector2d tmp_pt = ScanMatcher::TransformPoint(laser_pts[i], Trans);    // (x,y)
            std::vector<int> cell_xy;
            // 世界坐标转地图坐标   需要用到地图的 _origin_x , _origin_y  
            cell_xy = GetCoordsWorld2Cell(tmp_pt(0), tmp_pt(1));   // (col, row)
            // MapIndex() 把二维地图转化为一维数组index
            _cells[cell_xy[1]][cell_xy[0]].occ_state = CELL_STATUS_OCC;
        }

        // 进行障碍物的膨胀--最大距离固定死．
        mapUpdateLikelihood(_max_occ_dist);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 将一帧点云数据移除map
    void RemoveScan()
    {

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 设置地图中心在世界坐标系下的坐标
    void SetOriginCoord(Eigen::Vector3d const origin_coord)
    {
        _origin_x = origin_coord[0];
        _origin_y = origin_coord[1];
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    /**
     * @brief 可视化 
     **/
    cv::Mat Visualization() const
    {
        // 构造opencv mat 
        cv::Mat gn_map_img( _size_y, _size_x, CV_8UC1, cv::Scalar(255));
        
        // 转为灰度
        for (int row = 0; row < _size_y; row++)
        {
            for (int col = 0; col < _size_x; col++)
            {
                gn_map_img.at<uint8_t>(row, col) = _cells[row][col].score * 255;
            }
        }

        return gn_map_img; 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Get the cell at the given point
    map_cell MapGetCell(double ox, double oy)
    {
        // 世界坐标系转栅格序号   
        std::vector<int> cell_coords = GetCoordsWorld2Cell(ox, oy);    // (col, row)  
        map_cell cell;
        if (!MapValid(cell_coords[0], cell_coords[1]))
            return cell;

        return _cells[cell_coords[1]][cell_coords[0]];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     *@brief 世界坐标系转换为map坐标系  
     *
     */
    std::vector<int> GetCoordsWorld2Cell(double x, double y)
    {
        std::vector<int> cell_coords = {0,0}; 
        // 先判断 地图尺寸是奇数还是偶数   floor()  返回小于传入参数的最大整数  
        if(_size_x%2)   
        { 
           cell_coords[0] = floor((x - _origin_x) / _resolution + 0.5) + _size_x / 2;  // 奇数
        }
        else
        {
          cell_coords[0] = floor((x - _origin_x) / _resolution) + _size_x / 2;  // 偶数
        }

        if(_size_y%2)   
        { 
           cell_coords[1] =  _size_y / 2 - floor((y - _origin_y) / _resolution + 0.5);  // 奇数
        }
        else
        {
          cell_coords[1] = _size_y / 2 - floor((y - _origin_y) / _resolution);          // 偶数
        }

        return cell_coords;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int const& GetWidth() const
    {
        return _size_x;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int const& GetHeight() const
    {
        return _size_y;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double const& GetOriginX() const
    {
        return _origin_x;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double const& GetOriginY() const
    {
        return _origin_y;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double const& GetResolution() const
    {
        return _resolution;  
    }

private:

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief  更新地图的距离值 这个函数用来计算用来定位的地图中的每隔栅格到最近障碍物的距离
     *         其中障碍物的栅格的距离为0 然后通过bfs进行搜索来计算每一个栅格到障碍物的距离
     * @param max_occ_dist 离障碍物的最大距离 
     */
    void mapUpdateLikelihood(double max_occ_dist)
    {
        // std::priority_queue<CellData> Q;           // 为什么要用优先队列 ???????????
        std::queue<CellData> Q;                      // 队列 

        // 计算出得分的就标记 
        unsigned char* marked;
        marked = new unsigned char[_size_x * _size_y];
        memset(marked, 0, sizeof(unsigned char) * _size_x * _size_y);

        // 得到一个CachedDistanceMap   单例模式  
        CachedDistanceMap* cdm = get_distance_map(_resolution, max_occ_dist);
        // 这个sigma已经在外面设置过了 在handmap msg里面就会设置
        _min_score = exp(-max_occ_dist * max_occ_dist / (2 * _likelihood_sigma * _likelihood_sigma));

        /*************************** 所有的障碍物都放入队列Q中 **************************************/
        CellData cell;
        cell.map_ = this;
        //计算出来所有的边界障碍物 只有边界障碍物才用来进行匹配 其他的障碍物都当成no-information
        /*所有障碍物的栅格  离障碍物的距离都标志为0  非障碍物的栅格都标记为max_occ_dist*/
        for (int col = 0; col < _size_x; col++) 
        {
            cell.src_i_ = cell.i_ = col;
            for (int row = 0; row < _size_y; row++)
            {
                // 如果该cell是占据的话    那么里障碍物的距离就直接设置为0 了  
                if (_cells[row][col].occ_state == CELL_STATUS_OCC) 
                {
                    // 最近距离设置为 0 
                    _cells[row][col].occ_dist = 0.0;
                    _cells[row][col].score = 1.0;         // 得分直接设成1  
                    cell.src_j_ = cell.j_ = row;                    // 作为障碍物的位置进行设置 
                    marked[MapIndex(col, row)] = 1;                 // 占据的标记为1  
                    Q.push(cell);                                   // 放置到队列 
                }
                else
                {
                    _cells[row][col].occ_dist = max_occ_dist;   // 非占据的栅格  距离直接设置为最大 
                }
            }
        }
        
        // BFS  
        while (!Q.empty()) 
        {
            CellData current_cell = Q.front();
            // CellData current_cell = Q.top();

            /*往上、下、左、右四个方向拓展*/
            // 如果当前cell的x坐标大于0   则检查左边的
            if (current_cell.i_ > 0)
            {
                    calculateGaussionValue(current_cell.i_ - 1, current_cell.j_,
                        current_cell.src_i_, current_cell.src_j_,
                        Q, cdm, marked);
            }
            // 如果当前cell的y坐标大于0   则检查上边的
            if (current_cell.j_ > 0)
            {
                    calculateGaussionValue(current_cell.i_, current_cell.j_ - 1,
                        current_cell.src_i_, current_cell.src_j_,
                        Q, cdm, marked);
            }
            // 如果当前cell的x坐标小于最大值  则检查右边的
            if ((int)current_cell.i_ < _size_x - 1)
            {
                    calculateGaussionValue(current_cell.i_ + 1, current_cell.j_,
                        current_cell.src_i_, current_cell.src_j_,
                        Q, cdm, marked);
            }
            // 如果当前cell的y坐标小于最大值  则检查下边的
            if ((int)current_cell.j_ < _size_y - 1)
            {
                    calculateGaussionValue(current_cell.i_, current_cell.j_ + 1,
                        current_cell.src_i_, current_cell.src_j_,
                        Q, cdm, marked);
            }

            Q.pop();
        }
        
        delete[] marked;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 
     * @param map       对应的地图
     * @param i         该点对应的x坐标
     * @param j         该点对应的y坐标
     * @param src_i     该点对应的障碍物的x坐标
     * @param src_j     该点对应的障碍物的y坐标
     * @param Q
     * @param cdm
     * @param marked
     */
    void calculateGaussionValue(unsigned int i, unsigned int j,
        unsigned int src_i, unsigned int src_j,
        std::queue<CellData>& Q,
        //std::priority_queue<CellData>& Q,
        CachedDistanceMap* cdm,
        unsigned char* marked)
    {
        // 如果已经计算过了   那么不需要后续的计算了  直接退出即可     
        if (marked[MapIndex(i, j)])
            return;

        // 这里计算的距离是栅格的距离
        unsigned int di = abs(i - src_i);
        unsigned int dj = abs(j - src_j);
        double distance = cdm->_distances[di][dj];     // 通过 CachedDistanceMap 获取距离 
        // 若该栅格距离过大  那么就不会向外扩展了  
        if (distance > cdm->_cell_radius)
            return;

        // 转换为实际距离 
        _cells[j][i].occ_dist = distance * _resolution;
        /* 计算高斯似然场 value  */
        double z = _cells[j][i].occ_dist;
        // 得分  最大为1  
        _cells[j][i].score = exp(-(z * z) / (2 * _likelihood_sigma * _likelihood_sigma));
        // 构造cell 
        CellData cell;
        cell.map_ = this;
        cell.i_ = i;
        cell.j_ = j;
        // 设置该cell的对应障碍
        cell.src_i_ = src_i;
        cell.src_j_ = src_j;
        // 将空闲的cell放入  用于后续继续扩张  
        Q.push(cell);
        // 计算完毕设置为 1   
        marked[MapIndex(i, j)] = 1;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 构造  CachedDistanceMap
     * @param[in] len_pre_cell 每一个cell 的真实size
     * @param[in] max_dist 障碍物影响的最大距离   
     * @return CachedDistanceMap指针  
     */
    CachedDistanceMap* get_distance_map(double len_pre_cell, double max_dist)
    {
        static CachedDistanceMap* cdm = NULL;    // 局部static 变量  1. 在全局数据区分配内存   2. 在程序执行到该对象的声明处时被首次初始化，即以后的函数调用不再进行初始化
                                                 // 3. 作用域为局部作用域，也就是不能在函数体外面使用它  
        // 先判断主要参数是否变化    
        if (!cdm || (cdm->_len_pre_cell != len_pre_cell) || (cdm->_max_dist != max_dist))
        {   // 如果变换则 先删除以前的数据  
            if (cdm)
                delete cdm;
            cdm = new CachedDistanceMap(len_pre_cell, max_dist);
        }

        return cdm;
    }
    
public:
    // Map scale (m/cell) 地图的分辨率
    double _resolution;
    // The map data, stored as a grid    指向栅格地图存储区   地图保存的位置 !!!!!!!!!!!!!   
    //map_cell* _cells;
    std::vector<std::vector<map_cell>> _cells;  
    // Max distance at which we care about obstacles, for constructing
    // likelihood field
    double _min_score;
    double _likelihood_sigma;                    //似然场的标准差
    double _max_occ_dist;                        // 似然场的最大影响距离 
};

/*
// 操作符号重载    距离比较
bool operator<(const CellData& a, const CellData& b)
{   // 小顶堆  
    return a.map_->_cells[a.j_, a.i_].occ_dist > b.map_->_cells[b.j_, b.i_].occ_dist;
}
*/ 

/************** 保留的一些函数 **************/
// Convert from map index to world coords  地图坐标转换到世界坐标
// @TODO 把i, j的()去除  
#define MAP_WXGX(map, i) (map._origin_x + (i-map._size_x / 2) * map._resolution)   // origin_x, origin_y 为地图中心的世界坐标  
#define MAP_WYGY(map, j) (map._origin_y + (j-map._size_y / 2) * map._resolution)

// Convert from world coords to map coords 世界坐标转换到地图坐标   只适用对与奇数的地图size
#define MAP_GXWX(map, x) (floor((x - map._origin_x) / map._resolution + 0.5) + map._size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map._origin_y) / map._resolution + 0.5) + map._size_y / 2)

#define MAP_GXWX_DOUBLE(map, x) ((x - map._origin_x) / map._resolution + 0.5 + double(map._size_x / 2))
#define MAP_GYWY_DOUBLE(map, y) ((y - map._origin_y) / map._resolution + 0.5 + double(map._size_y / 2))


#endif
