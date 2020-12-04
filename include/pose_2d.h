// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef POSE_2D_H
#define POSE_2D_H

#include <Eigen/Core>

const double PI = 3.1415926;

class Pose2d{
public:
    
    Pose2d(){
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }
    Pose2d(double x, double y, double theta):x_(x), y_(y), theta_(theta){}
    
    // 重载pose的乘法   Pose2d (x,y,theta)  p1*p2   T1*T2  
    const Pose2d operator*(const Pose2d& p2)   
    {  
        Pose2d p;
        Eigen::Matrix2d R;
        // 构造旋转矩阵
        R << cos(theta_), -sin(theta_),
        sin(theta_), cos(theta_);
        Eigen::Vector2d pt2(p2.x_, p2.y_);          
        Eigen::Vector2d pt = R * pt2 + Eigen::Vector2d(x_, y_);     // t = Rt + t 
        
        p.x_ = pt(0);
        p.y_ = pt(1);
        p.theta_ = theta_ + p2.theta_;                              // 这里注意  由于是2D平面  所以旋转可以转换为一个旋转向量 z*theta,  
                                                                    // z为z轴, 即绕着z轴旋转   因此两次旋转 R1*R2 的结果 可以看成绕着z轴连续旋转2次 , 因此角度直接相加 
        NormAngle( p.theta_);                                       // 规范化 
        return p;
    }  
    
    // 重载  T*P
    const Eigen::Vector2d operator*(const Eigen::Vector2d& p)   
    {  
        Eigen::Matrix2d R;
        R << cos(theta_), -sin(theta_),
        sin(theta_), cos(theta_);
        Eigen::Vector2d t(x_, y_);
        return R * p + t;
    }  
    
    
    Pose2d inv()
    {
        double x = -( cos(theta_) * x_ + sin(theta_) * y_);
        double y = -( -sin(theta_) * x_ + cos(theta_) * y_);
        double theta = -theta_;
        return Pose2d(x, y, theta);
    }
    
    // 对角度进行标准化     [-pi, pi]
    static void NormAngle ( double& angle )
    {        
        if( angle >= PI)
            angle -= 2.0*PI;
        if( angle < -PI)
            angle += 2.0*PI;
    }

private:
    double x_, y_, theta_; 

}; //class Pose2d


#endif

