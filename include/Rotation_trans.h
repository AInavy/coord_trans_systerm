/************************************************************************
    > File Name: get_RT.h
    > Author: lsh
    > Mail: lsh1999mail@163.com 
    > Created Time: 2020年02月24日 星期一 11时08分19秒
************************************************************************/

#ifndef ROTATION_TRANS_H
#define ROTATION_TRANS_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>

//以下六个是旋转角转旋转矩阵
cv::Mat getRx(const double theta_x);

cv::Mat getRy(const double theta_y);

cv::Mat getRz(const double theta_z);

cv::Mat getRxyz(const double thetax, const double thetay, const double thetaz);

cv::Mat getRzxy(const double thetax, const double thetay, const double thetaz);

//RTK用此函数
cv::Mat getRyxz(const double thetax, const double thetay, const double thetaz);

//旋转矩阵(旋转向量)转四元数
//Ros：输入，旋转矩阵(3*3)或旋转向量(3*1)
//Qs：输出,四元数(4*1,顺序为w,x,y,z)
void getQuaternion(cv::Mat Ros, cv::Mat &Qs);

//四元数转旋转矩阵
//Quaternion：输入，四元数(4*1,顺序为w,x,y,z)
//Qs：输出,旋转矩阵(3*3)
void get_Q_Rotation(const std::vector<float> Quaternion, cv::Mat &rt_mat);

#endif
