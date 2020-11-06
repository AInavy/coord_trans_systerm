/************************************************************************
    > File Name: get_RT.cpp
    > Author: lsh
    > Mail: lsh1999mail@163.com 
    > Created Time: 2020年02月24日 星期一 10时47分28秒
************************************************************************/

#include "Rotation_trans.h"


cv::Mat getRx(const double theta_x)
{
    double pi = 3.1415926535898;
    double thetax = theta_x*pi/180.0;

    cv::Mat Rx = cv::Mat(3, 3, CV_64FC1);
    Rx.at<double>(0,0) = 1.0;
    Rx.at<double>(0,1) = 0.0;
    Rx.at<double>(0,2) = 0.0;
    Rx.at<double>(1,0) = 0.0;
    Rx.at<double>(1,1) = cos(thetax);
    Rx.at<double>(1,2) = sin(thetax);
    Rx.at<double>(2,0) = 0.0;
    Rx.at<double>(2,1) = -sin(thetax);
    Rx.at<double>(2,2) = cos(thetax);

    return Rx;
}

cv::Mat getRy(const double theta_y)
{
    double pi = 3.1415926535898;
    double thetay = theta_y*pi/180.0;

    cv::Mat Ry = cv::Mat(3, 3, CV_64FC1);
    Ry.at<double>(0,0) = cos(thetay);
    Ry.at<double>(0,1) = 0.0;
    Ry.at<double>(0,2) = -sin(thetay);
    Ry.at<double>(1,0) = 0.0;
    Ry.at<double>(1,1) = 1.0;
    Ry.at<double>(1,2) = 0.0;
    Ry.at<double>(2,0) = sin(thetay);
    Ry.at<double>(2,1) = 0.0;
    Ry.at<double>(2,2) = cos(thetay);

    return Ry;
}

cv::Mat getRz(const double theta_z)
{
    double pi = 3.1415926535898;
    double thetaz = theta_z*pi/180.0;

    cv::Mat Rz = cv::Mat(3, 3, CV_64FC1);
    Rz.at<double>(0,0) = cos(thetaz);
    Rz.at<double>(0,1) = sin(thetaz);
    Rz.at<double>(0,2) = 0.0;
    Rz.at<double>(1,0) = -sin(thetaz);
    Rz.at<double>(1,1) = cos(thetaz);
    Rz.at<double>(1,2) = 0.0;
    Rz.at<double>(2,0) = 0.0;
    Rz.at<double>(2,1) = 0.0;
    Rz.at<double>(2,2) = 1.0;
    return Rz;
}

cv::Mat getRxyz(const double thetax, const double thetay, const double thetaz)
{

        cv::Mat Rxyz;
        Rxyz = getRx(thetax) * getRy(thetay) * getRz(thetaz);
        return Rxyz;
}


cv::Mat getRzxy(const double thetax, const double thetay, const double thetaz)
{

        cv::Mat Rzyx;
        Rzyx =  getRz(thetaz) * getRx(thetay) * getRy(thetax) ;
        return Rzyx;
}

cv::Mat getRyxz(const double thetax, const double thetay, const double thetaz)
{

        cv::Mat Ryxz;
        Ryxz =  getRy(thetay) * getRx(thetax) * getRz(thetaz);
        return Ryxz;
}

void getQuaternion(cv::Mat Ros, cv::Mat &Qs)
{
    std::vector<double> Q(4);

    cv::Mat R;

    if(Ros.cols == 3 && Ros.rows == 3)
    {
        R = Ros.clone();
    }
    else
    {
        cv::Rodrigues(Ros, R);
    }

    double m11 = R.at<double>(0, 0);
    double m12 = R.at<double>(0, 1);
    double m13 = R.at<double>(0, 2);
    double m21 = R.at<double>(1, 0);
    double m22 = R.at<double>(1, 1);
    double m23 = R.at<double>(1, 2);
    double m31 = R.at<double>(2, 0);
    double m32 = R.at<double>(2, 1);
    double m33 = R.at<double>(2, 2);

    double tr = m11 + m22 + m33;
    double temp = 0.0;
    if(tr > 0.0)
    {
        temp = 0.5f / sqrtf(tr+1);
        Q[0] = 0.25f / temp;
        Q[1] = (m23 - m32) * temp;
        Q[2] = (m31 - m13) * temp;
        Q[3] = (m12 - m21) * temp;
    }
    else
    {
        if(m11 > m22 && m11 > m33)
        {
            temp = 2.0f * sqrtf(1.0f + m11 - m22 - m33);
            Q[0] = (m32 - m23) / temp;
            Q[1] = 0.25f * temp;
            Q[2] = (m12 + m21) / temp;
            Q[3] = (m13 + m31) / temp;
        }
        else if( m22 > m33)
        {
            temp = 2.0f * sqrtf(1.0f + m22 - m11 - m33);
            Q[0] = (m13 - m31) / temp;
            Q[1] = (m12 + m21) / temp;
            Q[2] =  0.25f * temp;
            Q[3] = (m23 + m32) / temp;
        }
        else
        {
            temp = 2.0f * sqrtf(1.0f + m33 - m11 - m22);
            Q[0] = (m21 - m12) / temp;
            Q[1] = (m13 + m31) / temp;
            Q[2] = (m23 + m32) / temp;
            Q[3] = 0.25f * temp;
        }
    }

    double Qmst = sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);

    Qs.at<double>(0) = Q[0] / Qmst;
    Qs.at<double>(1) = Q[1] / Qmst;
    Qs.at<double>(2) = Q[2] / Qmst;
    Qs.at<double>(3) = Q[3] / Qmst;
}


void get_Q_Rotation(const std::vector<double> Quaternion, cv::Mat &rt_mat)
{

    rt_mat.at<double>(0, 0) = 1 - 2 * (Quaternion[2] * Quaternion[2]) - 2 * (Quaternion[3] * Quaternion[3]);
    rt_mat.at<double>(0, 1) = 2 * Quaternion[1] * Quaternion[2] - 2 * Quaternion[0] * Quaternion[3];
    rt_mat.at<double>(0, 2) = 2 * Quaternion[1] * Quaternion[3] + 2 * Quaternion[0] * Quaternion[2];
    rt_mat.at<double>(1, 0) = 2 * Quaternion[1] * Quaternion[2] + 2 * Quaternion[0] * Quaternion[3];
    rt_mat.at<double>(1, 1) = 1 - 2 * (Quaternion[1] * Quaternion[1]) - 2 * (Quaternion[3] * Quaternion[3]);
    rt_mat.at<double>(1, 2) = 2 * Quaternion[2] * Quaternion[3] - 2 * Quaternion[0] * Quaternion[1];
    rt_mat.at<double>(2, 0) = 2 * Quaternion[1] * Quaternion[3] - 2 * Quaternion[0] * Quaternion[2];
    rt_mat.at<double>(2, 1) = 2 * Quaternion[2] * Quaternion[3] + 2 * Quaternion[0] * Quaternion[1];
    rt_mat.at<double>(2, 2) = 1 - 2 * (Quaternion[1] * Quaternion[1]) - 2 * (Quaternion[2] * Quaternion[2]);

    double Rms = cv::determinant(rt_mat);

    rt_mat = rt_mat / Rms;
}
