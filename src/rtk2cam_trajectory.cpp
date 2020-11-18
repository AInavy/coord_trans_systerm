#include "rtk2cam_trajectory.h"


// cam traj: pre -> cur: R|t
bool rtk2cam_traj(rtk_info pre_rtk, rtk_info cur_rtk, cv::Mat cam2imuRT, cv::Mat &R, cv::Mat &t)
{
	cv::Mat R1(3, 3, CV_64F), T1(3, 1, CV_64F), R2(3, 3, CV_64F), T2(3, 1, CV_64F);

    pre_rtk.heading = pre_rtk.heading < 0? pre_rtk.heading+360.0 : pre_rtk.heading;
    cur_rtk.heading = cur_rtk.heading < 0? cur_rtk.heading+360.0 : cur_rtk.heading;

	R1 = getRyxz(pre_rtk.pitch, pre_rtk.roll, -pre_rtk.heading);
	R2 = getRyxz(cur_rtk.pitch, cur_rtk.roll, -cur_rtk.heading);

	T1.at<double>(0) = pre_rtk.lon;
	T1.at<double>(1) = pre_rtk.lat;
	T1.at<double>(2) = pre_rtk.alltitude;
	T2.at<double>(0) = cur_rtk.lon;
	T2.at<double>(1) = cur_rtk.lat;
	T2.at<double>(2) = cur_rtk.alltitude;

	cv::Mat Rc(3, 3, CV_64F), Tc(3, 1, CV_64F);

	Rc = R1.t() * R2;
	Tc = Rc * R1 * (T2 - T1);

	cv::Mat R0(3, 3, CV_64F), T0(3, 1, CV_64F);

	cv::Mat R_cam2imu(3, 3, CV_64F), T_cam2imu(3, 1, CV_64F);
	R_cam2imu = cam2imuRT.colRange(0, 3);
	T_cam2imu = cam2imuRT.colRange(3, 4);

	R0 = R_cam2imu * Rc * R_cam2imu.t();
	T0 = R_cam2imu * Tc + T_cam2imu - R0 * T_cam2imu;


   // R = R0.clone();
    R = R0;
    t = T0;
	
//	t.at<double>(0) = T0.at<double>(0);
//	t.at<double>(1) = T0.at<double>(1);
//	t.at<double>(2) = T0.at<double>(2);

	return true;

}
