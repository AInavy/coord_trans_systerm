#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>
#include <iomanip>
#include "transformation.h"
#include "Rotation_trans.h"
#include "rtk2cam_trajectory.h"

int main()
{


    //read and parse ins and imu data
   /* 
    std::vector<std::string> dst;
    std::string src = "i am a student,you are a teacher!";
    boost::split(dst, src, boost::is_any_of(" "), boost::token_compress_on);
   */


    //Euler to Rotation Matrix 3x3
    cv::Mat R = cv::Mat(3, 3, CV_64F);
    double heading_tmp= 108.0;
    double pitch = 1.3;
    double roll = -0.03;
    if(heading_tmp < 0)
        heading_tmp += 360;

    double heading = heading_tmp;
    R = getRyxz(pitch, roll, -heading);
    std::cout<<"Euler to R: "<<std::endl<<R<<std::endl;

    //-R[1] to R[0] swap

    //Rotation Matrix to Quaternion [w x y z]
    cv::Mat Q = cv::Mat(4, 1, CV_64F);
    getQuaternion(R, Q);
    std::cout<<"R to Q: "<<std::endl<<Q<<std::endl;

    // wgs84->gauss
    double Longitude = 116.34232344;
    double Latitude = 40.23432452;
    double Hell = 25.0;
    double x, y, z = Hell;
    WGS842GKP(Latitude, Longitude, Hell, x, y, z);
    std::cout<<setiosflags(ios::fixed)<<setprecision(8)<<x<<" "<<y<<" "<<z<<std::endl;


    //rtk->cam traj R|t

	cv::Mat RTm(3, 4, CV_64F); //PnP calib method -> imu2cam R|t
    cv::Mat Rm(3, 3, CV_64F), Tm(3, 1, CV_64F);
	Rm.at<double>(0, 0) = 0.9998567909731263;
	Rm.at<double>(0, 1) = -0.0113297695152254;
	Rm.at<double>(0, 2) = -0.01257115220071671;
	Tm.at<double>(0) = 0;//0.09129110161410388;
	Rm.at<double>(1, 0) = -0.01265244226743288;
	Rm.at<double>(1, 1) = -0.007132019309180185;
	Rm.at<double>(1, 2) = -0.999894519439547;
	Tm.at<double>(1) = 0;//-0.3873052626239157;
	Rm.at<double>(2, 0) = 0.01123891674455298;
	Rm.at<double>(2, 1) = 0.9999103812958965;
	Rm.at<double>(2, 2) = -0.007274347194329985;
	Tm.at<double>(2) = 0;//-0.873220970452168;    
	Rm = Rm.t();
	Tm = -Rm * Tm;
	RTm.at<double>(0, 0) = Rm.at<double>(0, 0);
	RTm.at<double>(0, 1) = Rm.at<double>(0, 1);
	RTm.at<double>(0, 2) = Rm.at<double>(0, 2);
	RTm.at<double>(0, 3) = Tm.at<double>(0);
	RTm.at<double>(1, 0) = Rm.at<double>(1, 0);
	RTm.at<double>(1, 1) = Rm.at<double>(1, 1);
	RTm.at<double>(1, 2) = Rm.at<double>(1, 2);
	RTm.at<double>(1, 3) = Tm.at<double>(1);
	RTm.at<double>(2, 0) = Rm.at<double>(2, 0);
	RTm.at<double>(2, 1) = Rm.at<double>(2, 1);
	RTm.at<double>(2, 2) = Rm.at<double>(2, 2);
	RTm.at<double>(2, 3) = Tm.at<double>(2);

    std::cout<<"RTm"<<std::endl<<RTm<<std::endl;
    rtk_info pre_rtk, cur_rtk;
    cv::Mat Rc, tc;

     //20804.781361 4437953.453300 435591.871400 39.580000 1.318000 -2.548000 334.614000
     //20805.081409 4437954.657200 435591.327100 39.590000 1.226000 -2.468000 334.536000
    pre_rtk.lat = 435591.871400;
    pre_rtk.lon = 4437953.453300;
    pre_rtk.alltitude = 39.580000;
    pre_rtk.heading = 334.614000;
    pre_rtk.pitch = -2.548000;
    pre_rtk.roll = 1.318000;

    cur_rtk.lat = 435591.327100;
    cur_rtk.lon = 4437954.657200;
    cur_rtk.alltitude = 39.590000;
    cur_rtk.heading = 334.536000;
    cur_rtk.pitch = -2.468000;
    cur_rtk.roll = 1.226000;
    
    rtk2cam_traj(cur_rtk, pre_rtk, RTm, Rc, tc);
    std::cout<<"Rc"<<std::endl<<Rc<<std::endl<<"tc"<<std::endl<<tc<<std::endl;
    std::cout<<"rtk-s: "<<std::sqrt(std::pow((cur_rtk.lat - pre_rtk.lat), 2.0) +
            std::pow((cur_rtk.lon - pre_rtk.lon), 2.0) +
            std::pow((cur_rtk.alltitude - pre_rtk.alltitude), 2.0))<<std::endl;
    std::cout<<"cam-s: "<<std::sqrt(std::pow(tc.at<double>(0), 2.0) + 
            std::pow(tc.at<double>(1), 2.0) +
            std::pow(tc.at<double>(2), 2.0))<<std::endl;

    return 0;
}

