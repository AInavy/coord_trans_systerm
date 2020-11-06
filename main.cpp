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
    const double pitch = 1.3;
    const double roll = -0.03;
    if(heading_tmp < 0)
        heading_tmp += 360;

    const double heading = heading_tmp;
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
        
    return 0;
}

