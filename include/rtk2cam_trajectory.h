#include "Rotation_trans.h"

struct rtk_info{
    double timestamp;
    double lon;
    double lat;
    double alltitude;
    double heading;
    double pitch;
    double roll;
    rtk_info()
    {
        timestamp = 0.0;
        lon = 0.0;
        lat = 0.0;
        alltitude = 0.0;
        heading = 0.0;
        pitch = 0.0;
        roll = 0.0;
    }
};

bool rtk2cam_traj(rtk_info pre_rtk, rtk_info cur_rtk, cv::Mat cam2imuRT, cv::Mat &R, cv::Mat &t);
