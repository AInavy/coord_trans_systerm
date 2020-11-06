//#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include <iostream>
#include "proj_api.h"
#include "transformation.h"
#include "opencv2/opencv.hpp"
 
using namespace std;
 
void ecef_to_lla(double x, double y, double z, double &latitude, double &longitude, double &alt);
void lla_to_ecef(double latitude, double longitude, double alt, double &x, double &y, double &z);
void WGS842GKP(double Latitude, double Longitude, double Hell, double &x, double &y, double &z);
void GKP2WGS84(double x, double y, double z, int zoneID, double &Longitude, double &Latitude, double &Hell);

#endif
