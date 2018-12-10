#include "circlecontroller.h"
#include <iostream>
circleController::circleController(int max, int min)
{
max_=max; //maximal size
min_=min; //minimal size
}

double circleController::calculate(vpHomogeneousMatrix cMw, double r_soll){
    ///r_soll (radius in the image plane in pixels)
    /// tz = Z coordinate in camera frame
    ///l_x = 4,8 µm (length pixel of the camera)
    /// f = 0.005 m (focal length)
    /// ml_x =180 µm (length pixel of the monitor)
    /// R (radius of the displayed circle in pixels)

    double l_x = 0.0000048;
    double ml_x =  0.00018;
    double f = 0.005;
    double tz = cMw[2][3];

    double R = (r_soll*l_x*tz)/(f*ml_x);
    if(R>max_){
        R=max_;
    }
    else if(R<min_){
        R=min_;
    }
    return R;

}
