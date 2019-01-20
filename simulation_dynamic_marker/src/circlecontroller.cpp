#include "circlecontroller.h"
#include <iostream>
circleController::circleController(double max, double min)
{
max_=max; //maximal size
min_=min; //minimal size
}

double circleController::calculate(vpHomogeneousMatrix cMw, double r_soll){
    ///r_soll (radius in the image plane in pixels)
    /// tz = Z coordinate in camera frame
    ///kl_x = 4,8 µm (length pixel of the camera)
    /// f = 0.005 m (focal length)
    /// ml_x =180 µm (length pixel of the monitor)
    /// R (radius of the displayed circle in pixels)

    double kl_x = 0.0000048;
    double ml_x =  0.00018;
    double f = 0.005;
    double tz = cMw[2][3];

    double R = (r_soll*kl_x*tz)/(f*ml_x);
    if(R>max_){
        R=max_;
    }
    else if(R<min_){
        R=min_;
    }
    return R;

}

double circleController::calculate(cv::Vec3d tvec, double r_soll){
    ///r_soll (radius in the image plane in pixels)
    /// tz = Z coordinate in camera frame (estimated)
    ///kl_x = 4,8 µm (length pixel of the camera)
    /// f = 0.005 m (focal length)
    /// ml_x =180 µm (length pixel of the monitor)
    /// R (radius of the displayed circle in pixels)

    double kl_x = 0.0000048;
    double ml_x =  0.00018;
    double f = 0.005;
    double tz = tvec(2);

    double R = (r_soll*kl_x*tz)/(f*ml_x);
    if(R>max_){
        R=max_;
    }
    else if(R<min_){
        R=min_;
    }
    return R;
}

void circleController::setMax(double max){
    max_ = max;
}
void circleController::setMin(double min){
    min_ = min;
}
double circleController::getMax(){
    return max_;
}
double circleController::getMin(){
    return min_;
}
