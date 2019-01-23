#include "circlecontroller.h"
circleController::circleController(double max, double min)
{
max_=max; //maximal size
min_=min; //minimal size
}

double circleController::calculate(double tz, double r_soll, cv::Mat cam){
    ///r_soll (radius in the image plane in pixels)
    /// tz = Z coordinate in camera frame (estimated)
    ///kl_x = 3.7 µm (length pixel of the camera)
    /// f = 0.005 m (focal length)
    /// ml_x =180 µm (length pixel of the monitor)
    /// R (radius of the displayed circle in pixels)

    double kl_x = 0.00000375;//point grey camera
    //double ml_x =  0.00018;//mi monitor
    double ml_x =  0.000311;//monitor flo

    float f = cam.at<float>(0,0)*kl_x;

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
