#ifndef CIRCLECONTROLLER_H
#define CIRCLECONTROLLER_H
#include <opencv2/opencv.hpp>

class circleController
{
public:
    circleController(double max, double min);
   double calculate(double tz, double r_soll, cv::Mat cam);
    void setMax(double max);
    void setMin(double min);
    double getMax();
    double getMin();
private:

    double min_; //maximum value of manipulated variable
    double max_; //minimum value of manipulated variable
};

#endif // CIRCLECONTROLLER_H
