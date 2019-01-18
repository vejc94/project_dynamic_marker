#ifndef CIRCLECONTROLLER_H
#define CIRCLECONTROLLER_H
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>


class circleController
{
public:
    circleController(double max, double min);

    double calculate(vpHomogeneousMatrix cMw, double r_soll);
    void setMax(double max);
    void setMin(double min);
    double getMax();
    double getMin();
private:

    double min_; //maximum value of manipulated variable
    double max_; //minimum value of manipulated variable
};

#endif // CIRCLECONTROLLER_H
