#ifndef CIRCLECONTROLLER_H
#define CIRCLECONTROLLER_H
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>


class circleController
{
public:
    circleController(int max, int min);

    double calculate(vpHomogeneousMatrix cMw, double r_soll);
private:

    double min_; //maximum value of manipulated variable
    double max_; //minimum value of manipulated variable
};

#endif // CIRCLECONTROLLER_H
