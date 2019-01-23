#ifndef CIRCLEPACKING_H
#define CIRCLEPACKING_H

#include <math.h>
#include <opencv2/core.hpp>

void squarePacking(double radius);
void hexagonalPacking(double radius);
void squarePackingContinous(double radius);
void hexagonalPackingContinous(double radius);
std::vector<cv::Point> getCentersHX();
std::vector<cv::Point> getCentersSQ();




#endif // CIRCLEPACKING_H
