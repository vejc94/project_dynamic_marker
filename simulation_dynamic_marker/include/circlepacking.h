#ifndef CIRCLEPACKING_H
#define CIRCLEPACKING_H
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

extern int mon_rows;
extern int mon_cols;
extern int gap;
void squarePacking(cv::Mat image, double radius);
void hexagonalPacking(cv::Mat image, double radius);
void squarePackingContinous(cv::Mat image, double radius);
void hexagonalPackingContinous(cv::Mat image, double radius);
std::vector<cv::Point> getCentersHX();
std::vector<cv::Point> getCentersSQ();
#endif // CIRCLEPACKING_H
