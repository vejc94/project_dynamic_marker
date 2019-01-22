#ifndef CENTER_DETECTOR_H
#define CENTER_DETECTOR_H
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

bool contourAreaComparator(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2);
void centerDetector(cv::Mat &image, std::vector<cv::RotatedRect> &det_ellipses);
#endif // CENTER_DETECTOR_H
