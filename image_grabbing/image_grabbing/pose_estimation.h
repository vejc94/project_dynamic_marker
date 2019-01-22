#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
void estimatePosePNP(std::vector<cv::RotatedRect>& list_ell, cv::Mat cam, int packing, cv::Vec3d &rvec, cv::Vec3d &tvec);
#endif // POSE_ESTIMATION_H
