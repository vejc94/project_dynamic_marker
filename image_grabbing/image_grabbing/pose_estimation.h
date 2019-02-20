#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

//declaring variables
extern double r_soll;
extern int mon_rows;
extern int mon_cols;
extern int gap;
extern double pixel_pitch;

void estimatePosePNP(std::vector<cv::RotatedRect>& list_ell, cv::Mat &cam, int packing, cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &dist_coeff);
void estimatePoseArucoBoard(cv::Mat &image,cv::Mat &cam, cv::Mat &distCoeff, cv::Vec3d &rvec, cv::Vec3d &tvec);
void estimatePoseArucoMarker(cv::Mat &image ,cv::Mat &distCoeff, cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &cam_pam);
void PoseConics(std::vector<cv::RotatedRect> det_ell, cv::Mat &cam_pam, cv::Vec3d &rvec, cv::Vec3d &tvec, double real_radius, cv::Mat &dist_coeff);
void transform(double x_in, double y_in, double& x_out, double& y_out, cv::Mat &cam_pam, cv::Mat dist_coeff);
bool centerListComparator1(cv::Point point1, cv::Point point2);
bool centerListComparator2(cv::Point point1, cv::Point point2);
void drawAxis(cv::Mat &img, cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &cam_pam, cv::Mat &dist_coeff);
#endif // POSE_ESTIMATION_H
