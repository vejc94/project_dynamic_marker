#include "pose_estimation.h"
#include "circlepacking.h"

using namespace cv;

//double pixel_pitch = 0.00018;//mi monitor
double pixel_pitch =  0.000311;//monitor flo

void estimatePosePNP(std::vector<cv::RotatedRect>& list_ell, Mat &cam, int packing, cv::Vec3d &rvec, cv::Vec3d &tvec){
    if(list_ell.empty() && (getCentersSQ().empty()||getCentersHX().empty())) return;

    //monitor parameters
    int mon_rows = 1080;
    int mon_cols = 1920;

    //saving centers' real position on the monitor
    std::vector<cv::Point> list_centers;
    if(packing==0) list_centers = getCentersSQ();
    else if(packing == 1) list_centers = getCentersHX();
    else{std::cout << "invalid type of packing" << std::endl;}

    if(list_ell.size()!=list_centers.size()){
        std::cout << "the number of detected centers and the real number of centers aren't the same" << std::endl;
        std::cout << "detected" << list_ell.size()<<std::endl;
        std::cout << "real" << list_centers.size()<<std::endl;
        return;
    }

    //saving centers from camera image
    std::vector<cv::Point2f> list_det_centers;
    for(uint i = 0; i < list_ell.size(); i++){
        list_det_centers.push_back(list_ell.at(i).center);
    }

    std::sort(list_centers.begin(),list_centers.end(), centerListComparator1); // sort the real centers
    std::sort(list_det_centers.begin(),list_det_centers.end(),centerListComparator2);// sort the detected centers

    std::vector<cv::Point3f> objectPoints(list_centers.size());
    std::vector<cv::Point2f> imagePoints(list_det_centers.size());


    ///converting [u,v]_w into [X_w, Y_w, Z_w]
    for(uint i = 0; i < list_det_centers.size(); i++){
        objectPoints.at(i).x = (list_centers.at(i).x -  mon_cols/2)*pixel_pitch;
        objectPoints.at(i).y = (list_centers.at(i).y - mon_rows/2)*pixel_pitch;
        objectPoints.at(i).z = 0;

        imagePoints.at(i).x = list_det_centers.at(i).x;
        imagePoints.at(i).y = list_det_centers.at(i).y;
    }

    //cv::Mat Rmat; // rotation matrix

    //Pose estimation
    if( (list_det_centers.size() > 3)) {
        cv::solvePnP(objectPoints,imagePoints, cam, std::vector<int>(0), rvec, tvec);
        std::cout<< "translation vector" <<tvec << std::endl;
        std::cout<< "rotation vector" <<rvec << std::endl;
        //cv::Rodrigues(rvec, Rmat);
    }
}


///real centers sorting function. in ascending order
/// the point [0,0](pixels) is in the top left of the world coordinate frame.
bool centerListComparator1(cv::Point point1, cv::Point point2){
    double i = point1.y;
    double j = point2.y;
    if(i==j){
        double k = point1.x;
        double l = point2.x;
        return (k < l) ;
    }
    else{
        return (i < j);
    }
}
///detected centers sorting funtion. in ascending order.
/// the point [0,0] is in the top left of the image frame
bool centerListComparator2(cv::Point point1, cv::Point point2){
    double i = point1.y;
    double j = point2.y;
    if(i==j){
        double k = point1.x;
        double l = point2.x;
        return (k < l) ;
    }
    else{
        return (i < j);
    }
}
