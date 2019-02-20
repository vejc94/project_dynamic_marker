#include "center_detector.h"

//image processing to detect the ellipse and find the middle point
void centerDetector(cv::Mat &image, std::vector<cv::RotatedRect> &det_ellipses){

    //clear list
    det_ellipses.clear();

    //gray image
    cv::Mat image_gr;

    //convert image to gray scales
    cv::cvtColor(image,image_gr,CV_BGR2GRAY);

    image_gr.convertTo(image_gr, CV_8UC1); // gray scaled one channel image
    cv::Mat bin_image; // binarized image 0 - 255
    cv::threshold(image_gr, bin_image, 50, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours; // list of detected contours
    cv::findContours(bin_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), contourAreaComparator);//sort contours by contour area in descending order

    // contour(0) is display contour for black circles. if the contour has less than 5 point it is not an ellipse

    //add all detected ellipses to the list
    for(uint i = 0; i < contours.size(); i++){

        if((contours.at(i).size() > 20 )){
            cv::RotatedRect det_ellipse = cv::fitEllipse(contours.at(i));
            double area_ell = 0.25 * M_PI * det_ellipse.size.width * det_ellipse.size.height;
            double area_con = cv::contourArea(contours.at(i));
            double A_ratio = area_con/area_ell;
            if((A_ratio>0.98)&&(A_ratio<1.1)){
                cv::ellipse(image, det_ellipse, cv::Scalar(255, 0, 255), 3);
                //cv::drawContours(image, contours, i, cv::Scalar(255, 0, 255), 3);
                det_ellipses.push_back(det_ellipse);
            }
        }
    }

}
//organize contours by area in decreasing order
bool contourAreaComparator(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2){
    double i = fabs( contourArea(contour1));
    double j = fabs( contourArea(contour2));
    return ( i > j );
}
