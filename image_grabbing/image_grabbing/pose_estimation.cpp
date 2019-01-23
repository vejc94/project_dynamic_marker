#include "pose_estimation.h"
#include "circlepacking.h"



//dictionary
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(10));

//parameters for aruco grid board 32 corners = 4x2 markers
int markers_x = 4; //Numbers of markers in X direction
int markers_y= 2; //Numbers of markers in Y direction
float markers_length = 410*pixel_pitch; //markers side lenght in meter (number of pixels * pixel pitch)
float markers_gap = 50*pixel_pitch; // Separation between two consecutive markers in the grid in meter (number of pixels * pixel pitch)

//create board
cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markers_x, markers_y, markers_length, markers_gap, dictionary);

//parameter for marker detection
std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
std::vector<int> markerIds;

using namespace cv;



void estimatePosePNP(std::vector<cv::RotatedRect>& list_ell, Mat &cam, int packing, cv::Vec3d &rvec, cv::Vec3d &tvec){
    if(list_ell.empty() && (getCentersSQ().empty()||getCentersHX().empty())) return;

//    //monitor parameters
//    int mon_rows = 1080;
//    int mon_cols = 1920;

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

void estimatePoseArucoBoard(cv::Mat &image, cv::Mat &cam, cv::Mat &distCoeff, cv::Vec3d &rvec, cv::Vec3d &tvec){

    //detect markers
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates,cam);

    //estimate Pose
    int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cam, distCoeff, rvec, tvec);
    if(!valid){
        std::cout<<"did not estimate pose"<<std::endl;
        return;
    }
    cv::aruco::drawAxis(image, cam, distCoeff, rvec, tvec, markers_length);
    std::cout<< "translation vector" <<tvec << std::endl;
    std::cout<< "rotation vector" <<rvec << std::endl;

    return;
}

void estimatePoseArucoMarker(cv::Mat &image, cv::Mat &distCoeff, cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &cam_pam){
    double single_marker_length = 1080*pixel_pitch;

    //detect markers
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates,cam_pam);

    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::estimatePoseSingleMarkers(markerCorners, single_marker_length, cam_pam, distCoeff, rvecs, tvecs);

    cv::aruco::drawAxis(image, cam_pam, distCoeff, rvec, tvec, single_marker_length);
    if(!rvecs.empty()&&!tvecs.empty())
    {
        rvec=rvecs.at(0);tvec=tvecs.at(0);
    }
    std::cout<< "translation vector" <<tvec << std::endl;
    std::cout<< "rotation vector" <<rvec << std::endl;
}


void PoseConics(std::vector<cv::RotatedRect> det_ell, cv::Mat &cam_pam, cv::Vec3d &rvec, cv::Vec3d &tvec, double real_radius){
    double x,y,x1,x2,y1,y2,sx1,sx2,sy1,sy2,major,minor,v0,v1;

    if(det_ell.empty())
    {
        std::cout << "no ellipses detected" << std::endl;
        return;
    }

    cv::RotatedRect circle = det_ell.at(0);

    //transform the center
    transform(circle.center.x, circle.center.y, x,y, cam_pam);

    //circle parameters v0,1 and m0,1 in image coordinates
    float circle_m0 = circle.size.width*0.25;
    float circle_m1 = circle.size.height*0.25;
    float circle_v0 = cos(circle.angle/180.0 * M_PI);
    float circle_v1 = sin(circle.angle/180.0 * M_PI);

    //calculate the major axis
    //endpoints in image coords
    sx1 = circle.center.x + circle_v0 * circle_m0 * 2;
    sx2 = circle.center.x - circle_v0 * circle_m0 * 2;
    sy1 = circle.center.y + circle_v1 * circle_m0 * 2;
    sy2 = circle.center.y - circle_v1 * circle_m0 * 2;

    //endpoints in camera coords
    transform(sx1, sy1, x1, y1,cam_pam);
    transform(sx2, sy2, x2, y2, cam_pam);

    //semiaxis length
    major = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;

    v0 = (x2-x1)/major/2.0;
    v1 = (y2-y1)/major/2.0;

    //calculate the minor axis
    //endpoints in image coords
    sx1 = circle.center.x + circle_v1 * circle_m1 * 2;
    sx2 = circle.center.x - circle_v1 * circle_m1 * 2;
    sy1 = circle.center.y - circle_v0 * circle_m1 * 2;
    sy2 = circle.center.y + circle_v0 * circle_m1 * 2;

    //endpoints in camera coords
    transform(sx1, sy1, x1, y1,cam_pam);
    transform(sx2, sy2, x2, y2, cam_pam);

    //semiaxis length
    minor = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;

    //construct the conic
    double a,b,c,d,e,f;
    a = v0*v0/(major*major)+v1*v1/(minor*minor);
    b = v0*v1*(1/(major*major)-1/(minor*minor));
    c = v0*v0/(minor*minor)+v1*v1/(major*major);
    d = (-x*a-b*y);
    e = (-y*c-b*x);
    f = (a*x*x+c*y*y+2*b*x*y-1);

    cv::Matx33d data(a,b,d,b,c,e,d,e,f);

    cv::Vec3d eigenvalues;
    cv::Matx33d eigenvectors;
    cv::eigen(data, eigenvalues, eigenvectors);

    // compute ellipse parameters in real-world
    double L1 = eigenvalues(1);
    double L2 = eigenvalues(0);
    double L3 = eigenvalues(2);
    int V2 = 0;
    int V3 = 2;

    //position
    float circle_diameter = (real_radius-gap)*2*pixel_pitch;//converting from pixels into meters the radius of the circle shown in the display

    double z = circle_diameter/sqrt(-L2*L3)/2.0;
    cv::Matx13d position_mat = L3 * sqrt((L2 - L1) / (L2 - L3)) * eigenvectors.row(V2) + L2 * sqrt((L1 - L3) / (L2 - L3)) * eigenvectors.row(V3);
    tvec = cv::Vec3f(position_mat(0), position_mat(1), position_mat(2));
    int S3 = (tvec(2) * z < 0 ? -1 : 1);
    tvec*=S3*z;


    //rotation
    cv::Matx13d normal_mat = sqrt((L2 - L1) / (L2 - L3)) * eigenvectors.row(V2) + sqrt((L1 - L3) / (L2 - L3)) * eigenvectors.row(V3);
    cv::normalize(cv::Vec3f(normal_mat(0), normal_mat(1), normal_mat(2)), rvec, 1, cv::NORM_L2SQR);
    rvec(0)= atan2(rvec(1), rvec(0));
    rvec(2)= 0; /* not recoverabel */

    std::cout<< "translation vector" <<tvec << std::endl;
    std::cout<< "rotation vector" <<rvec << std::endl;

    return;
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
//transform coordinates into canonical frame.
void transform(double x_in, double y_in, double& x_out, double& y_out, cv::Mat &cam_pam){
    x_out = (x_in-cam_pam.at<float>(0,2))/cam_pam.at<float>(0,0);//(x-x_c)/p_x
    y_out = (y_in-cam_pam.at<float>(1,2))/cam_pam.at<float>(1,1);//(y-y_c/)p_y
}
