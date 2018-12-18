//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/cvstd.hpp>
//ViSP
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp3/robot/vpSimulatorCamera.h>

//project
#include <iostream>
#include <math.h>
#include "circlecontroller.h"
#include "circlepacking.h"

class vpVirtualGrabber
{
public:
    vpVirtualGrabber(vpImage<vpRGBa> target, const vpCameraParameters &cam) : target_(), cam_()
    {
        ///initialize the 3D plane coordinates of each corner in the plane Z=0.
        /// plane coordinates are world coordinates
        ///Display size 345mm X 194mm
        for (int i = 0; i < 4; i++)
            X[i].resize(3);
        // Parameters for dynamic marker
        //Top left          Top right           Bottom right       Bottom left
//        X[0][0] = -0.1725;  X[1][0] =  0.1725;  X[2][0] = 0.1725;  X[3][0] = -0.1725;
//        X[0][1] = -0.097;   X[1][1] = -0.097;   X[2][1] = 0.097;   X[3][1] =  0.097;
//        X[0][2] =  0;       X[1][2] =  0;       X[2][2] = 0;       X[3][2] =  0;
        //Parameters for aruco marker
//        //Top left          Top right           Bottom right       Bottom left
        X[0][0] = -0.1296;  X[1][0] =  0.1296;  X[2][0] = 0.1296;  X[3][0] = -0.1296;
        X[0][1] = -0.1296;  X[1][1] = -0.1296;  X[2][1] = 0.1296;  X[3][1] =  0.1296;
        X[0][2] =  0;       X[1][2] =  0;       X[2][2] = 0;       X[3][2] =  0;


        target_=target;

        // Initialize the image simulator
        cam_ = cam;
        sim_.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
        sim_.init(target, X);

    }
    void acquire(vpImage<vpRGBa> &Icamera, const vpHomogeneousMatrix &cMw, vpImage<vpRGBa> &Iimage)
    {

        sim_.setCleanPreviousImage(true, vpColor::gray);
        sim_.init(Iimage,X);
        sim_.setCameraPosition(cMw);
        sim_.getImage(Icamera, cam_);
    }
private:
    vpColVector X[4]; // 3D coordinates of the target corners
    vpImage<vpRGBa> target_;
    vpCameraParameters cam_;
    vpImageSimulator sim_ ;
};


//detection of the ellipses
std::vector<cv::RotatedRect> centerDetector(vpImage<vpRGBa> &I);

//error calculation of the dynamic marker
void projectionError(std::vector<cv::RotatedRect>& list_ell, vpHomogeneousMatrix& wMc, vpCameraParameters cam, int packing);

//simulation of the aruco marker and error calculation
void arucoSimulation();
void arucoPoseError(cv::Mat distCoeff, std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs, cv::Mat cam_pam, float marker_length, std::vector<std::vector<cv::Point2f>> markerCorners, vpHomogeneousMatrix& wMc);

//for sorting the list of the detected ellipses and the created ellipses
bool contourAreaComparator(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2);
bool centerListComparator1(cv::Point point1, cv::Point point2);
bool centerListComparator2(cv::Point point1, cv::Point point2);

int main()
{
    arucoSimulation();

    ///create initializing image
    double init_radius = 200; //radius in pixels
    cv::Mat image_cv = cv::Mat(1080,1920,CV_8UC3,cv::Scalar(0,0,0)); //BGR Channels
    cv::Mat image_rgb;///RGBa needed for visp Images. visp doesn't support vpImage<BGR> constructors!
    cv::circle(image_cv, cv::Point(image_cv.cols/2, image_cv.rows/2), init_radius, cv::Scalar(0,0,255),-1);//circle draws with BGR Channels
    cv::cvtColor(image_cv,image_rgb,CV_BGR2RGBA);

    vpImage<vpRGBa> Iimage;//image displayed in the monitor
    vpImage<vpRGBa> Icamera(1024,1280,255); //image projected in the camera
    vpImageConvert::convert(image_rgb,Iimage);

    /// Camera Parameters
    /// 1. p_x = ratio between focal length 'f=5mm' and pixel lenght 'l_x=4,8um'.
    /// 2. p_y = ratio between focal length 'f=5mm' and pixel lenght 'l_y=4,8um'.
    /// 3. & 4. u_0 & v_0 -> coordinates of the principal point

    vpCameraParameters cam(1083, 1083, Icamera.getWidth()/2, Icamera.getHeight()/2);

    vpHomogeneousMatrix cMw(-0.0, 0, 0.8, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0)); //  camera coordinates to world(image) coordinates

    vpVirtualGrabber g(Iimage, cam); // Initialize image simulator
    g.acquire(Icamera,cMw,Iimage);//acquire image projection with camera position wrt world coordinates

    cv::Mat icamera_cv;///needed to convert the rgb visp image to bgr in order to display the rendered image in the right way.
    vpImageConvert::convert(Icamera,icamera_cv);
    cv::cvtColor(icamera_cv,icamera_cv,CV_RGBA2BGR);


    vpHomogeneousMatrix wMc;// world to camera matrix. world coordinates are image coordinates
    wMc=cMw.inverse();

    vpSimulatorCamera robot; //instance of the free flying camera
    robot.setPosition(wMc);
    robot.setSamplingTime(0.006);// Modify the default sampling time to 0.006 second
    robot.setMaxTranslationVelocity(2.);
    robot.setMaxRotationVelocity(vpMath::rad(90));
    vpColVector v(6);

    v[2]=  0.5; // vz = 0.5 m/s

    std::vector<cv::RotatedRect> det_ellipses;
    circleController controller(192,5);

    for(int i=0;;i++){
        robot.setVelocity(vpRobot::CAMERA_FRAME,v);


        robot.getPosition(wMc);
        cMw=wMc.inverse();

        det_ellipses = centerDetector(Icamera);//detect center of the ellipse in the camera image

        if(det_ellipses.empty()){
            robot.setVelocity(vpRobot::CAMERA_FRAME,v);
            robot.getPosition(wMc);
            cMw=wMc.inverse();
            g.acquire(Icamera, cMw, Iimage);
            continue;
        }

        double Radius = controller.calculate(cMw, 50);//r_soll = 80 pixels

        //if(Radius>192){
          //  squarePackingContinous(image_cv,Radius);
            //projectionError(det_ellipses,wMc,cam,0);
        //}
        //else {
            hexagonalPackingContinous(image_cv, Radius);
            projectionError(det_ellipses,wMc,cam,1);
        //}



        cv::cvtColor(image_cv,image_rgb,CV_BGR2RGBA);

        cv::namedWindow("monitor",cv::WINDOW_NORMAL);
        cv::imshow("monitor", image_cv);

        cv::waitKey(1);


        vpImageConvert::convert(image_rgb, Iimage);
        g.acquire(Icamera, cMw, Iimage);
        vpImageConvert::convert(Icamera,icamera_cv);
        cv::cvtColor(icamera_cv,icamera_cv,CV_RGBA2BGR);

        cv::imshow("image de la camara", icamera_cv);
        cv::waitKey(1);

    }


    return 0;

}


//image processing to detect the ellipse and find the middle point
std::vector<cv::RotatedRect> centerDetector(vpImage<vpRGBa> &I){

    cv::Mat image;

    //convert visp-format(RGBa) image to opencv-format(RGBA)
    vpImageConvert::convert(I, image);
    cv::cvtColor(image,image,CV_RGBA2BGR);
    cv::cvtColor(image,image,CV_BGR2GRAY);

    //image.convertTo(image, CV_8UC1); // gray scaled one channel image
    cv::Mat bin_image; // binarized image 0 - 255
    cv::threshold(image, bin_image, 50, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours; // list of detected contours
    cv::findContours(bin_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), contourAreaComparator);//sort contours by contour area in descending order

    std::vector<cv::RotatedRect> det_ellipses; //List of detected ellipses
    // contour(0) is display contour. if the contour has less than 5 point it is not an ellipse

    //add all detected ellipses to the list
    for(uint i = 0; i < contours.size(); i++){

        if((contours.at(i).size() > 20 )){
            cv::RotatedRect det_ellipse = cv::fitEllipse(contours.at(i));
            double area_ell = 0.25*M_PI*det_ellipse.size.width*det_ellipse.size.height;
            double area_con = cv::contourArea(contours.at(i));
            double A_ratio = area_con/area_ell;
            if((A_ratio>0.90)&&(A_ratio<1.1)){
                det_ellipses.push_back(det_ellipse);
            }


        }
    }

    return det_ellipses;
}

void projectionError(std::vector<cv::RotatedRect>& list_ell,vpHomogeneousMatrix& wMc, vpCameraParameters cam, int packing){
    if(list_ell.empty() && (getCentersSQ().empty()||getCentersHX().empty())) return;

    std::vector<cv::Point> list_centers;
    if(packing==0) list_centers = getCentersSQ();
    else if(packing == 1) list_centers = getCentersHX();
    else{std::cout << "invalid type of packing" << std::endl;}

    if(list_ell.size()!=list_centers.size()) return;

    std::vector<cv::Point2f> list_det_centers;
    std::vector<float> list_errors;

    for(uint i = 0; i < list_ell.size(); i++){
        list_det_centers.push_back(list_ell.at(i).center);
    }
    std::sort(list_centers.begin(),list_centers.end(), centerListComparator1); // sort the real centers
    std::sort(list_det_centers.begin(),list_det_centers.end(),centerListComparator2);// sort the detected centers

    //saving errors
    vpMatrix M_proj(3,4);
    M_proj[0][0] = 1;
    M_proj[1][1] = 1;
    M_proj[2][2] = 1;
    ///vector image plane
    vpColVector center_cam(3);
    center_cam[2]=1;
    ///vector world coordinate frame where Z=0
    vpColVector center_mon(4);
    center_mon[2]=0;
    center_mon[3]=1;

    std::vector<cv::Point3f> objectPoints(list_centers.size());
    std::vector<cv::Point2f> imagePoints(list_det_centers.size());


    ///converting [u,v]_w into [X_w, Y_w, Z_w]
    for(uint i = 0; i < list_det_centers.size(); i++){
        objectPoints.at(i).x = (list_centers.at(i).x - 960)*0.00018;
        objectPoints.at(i).y = (list_centers.at(i).y - 540)*0.00018;
        objectPoints.at(i).z = 0;

        imagePoints.at(i).x = list_det_centers.at(i).x;
        imagePoints.at(i).y = list_det_centers.at(i).y;
    }


    //for solving solvepnp
    cv::Mat cam_cv = (cv::Mat1d(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1); //camera matrix opencv
    cv::Mat rvec, tvec, Rmat; // rotation, translation vector and rotation matrix
    if( (list_det_centers.size() > 3)) {
        cv::solvePnP(objectPoints,imagePoints,cam_cv, std::vector<int>(0), rvec, tvec);
        cv::Rodrigues(rvec,Rmat);
        std::cout << tvec.at<double>(2) - wMc.inverse().getTranslationVector()[2] << std::endl;
    }


    ///error calculation opencv function project()
    cv::Mat crw;
    cv::Mat cRw = (cv::Mat1d(3,3) << wMc.inverse()[0][0],wMc.inverse()[0][1],wMc.inverse()[0][2],wMc.inverse()[1][0],wMc.inverse()[1][1],wMc.inverse()[1][2],wMc.inverse()[2][0],wMc.inverse()[2][1],wMc.inverse()[2][2] );
    cv::Rodrigues(cRw, crw);

    std::vector<cv::Point2f> proj_points;
    cv::Mat ctw = (cv::Mat1d(3,1) << wMc.inverse()[0][3], wMc.inverse()[1][3], wMc.inverse()[2][3]);

    cv::projectPoints(objectPoints,crw,ctw,cam_cv,std::vector<int>(0),proj_points);

    float error = 0;
    for(uint i = 0; i < proj_points.size(); i++){
        float error_x = proj_points.at(i).x - imagePoints.at(i).x;
        float error_y = proj_points.at(i).y - imagePoints.at(i).y;
        error += error_x*error_x + error_y*error_y;
    }
    //std::cout << sqrt(error/proj_points.size()) << std::endl;


    ///error calculation for the center points with my method.
    for(uint i = 0; i < list_det_centers.size(); i++){
        ///error calculation opencv function project()


        /// (u_mon - u_0_mon)*px = x

        center_mon[0] = (list_centers.at(i).x - 960)*0.00018;
        center_mon[1] = (list_centers.at(i).y - 540)*0.00018;

        //[u, v, 1]^T = K*M*[ cRw | ctw ]*[X, Y, Z=0,1]
        center_cam = cam.get_K()*M_proj*(vpMatrix)wMc.inverse()*center_mon;

        float error_x = center_cam[0]/center_cam[2]-list_det_centers.at(i).x;
        float error_y = center_cam[1]/center_cam[2]-list_det_centers.at(i).y;
        float error = error_x*error_x + error_y*error_y;

        list_errors.push_back(error);
    }

    for(uint i = 0; i<list_errors.size(); i++){
        error+=list_errors.at(i);
    }
    //std::cout <<sqrt(error/list_errors.size())<< std::endl;
    //variables for file saving
    //    std::string filename = "errors1.yaml";
    //    cv::FileStorage fs(filename, cv::FileStorage::APPEND);
    //    fs << "projection error" << sqrt(error/list_errors.size());
    //    fs << "N" << (int)list_errors.size();
    //    fs.release();
}

void arucoSimulation(){
    //creating aruco marker image
    cv::Mat marker_image_cv;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(10));
    cv::aruco::drawMarker(dictionary, 23, 1440, marker_image_cv, 1);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Mat marker_image_rgb;///RGBa needed for visp Images. visp doesn't support vpImage<BGR> constructors!

    cv::cvtColor(marker_image_cv,marker_image_rgb,CV_GRAY2RGB);
    //for pose estimation
    cv::Mat distCoeff;
    std::vector<cv::Vec3d> rvecs, tvecs;
    float marker_length = 1440*0.00018;



    vpImage<vpRGBa> Iimage;//image displayed in the monitor
    vpImage<vpRGBa> Icamera(1024,1280,255); //image projected in the camera
    vpImageConvert::convert(marker_image_rgb,Iimage);

    /// Camera Parameters
    /// 1. p_x = ratio between focal length 'f=5mm' and pixel lenght 'l_x=4,8um'.
    /// 2. p_y = ratio between focal length 'f=5mm' and pixel lenght 'l_y=4,8um'.
    /// 3. & 4. u_0 & v_0 -> coordinates of the principal point

    vpCameraParameters cam(1083, 1083, Icamera.getWidth()/2, Icamera.getHeight()/2);
    cv::Mat cam_cv = (cv::Mat1d(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1); //camera matrix opencv

    vpHomogeneousMatrix cMw(-0.0, 0, 1, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0)); //  camera coordinates to world(image) coordinates

    vpVirtualGrabber g(Iimage, cam); // Initialize image simulator
    g.acquire(Icamera,cMw,Iimage);//acquire image projection with camera position wrt world coordinates

    cv::Mat icamera_cv;///needed to convert the rgb visp image to bgr in order to display the rendered image in the right way.
    vpImageConvert::convert(Icamera,icamera_cv);
    cv::cvtColor(icamera_cv,icamera_cv,CV_RGBA2BGR);


    vpHomogeneousMatrix wMc;// world to camera matrix. world coordinates are image coordinates
    wMc=cMw.inverse();

    vpSimulatorCamera robot; //instance of the free flying camera
    robot.setPosition(wMc);
    robot.setSamplingTime(0.006);// Modify the default sampling time to 0.006 second
    robot.setMaxTranslationVelocity(2.);
    robot.setMaxRotationVelocity(vpMath::rad(90));
    vpColVector v(6);

    v[2]=  0.5; // vz = 0.5 m/s


    for(int i=0;;i++){
        robot.setVelocity(vpRobot::CAMERA_FRAME,v);
        robot.getPosition(wMc);
        cMw=wMc.inverse();

        //detection of the aruco image in the camera
        cv::aruco::detectMarkers(icamera_cv, dictionary, markerCorners, markerIds, parameters, rejectedCandidates,cam_cv);
        cv::aruco::drawDetectedMarkers(icamera_cv, markerCorners, markerIds);

        //estimate pose
        arucoPoseError(distCoeff, rvecs, tvecs, cam_cv, marker_length, markerCorners, wMc);
        cv::imshow("image de la camara", icamera_cv);
        cv::waitKey(1);
        vpImageConvert::convert(icamera_cv, Icamera);

        g.acquire(Icamera, cMw, Iimage);
        vpImageConvert::convert(Icamera,icamera_cv);
    }
}

void arucoPoseError(cv::Mat distCoeff, std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs, cv::Mat cam_pam, float marker_length, std::vector<std::vector<cv::Point2f>> markerCorners, vpHomogeneousMatrix &wMc){
    //rotation matrix of the rotation vector
    cv::Mat Rmat;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length, cam_pam, distCoeff, rvecs, tvecs);
    cv::Vec3d tvec = tvecs[0];

    //cv::Rodrigues(rvecs.at(0),Rmat);

    //error of the z component of the translation vector
    std::cout << tvec[2] - wMc.inverse().getTranslationVector()[2] << std::endl;

}
//sort contours in descending order of area.
bool contourAreaComparator(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2){
    double i = fabs( cv::contourArea(contour1));
    double j = fabs( cv::contourArea(contour2));
    return ( i > j );
}

///real centers sorting function. in descending order
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
///detected centers sorting funtion. in descending order.
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
