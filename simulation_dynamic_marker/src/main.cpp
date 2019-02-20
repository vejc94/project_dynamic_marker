//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/cvstd.hpp>


//ViSP
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/core/vpPixelMeterConversion.h>
//project
#include <iostream>
#include <c++/5/cmath>
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
        target_=target;
        double width = target.getWidth()*0.00018;
        double height = target.getHeight()*0.00018;

        //        //Top left          Top right           Bottom right       Bottom left
        X[0][0] = -width/2;   X[1][0] = width/2;    X[2][0] = width/2;   X[3][0] = -width/2;
        X[0][1] = -height/2;  X[1][1] = -height/2;  X[2][1] = height/2;  X[3][1] =  height/2;
        X[0][2] =  0;         X[1][2] =  0;         X[2][2] = 0;         X[3][2] =  0;

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

//noise adding function
void addNoise(cv::Mat img_src, cv::Mat &img_dst, double stddev);
void setSPNoise(cv::Mat img_src, cv::Mat &img_dst,double p);
//detection of the ellipses
std::vector<cv::RotatedRect> centerDetector(vpImage<vpRGBa> &I);

//error calculation of the dynamic marker
double projectionError(std::vector<cv::RotatedRect>& list_ell, vpCameraParameters cam, int packing, cv::Vec3d &rvec, cv::Vec3d &tvec);

//simulation of the aruco marker and error calculation
void arucoSimulation();
double arucoBoardPoseError(std::vector<std::vector<cv::Point2f>> markerCorners, std::vector<int> markerIds, cv::Ptr<cv::aruco::GridBoard> board, cv::Mat cam_pam, cv::Mat distCoeff, cv::Vec3d &rvec, cv::Vec3d &tvec);
double arucoPoseError(cv::Mat &distCoeff, std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs, cv::Mat &cam_pam, double marker_length, std::vector<std::vector<cv::Point2f>> markerCorners);

//for sorting the list of the detected ellipses and the created ellipses
bool contourAreaComparator(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2);
bool centerListComparator1(cv::Point point1, cv::Point point2);
bool centerListComparator2(cv::Point point1, cv::Point point2);

//for writing the error
void writeError(std::string filename, double error_z, vpHomogeneousMatrix wMc);

//transform from image to camera (canonical) coordinates
void transform(double x_in, double y_in, double& x_out, double& y_out, vpCameraParameters cam_pam);

//to estimate the pose with conics
double PoseConics(std::vector<cv::RotatedRect> det_ell, vpCameraParameters cam_pam, cv::Vec3d &rvec, cv::Vec3d &tvec, double real_radius);

//error calculation translation vector
double translationError(cv::Vec3d t_est, vpHomogeneousMatrix &wMc);

//error calculation rotation vector
double rotationError(cv::Vec3d r_est, vpHomogeneousMatrix &wMc);

//tranform coordinate frames
void transformCoordinatesArucoBoard(cv::Vec3d &t_w, cv::Vec3d &r_w, cv::Ptr<cv::aruco::GridBoard> board, vpHomogeneousMatrix &wMc, vpHomogeneousMatrix &wMc2);
void transformCoordinatesArucoMarker(cv::Vec3d &t_w, cv::Vec3d &r_w, vpHomogeneousMatrix &wMc, vpHomogeneousMatrix &wMc2);
void drawAxis(cv::Mat &img, cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &cam_pam, cv::Mat &dist_coeff);

//Aruco Parameters
std::vector<int> markerIds;
double single_marker_length = 1440*0.00018;

///parameters for aruco grid board 32 corners = 4x2 markers
int markers_x = 4; //Numbers of markers in X direction
int markers_y= 2; //Numbers of markers in Y direction
double markers_length = 410*0.00018; //markers side lenght in meter (number of pixels * pixel pitch)
double markers_gap = 50*0.00018; // Separation between two consecutive markers in the grid in meter (number of pixels * pixel pitch)

//parameter for marker detection
std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(10));

//global parameters
int mon_rows = 1080;
int mon_cols = 1920;
int gap = 20;

double error_T;
double error_R;
std::string filename_R = "pruebaR.txt";//"error_6c_R.txt";
std::string filename_T = "pruebaT.txt";//"error_6c_T.txt";


//initial position of the camera
double x, y, z;
//roll, pitch, yaw
double phi, theta, psi;
//speed in z
double v_z;

//For pose estimation
cv::Mat distCoeff;
std::vector<cv::Vec3d> rvecs, tvecs;
cv::Vec3d rvec, tvec, tvec_last;

double diff;
double Radius;

int main()
{

    //Read initial Parameters
    std::cout << "choose the initial translation vector [x, y, z] of the camera in meter:" << std::endl;
    std::cin >> x >> y >> z;
    if(z < 0.) {
        std::cout << "invalid z value" << std::endl;
        return 0;
    }
    std::cout << "choose the initial rotation vector [phi, theta, psi] of the camera in degree:" << std::endl;
    std::cin >> phi >> theta >> psi;
    if(abs(psi)>90 ||(abs(theta)>90)||(abs(phi)>90)){
        std::cout << "invalid angles" << std::endl;
        return 0;
    }
    std::cout << "choose velocity in z:" << std::endl;
    std::cin >> v_z;

    //Simulation with aruco marker
    //arucoSimulation();return 0;

    ///create initializing image
    double init_radius = 200; //radius in pixels
    cv::Mat image_cv = cv::Mat(1080,1920,CV_8UC3,cv::Scalar(0, 0, 0)); //BGR Channels
    cv::Mat image_rgb;///RGBa needed for visp Images. visp doesn't support vpImage<BGR> constructors!
    cv::circle(image_cv, cv::Point(image_cv.cols/2, image_cv.rows/2), init_radius, cv::Scalar(255, 255, 255),-1);//circle draws with BGR Channels


    //cv::cvtColor(image_rgb,image_rgb,CV_BGR2RGBA);
    cv::cvtColor(image_cv, image_rgb, CV_BGR2RGB);

    vpImage<vpRGBa> Iimage;//image displayed in the monitor
    vpImage<vpRGBa> Icamera(1024,1280,255); //image projected in the camera
    vpImageConvert::convert(image_rgb,Iimage);

    /// Camera Parameters
    /// 1. p_x = ratio between focal length 'f=5mm' and pixel lenght 'l_x=4,8um'.
    /// 2. p_y = ratio between focal length 'f=5mm' and pixel lenght 'l_y=4,8um'.
    /// 3. & 4. u_0 & v_0 -> coordinates of the principal point

    vpCameraParameters cam(1083, 1083, Icamera.getWidth()/2, Icamera.getHeight()/2);
    cv::Mat cam_cv = (cv::Mat1d(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1); //camera matrix opencv

    vpHomogeneousMatrix cMw(x, y, z, vpMath::rad(phi), vpMath::rad(theta), vpMath::rad(psi)); //  camera coordinates to world(image) coordinates

    vpVirtualGrabber g(Iimage, cam); // Initialize image simulator
    vpHomogeneousMatrix wMc;// world to camera matrix. world coordinates are image coordinates
    wMc=cMw.inverse();

    vpSimulatorCamera robot; //instance of the free flying camera
    robot.setPosition(wMc);
    robot.setSamplingTime(0.006);// Modify the default sampling time to 0.006 second
    robot.setMaxTranslationVelocity(2.);
    robot.setMaxRotationVelocity(vpMath::rad(90));
    vpColVector v(6);

    v[2]= v_z;// m/s

    std::vector<cv::RotatedRect> det_ellipses;

    ///varying size marker
    circleController controller(520,5);

    ///fixed marker
    //circleController controller(110,110);

    //        tvec_last(2)=z;//z;
    //        tvec(2)=z;

    bool loop = true;
    int i=0;
    while(i<39){
        for(double j = 0.; (z - j) > 0.3 ; j = j+0.015){

            //1a. Posicion de mi camara real CON velocidad
            //                    robot.setVelocity(vpRobot::CAMERA_FRAME,v);
            //                    robot.getPosition(wMc);
            //                    cMw=wMc.inverse();

            ///1b. posicion de mi monitor a mi camara real SIN velocidad
            vpHomogeneousMatrix  cMw_new = vpHomogeneousMatrix(x, y, z-j, vpMath::rad(phi), vpMath::rad(theta), vpMath::rad(psi));
            robot.setPosition(cMw_new.inverse());
            wMc = cMw_new.inverse();
            cMw=wMc.inverse();
            cMw_new.~vpHomogeneousMatrix();//borrando la matriz de la nueva posicion

            ///2. Calcular el radio de los circulos que voy a mostrar en el monitor
            Radius = controller.calculate(cMw, 50);//r_soll = 50 pixels

            ///3. Dibujar los circulos en el monitor dependiendo de lo que mida el radio
            /// y durante ese proceso se preparan las listas de las posiciones de los
            /// circulos reales para la estimacion de pose.
            if(Radius>192){
                //pose estimation with 6 circles
                //squarePacking(image_cv,270);//Radius = 270 to display 6 circles

                //pose estimation with conics
                Radius = controller.calculate(cMw, 520);
                //squarePacking(image_cv, Radius);
                hexagonalPackingContinous(image_cv, Radius);
            }
            else {
                //hexagonalPacking(image_cv, Radius);
                hexagonalPackingContinous(image_cv, Radius);
            }

            ///4. Sacar una imagen del monitor sabiendo la posicion de mi camara
            /// y detectar los circulos.
            //cv::cvtColor(image_cv,image_rgb,CV_BGR2RGBA);
            addNoise(image_cv,image_rgb,20);
            vpImageConvert::convert(image_rgb, Iimage);
            g.acquire(Icamera, cMw, Iimage);
            det_ellipses = centerDetector(Icamera);//detect center of the ellipse in the camera image

            ///5. revisar que calcule alguna ellipse y si es verdad entonces estimar pose!
            /// si no, volver a la loop
            if(det_ellipses.empty()){
                continue;
            }

            if(Radius>192){
                //pose estimation with 6 circles
                //projectionError(det_ellipses,cam,0, rvec, tvec);

                //pose estimation with conics
                PoseConics(det_ellipses,cam, rvec, tvec, Radius);
            }
            else {
                //pose estimation with 4 or more circles
                projectionError(det_ellipses,cam,1,rvec,tvec);
            }

            ///6. Calcular y guardar el error de la pose
            error_T = translationError(tvec, wMc);
            error_R = rotationError(rvec, wMc);
            writeError(filename_T,error_T,wMc);
            writeError(filename_R,error_R,wMc);

            ///7. Mostrarla imagen de la camara
            cv::Mat image_cam;
            vpImageConvert::convert(Icamera,image_cam);
            cv::cvtColor(image_cam,image_cam,CV_RGBA2BGR);
            drawAxis(image_cam, rvec, tvec, cam_cv, distCoeff);
            cv::imshow("image de la camara", image_cam);
            cv::waitKey(0);
        }
        if(cMw.getTranslationVector()[2]<0.3) loop = false;
        i++;
    }

    return 0;

}


//image processing to detect the ellipse and find the middle point
std::vector<cv::RotatedRect> centerDetector(vpImage<vpRGBa> &I){
    cv::Mat image;

    //convert visp-format(RGBa) image to opencv-format(RGBA)
    vpImageConvert::convert(I, image);

    cv::cvtColor(image,image,CV_RGBA2BGR);
    //cv::Mat contour = cv::Mat::zeros(image.rows, image.cols, image.type());
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
//                cv::drawContours(contour, contours, i, cv::Scalar(255,0,0), 3);
                det_ellipses.push_back(det_ellipse);
            }


        }
    }
//        cv::imshow("conturas", contour);
//        cv::waitKey(0);
    return det_ellipses;
}

double projectionError(std::vector<cv::RotatedRect>& list_ell, vpCameraParameters cam, int packing, cv::Vec3d &rvec, cv::Vec3d &tvec){

    if(list_ell.empty() && (getCentersSQ().empty()||getCentersHX().empty())) return NAN;

    std::vector<cv::Point> list_centers;
    if(packing==0) list_centers = getCentersSQ();
    else if(packing == 1) list_centers = getCentersHX();
    else{std::cout << "invalid type of packing" << std::endl;}

    if(list_ell.size()!=list_centers.size()){
        std::cout << "number of detected ellipses: " <<list_ell.size() << std::endl;
        std::cout << "number of ellipses: "<<list_centers.size()<<std::endl;
        return NAN;
    }
    std::vector<cv::Point2f> list_det_centers;
    //    std::vector<double> list_errors;

    for(uint i = 0; i < list_ell.size(); i++){
        list_det_centers.push_back(list_ell.at(i).center);
    }
    std::sort(list_centers.begin(),list_centers.end(), centerListComparator1); // sort the real centers
    std::sort(list_det_centers.begin(),list_det_centers.end(),centerListComparator2);// sort the detected centers

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

    //errors of intrinsic parameters
    double delta_p = 2;
    double delta_u0 = 3;
    double delta_v0 = 1.5;

    //for solving solvepnp
    cv::Mat cam_cv = (cv::Mat1d(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1); //camera matrix opencv
    cam_cv.at<double>(0,0)+= delta_p;
    cam_cv.at<double>(1,1)+= delta_p;
    cam_cv.at<double>(0,2)+= delta_u0;
    cam_cv.at<double>(1,2)+= delta_v0;

    cv::Mat Rmat; // rotation, translation vector and rotation matrix
    if( (list_det_centers.size() > 3)) {
        //std::cout << "i am here" << std::endl;
        cv::solvePnP(objectPoints,imagePoints,cam_cv, std::vector<int>(0), rvec, tvec);
        cv::Rodrigues(rvec, Rmat);
    }

    return NAN;
}

void arucoSimulation(){
    //creating aruco marker image
    cv::Mat marker_image = cv::Mat::zeros(1440, 1440, CV_8UC3);//aruco draws marker in grayscale
    cv::Mat marker_image_rgb;///RGBa needed for visp Images. visp doesn't support vpImage<BGR> constructors!

    ///image for 1 marker
    //cv::aruco::drawMarker(dictionary, 23, 1440, marker_image, 1);

    ///create board
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markers_x, markers_y, markers_length, markers_gap, dictionary);
    //image for marker board
    board->draw(cv::Size(1920,1080), marker_image, 0 , 1);

    cv::cvtColor(marker_image, marker_image, CV_GRAY2BGR);
    ///convert from bgr to rgb for visp
    cv::cvtColor(marker_image, marker_image_rgb, CV_BGR2RGB);

    vpImage<vpRGBa> Iimage;//image displayed in the monitor
    vpImage<vpRGBa> Icamera(1024,1280,255); //image projected in the camera
    vpImageConvert::convert(marker_image_rgb,Iimage);

    /// Camera Parameters
    /// 1. p_x = ratio between focal length 'f=5mm' and pixel lenght 'l_x=4,8um'.
    /// 2. p_y = ratio between focal length 'f=5mm' and pixel lenght 'l_y=4,8um'.
    /// 3. & 4. u_0 & v_0 -> coordinates of the principal point

    vpCameraParameters cam(1083, 1083, Icamera.getWidth()/2, Icamera.getHeight()/2);
    cv::Mat cam_cv = (cv::Mat1d(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1); //camera matrix opencv

    vpHomogeneousMatrix cMw(x, y, z, vpMath::rad(phi), vpMath::rad(theta), vpMath::rad(psi)); //  camera coordinates to world(image) coordinates

    vpVirtualGrabber g(Iimage, cam); // Initialize image simulator
    g.acquire(Icamera,cMw,Iimage);//acquire image projection with camera position wrt world coordinates

    cv::Mat icamera_cv;///needed to convert the rgb visp image to bgr in order to display the rendered image in the right way.
    vpImageConvert::convert(Icamera,icamera_cv);
    cv::cvtColor(icamera_cv,icamera_cv,CV_RGBA2BGR);

    vpHomogeneousMatrix wMc;// world to camera matrix. world coordinates are image coordinates
    vpSimulatorCamera robot; //instance of the free flying camera markerIds
    robot.setSamplingTime(0.006);// Modify the default sampling time to 0.006 second
    robot.setMaxTranslationVelocity(2.);
    robot.setMaxRotationVelocity(vpMath::rad(90));
    vpColVector v(6);
    //for(int i = 0; i < 4; i++ ){
    wMc=cMw.inverse();
    robot.setPosition(wMc);


    v[2]=  v_z; // vz = 0.5 m/s

    bool loop = true;
    int i = 0;
    while(loop){

        for(double j = 0.; (z - j) > 0.3 ; j = j+0.1){
//        robot.setVelocity(vpRobot::CAMERA_FRAME,v);
//        robot.getPosition(wMc);
//        cMw=wMc.inverse();

        ///1. Posicionar la camara
            vpHomogeneousMatrix  cMw_new = vpHomogeneousMatrix(x, y, z-j, vpMath::rad(phi), vpMath::rad(theta), vpMath::rad(psi));
            robot.setPosition(cMw_new.inverse());
            wMc = cMw_new.inverse();
            cMw=wMc.inverse();
            cMw_new.~vpHomogeneousMatrix();//borrando la matriz de la nueva posicion

        ///2. Sacar nueva imagen del marcador desde la camara
        addNoise(marker_image, marker_image_rgb, 20);
        vpImageConvert::convert(marker_image_rgb,Iimage);
        g.acquire(Icamera, cMw, Iimage);
        vpImageConvert::convert(Icamera,icamera_cv);
        cv::cvtColor(icamera_cv,icamera_cv,CV_RGBA2BGR);

        ///3. detection of the aruco marker image in the camera
        cv::aruco::detectMarkers(icamera_cv, dictionary, markerCorners, markerIds, parameters, rejectedCandidates,cam_cv);

        ///draw borders of the detected camera
        cv::aruco::drawDetectedMarkers(icamera_cv, markerCorners, markerIds);

        ///4a.estimate pose with one marker
        //arucoPoseError(distCoeff, rvecs, tvecs, cam_cv, single_marker_length, markerCorners);

        ///4b. estimate pose board
        arucoBoardPoseError(markerCorners, markerIds, board, cam_cv, distCoeff, rvec, tvec);

        ///5. Transform world and aruco frames board
        cv::Vec3d r_w, t_w;
        vpHomogeneousMatrix wMc_2;
        //transformCoordinatesArucoBoard(t_w, r_w, board, wMc, wMc_2);
        //transformCoordinatesArucoMarker(t_w, r_w, wMc, wMc_2);

        //draw axis
        cv::aruco::drawAxis(icamera_cv, cam_cv, distCoeff, rvec, tvec, 0.1);

        ///calcular y guardar los errores
        error_R= rotationError(rvec,wMc);
        error_T= translationError(tvec,wMc);
        writeError(filename_T,error_T,wMc);
        writeError(filename_R,error_R,wMc);

        ///mostrar imagen de la camara
        cv::imshow("image de la camara", icamera_cv);
        cv::waitKey(0);

        if(wMc.inverse().getTranslationVector()[2] < 0.3)
            loop = false;
        }
        i++;

    }
}

double arucoPoseError(cv::Mat &distCoeff, std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs, cv::Mat &cam_pam, double marker_length, std::vector<std::vector<cv::Point2f>> markerCorners){

    //errors of intrinsic parameters
    double delta_p = 2;
    double delta_u0 = 3;
    double delta_v0 = 1.5;
    static bool camera_error=false;
    if(!camera_error){
        cam_pam.at<double>(0,0)+= delta_p;
        cam_pam.at<double>(1,1)+= delta_p;
        cam_pam.at<double>(0,2)+= delta_u0;
        cam_pam.at<double>(1,2)+= delta_v0;
        camera_error=true;
    }




    cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length, cam_pam, distCoeff, rvecs, tvecs);
    if(tvecs.empty()) return NAN;

    //transform from world to aruco coordinate frame
    cv::Mat Rmat =  (cv::Mat_<double>(3,3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);
    cv::Vec3d r_w2a, t_w2a;
    cv::Mat tres, rres;
    cv::Rodrigues(Rmat, r_w2a);
    cv::composeRT( r_w2a, t_w2a, rvecs.at(0), tvecs.at(0), rres, tres);

    tvec[0] = tres.at<double>(0);
    tvec[1] = tres.at<double>(1);
    tvec[2] = tres.at<double>(2);

    rvec[0] = rres.at<double>(0);
    rvec[1] = rres.at<double>(1);
    rvec[2] = rres.at<double>(2);
    //cv::Rodrigues(rvecs.at(0),Rmat);

    return NAN;
}

double arucoBoardPoseError(std::vector<std::vector<cv::Point2f>> markerCorners, std::vector<int> markerIds, cv::Ptr<cv::aruco::GridBoard> board, cv::Mat cam_pam, cv::Mat distCoeff, cv::Vec3d &rvec, cv::Vec3d &tvec){
    //errors of intrinsic parameters
    double delta_p = 2;
    double delta_u0 = 3;
    double delta_v0 = 1.5;
    static bool camera_error=false;
    if(!camera_error){
        cam_pam.at<double>(0,0)+= delta_p;
        cam_pam.at<double>(1,1)+= delta_p;
        cam_pam.at<double>(0,2)+= delta_u0;
        cam_pam.at<double>(1,2)+= delta_v0;
        camera_error=true;
    }
    int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cam_pam, distCoeff, rvec, tvec);

    if(!valid){
        return NAN;
    }
    //transform from world to aruco coordinate frame
    cv::Mat Rmat =  (cv::Mat_<double>(3,3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);
    cv::Vec3d t_w2a(2*markers_length + 1.5*markers_gap, markers_length + markers_gap / 2, 0);
    cv::Vec3d r_w2a;
    cv::Mat tres, rres;
    cv::Rodrigues(Rmat, r_w2a);
    cv::composeRT( r_w2a, t_w2a, rvec, tvec, rres, tres);

    tvec[0] = tres.at<double>(0);
    tvec[1] = tres.at<double>(1);
    tvec[2] = tres.at<double>(2);

    rvec[0] = rres.at<double>(0);
    rvec[1] = rres.at<double>(1);
    rvec[2] = rres.at<double>(2);

    return 0;

}

void addNoise(cv::Mat img_src, cv::Mat &img_dst, double stddev){
    cv::theRNG().state = cv::getTickCount();
    //cv::setRNGSeed(cv::getTickCount());
    cv::Mat img_src16;
    if(img_src.channels()==1){
        cv::cvtColor(img_src, img_src16, CV_GRAY2BGR);
        img_src16.convertTo(img_src16, CV_16SC3);
    }
    else
        img_src.convertTo(img_src16, CV_16SC3);

    cv::Mat noise_img = cv::Mat(img_src.size(),CV_16SC3);
    randn(noise_img, cv::Scalar::all(0.0), cv::Scalar::all(stddev));

    addWeighted(img_src16, 1.0, noise_img, 1.0, 0.0, img_src16);
    img_src16.convertTo(img_dst,img_src.type());

}


//sort contours in descending order of area.
bool contourAreaComparator(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2){
    double i = fabs( cv::contourArea(contour1));
    double j = fabs( cv::contourArea(contour2));
    return ( i > j );
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


void writeError(std::string filename, double error_z, vpHomogeneousMatrix wMc){

    std::ofstream error_file;
    error_file.open(filename,std::ios_base::app);
    error_file << error_z<< "," <<wMc.inverse().getTranslationVector()[2]<<std::endl;
    error_file.close();
}

void transform(double x_in, double y_in, double& x_out, double& y_out, vpCameraParameters cam_pam){
    x_out = (x_in-cam_pam.get_u0())/cam_pam.get_px();
    y_out = (y_in-cam_pam.get_v0())/cam_pam.get_py();
}

double PoseConics(std::vector<cv::RotatedRect> det_ell, vpCameraParameters cam_pam, cv::Vec3d &rvec, cv::Vec3d &tvec, double real_radius){
    double x,y,x1,x2,y1,y2,sx1,sx2,sy1,sy2,major,minor,v0,v1;

    cv::RotatedRect circle = det_ell.at(1);

    //transform the center
    transform(circle.center.x, circle.center.y, x,y, cam_pam);

    //circle parameters v0,1 and m0,1 in image coordinates
    double circle_m0 = circle.size.width*0.25;
    double circle_m1 = circle.size.height*0.25;
    double circle_v0 = cos(circle.angle/180.0 * M_PI);
    double circle_v1 = sin(circle.angle/180.0 * M_PI);

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
    double circle_diameter = (real_radius/1.5 - gap)*2*0.00018;//converting from pixels into meters the radius of the circle shown in the display

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

        std::cout <<"estimated r: "<< rvec << std::endl;
    //    std::cout << "real r: " << wMc.getThetaUVector()<< std::endl;

    return 0;
}

double translationError(cv::Vec3d t_est, vpHomogeneousMatrix &wMc){
    //error = ||T_est - T_real||_2 / ||T_real||_2 *100%

    vpTranslationVector t_real = wMc.inverse().getTranslationVector();
    double x = pow(t_real[0]-t_est(0),2);
    double y = pow(t_real[1]-t_est(1),2);
    double z = pow(t_real[2]-t_est(2),2);

    double error = sqrt(x+y+z)/sqrt(t_real[0]*t_real[0] + t_real[1]*t_real[1] + t_real[2]*t_real[2]);

//    std::cout << "real t: " << t_real << std::endl;
//    std::cout << "estimated t: " << t_est << std::endl;

    std::cout <<"error = "<< error*100 << std::endl;
    return error*100;
}


double rotationError(cv::Vec3d r_est, vpHomogeneousMatrix &wMc){
    vpMatrix R_real = (vpMatrix)wMc.inverse().getRotationMatrix();
    R_real = R_real.transpose();
    cv::Mat R_est = cv::Mat::zeros(3,3,CV_64F);
    cv::Rodrigues(r_est, R_est);
    cv::Mat R_res = cv::Mat::zeros(3,3, CV_64F);


//    std::cout << "r estimated: " << r_est<<std::endl;
//    std::cout << "r_real: " << wMc.inverse().getThetaUVector() << std::endl;

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                R_res.at<double>(i,j) += R_est.at<double>(i,k)* R_real[k][j];
            }
        }
    }
    cv::Vec3d r_res;

    cv::Rodrigues(R_res, r_res);
    double angle = cv::norm(r_res,cv::NORM_L2);//rad

    std::cout <<"angle = " <<angle*180/M_PI << std::endl;
    return angle*180/M_PI;

}

void transformCoordinatesArucoBoard(cv::Vec3d &t_w, cv::Vec3d &r_w, cv::Ptr<cv::aruco::GridBoard> board, vpHomogeneousMatrix &wMc, vpHomogeneousMatrix &wMc2){
    //marker length parameters
    double length =  board->getMarkerLength();
    double gap = board->getMarkerSeparation();

    //matrix from aruco to world frame
    vpHomogeneousMatrix wMa = vpHomogeneousMatrix(2*length+1.5*gap, length+gap/2, 0, vpMath::rad(180),vpMath::rad(0),vpMath::rad(0));

    wMc2 = wMa*wMc;
    vpTranslationVector t2 = wMc2.getTranslationVector();
    vpRotationVector r2 = wMc2.getThetaUVector();

    t_w(0) = t2[0]; t_w(1) = t2[1]; t_w(2) = t2[2];
    r_w(0) = r2[0];r_w(2) = r2[2];r_w(2) = r2[2];

}

void transformCoordinatesArucoMarker(cv::Vec3d &t_w, cv::Vec3d &r_w, vpHomogeneousMatrix &wMc, vpHomogeneousMatrix &wMc2){
    //matrix from aruco to world frame
    vpHomogeneousMatrix wMa = vpHomogeneousMatrix(0, 0, 0,vpMath::rad(180),vpMath::rad(0),vpMath::rad(0));

    wMc2 = wMa*wMc;
    vpTranslationVector t2 = wMc2.getTranslationVector();
    vpRotationVector r2 = wMc2.getThetaUVector();

    t_w(0) = t2[0]; t_w(1) = t2[1]; t_w(2) = t2[2];
    r_w(0) = r2[0];r_w(2) = r2[2];r_w(2) = r2[2];
}
void drawAxis(cv::Mat &img, cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &cam_pam, cv::Mat &dist_coeff){
    //axis points
    std::vector<cv::Point3f> points;
    points.push_back(cv::Point3f(0.1, 0, 0));
    points.push_back(cv::Point3f(0, 0.1, 0));
    points.push_back(cv::Point3f(0, 0, 0.1));
    points.push_back(cv::Point3f(0, 0, 0));
    //projected points list
    std::vector<cv::Point2f> proj_points;
    //project points in the image plane
    cv::projectPoints(points, rvec, tvec, cam_pam, dist_coeff, proj_points);
    //draw lines
    cv::line(img, proj_points.at(3), proj_points.at(0), cv::Scalar(0, 0, 255), 4);
    cv::line(img, proj_points.at(3), proj_points.at(1), cv::Scalar(0, 255, 0), 4);
    cv::line(img, proj_points.at(3), proj_points.at(2), cv::Scalar(255, 0, 0), 4);
}
