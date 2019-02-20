//Own files
#include "center_detector.h"
#include "circlecontroller.h"
#include "circlepacking.h"
#include "pose_estimation.h"

//Camera files
#include <flycapture/FlyCapture2.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//Others
#include <iostream>
#include<ctime>

void writeFile(cv::Vec3d value, std::string filename);
void writeYaml(std::vector<cv::Vec3d> tvecs, std::vector<cv::Vec3d> rvecs);
void imageAnalyze();
using namespace FlyCapture2;

//desired projected radius on the camera.
double r_soll = 50;
//monitor parameters
int mon_rows = 1080;
int mon_cols = 1920;
double pixel_pitch = 0.000283;//monitor raul
//gap between circles and monitor
int gap = 20;

std::vector<cv::RotatedRect> det_ellipses;//list of ellipses in the image
cv::Mat cam = cv::Mat::zeros(3,3, CV_32F);//camera intrinsic parameters
cv::Mat distCoeff = cv::Mat::zeros(1,5, CV_32F);
cv::Vec3d tvec, rvec;//translation and rotation vectors

//
circleController controller(520,5);//varying size marker
//circleController controller(110,110);//fixed marker

//absolut path to file
std::string filename = "/home/victor94/project_dynamic_marker/image_grabbing/image_grabbing/camera_parameters.yaml";
std::string imagesfolder= "/home/victor94/project_dynamic_marker/image_grabbing/build-image_grabbing-Desktop-Debug/imagenes/";
std::string errors = "/home/victor94/project_dynamic_marker/image_grabbing/build-image_grabbing-Desktop-Debug/errors/";

///In order to be able to read the file it is necessary to add the type of data
/// the matrix has in the yaml file!! for examples "dt: f"
void readCameraParameters(){
    //File reader
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);

    //check if file is open
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }
    fs["camera_matrix"] >> cam ;
    fs["distortion_coefficients"] >> distCoeff;
    fs.release();

}

/// The main function takes images on real time. It asks which type of marker is going to be used and the distance between screen and camera.
int main()
{
    // With function imageAnalze() the images that were already (100x) taken are processed for pose estimation
    //imageAnalyze();
    //return 0;


    Error error;
    Camera camera;
    CameraInfo camInfo;

    // Connect the camera
    error = camera.Connect( 0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;
        return false;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;
        return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber << std::endl;

    error = camera.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;
        return false;
    }

    //reading camera instrinsic matrix
    readCameraParameters();

    ///Marker options
    char marker = 0;
    std::cout << "choose with what marker to estimate pose (a)ruco, (b)oard, (c)onic, 6(x)circles" << std::endl;
    std::cin >> marker;

    //input for distance
    double z;
    std::string imgdist;
    std::cout << "file x cm/ ? " << std::endl;
    std::cin >> imgdist;
    double Radius;
    if(marker == 'x' || marker == 'c')
    {
        std::cout << "distance z: " << std::endl;
        std::cin >> z;
        //calculate Radius of the displayed circles on the monitor
        Radius = controller.calculate(z, r_soll, cam);
        std::cout << "circle radius: " <<Radius << std::endl;
    }

    if(cam.empty())
    {
        std::cout << "Failed to read camera parameters" << std::endl;
        return 0;
    }

    // capture loop
    std::string image_file;
    int i = 0;
    while(i < 100)
   {
        // Get the image
        Image rawImage;
        //Error
        error = camera.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK )
        {
            std::cout << "capture error" << std::endl;
            return 0;//continue;
        }

        // convert to rgb
        Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
        cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
        // undistort image
        cv::Mat image_undistorted = cv::Mat::zeros(image.rows, image.cols, image.type());
        cv::undistort(image, image_undistorted, cam, distCoeff);

        switch (marker) {
        // Pose estimation with single marker Aruco
        case 'a':
            estimatePoseArucoMarker(image ,distCoeff, rvec, tvec, cam);
            writeFile(rvec, errors + "rotation_aruco.txt");
            writeFile(tvec, errors + "translation_aruco.txt");
            image_file = imagesfolder + "aruco/";
            break;
        // Pose estimation with board of Aruco
        case 'b':
            estimatePoseArucoBoard(image, cam, distCoeff, rvec, tvec);
            writeFile(rvec, errors + "rotation_board.txt");
            writeFile(tvec, errors + "translation_board.txt");
            image_file = imagesfolder + "board/";
            break;
        // Pose estimation with dynamic Conic marker
        case 'c':
            //detect ellipses and save them
            centerDetector(image_undistorted, det_ellipses);

            /// 190 is the maximal Radius for hexagonal packing in the monitor
            /// if the Radius is bigger it will show just on conic, else it
            /// will show the dynamic marker with more circles
            if(Radius>190)
            {
                Radius = 520;
                hexagonalPackingContinous(Radius);//one conic
                //pose estimation conic
                PoseConics(det_ellipses, cam, rvec, tvec, Radius, distCoeff);
            }
            else
            {
                hexagonalPackingContinous(Radius);// calculate centers on the monitor
                estimatePosePNP(det_ellipses, cam, 1, rvec, tvec, distCoeff);
            }
            // Writing error in .txt format and images names.
            drawAxis(image_undistorted, rvec, tvec, cam, distCoeff);
            writeFile(rvec, errors + "rotation_conicb2.txt");
            writeFile(tvec, errors + "translation_conicb2.txt");
            image_file = imagesfolder + "conic/white_circles/";
            break;
        // Pose estimation with dynamic marker with 6 circles
        case 'x':
            //detect ellipses and save them
            centerDetector(image_undistorted, det_ellipses);

            /// 190 is the maximal Radius for hexagonal packing in the monitor
            /// if the Radius is bigger it will show 6 circles with square packing,
            /// else it will show the dynamic marker with more circles
            if(Radius>190)
            {
                Radius = 270;
                squarePacking(Radius);//6 circles

                //pose estimation 6 circles
                estimatePosePNP(det_ellipses, cam, 0, rvec, tvec, distCoeff);
            }
            else
            {
                hexagonalPackingContinous(Radius);// list of the centers
                estimatePosePNP(det_ellipses, cam, 1, rvec, tvec, distCoeff);
            }
            // Writing error in .txt format and images names.
            drawAxis(image_undistorted, rvec, tvec, cam, distCoeff);
            writeFile(rvec, errors + "rotation_6cb2.txt");
            writeFile(tvec, errors + "translation_6cb2.txt");
            image_file = imagesfolder + "6c/white_circles/";
            break;
        default:
            return 0;
            break;
        }
        // showing images
        cv::namedWindow("image", cv::WINDOW_NORMAL);
        cv::moveWindow("image", 1400, 150);
        cv::resizeWindow("image", cv::Size(800, 800));
        cv::imshow("image", image_undistorted);

        // saving images
        char buffer [4];
        std::sprintf(buffer, "%02d", i);
        image_file += imgdist + "cm_" + buffer;
        std::cout << buffer << std::endl;
        cv::imwrite(image_file + ".png", image);
        cv::waitKey(1);
        i++;
    }

    error = camera.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    camera.Disconnect();

    return 0;
}

// Writes the estimated pose in txt file
void writeFile(cv::Vec3d value, std::string filename){

    std::ofstream file;
    file.open(filename, std::ios_base::app);
    std::time_t t = std::time(0);

    file << t << "," << value << std::endl;

    file.close();
}
/// Writes the estimated pose in yaml files
void writeYaml(std::vector<cv::Vec3d> tvecs, std::vector<cv::Vec3d> rvecs){
    std::string yamlfile = errors + "conic_white_080cm_est.yaml";
    cv::FileStorage fs(yamlfile, cv::FileStorage::WRITE);
    // saving rotations
    fs << "rvecs" << "[";
    for(uint i = 0; i < rvecs.size(); i++){
        fs << rvecs.at(i);
    }
    fs << "]";
    fs << "tvecs" << "[";
    for(uint i = 0; i < tvecs.size(); i++){
        fs << tvecs.at(i);
    }
    fs << "]";
}

/// Analyzes the images saved in the image folders for further pose estimation
void imageAnalyze(){
    // list of translation and rotation vectors
    std::vector<cv::Vec3d> tvecs, rvecs;
    // reading camera parameters
    readCameraParameters();
    // reading images
    std::string imgs2read = imagesfolder + "conic/white_circles/080cm_";
    std::vector<cv::Mat> images;
    cv::Mat image, image_undistorted;

    char buffer [4];
    for(int i = 0; i < 100; i++){
        std::sprintf(buffer, "%02d", i);
        image = cv::imread(imgs2read + buffer + ".png");
        images.push_back(image);
    }

    //calculating the displayed radius
    double Radius = controller.calculate(0.8, r_soll, cam);
    //iterating over the undistorted images
    for(uint i = 0; i < images.size(); i++){
        image = images.at(i);
        cv::undistort(image, image_undistorted, cam, distCoeff);
        //detect ellipses and save them
        centerDetector(image_undistorted, det_ellipses);

        //calculate centers on the monitor
        if(Radius>190)
        {
            // pose estimation conic
            Radius = 520;
            hexagonalPackingContinous(Radius);//one conic
            PoseConics(det_ellipses, cam, rvec, tvec, Radius, distCoeff);

            // pose estimation 6circles
            // Radius = 270;
            // squarePacking(Radius);//6 circles

        }
        else
        {
            hexagonalPackingContinous(Radius);//getting list of object points

            //pose estimation dynamic marker
            estimatePosePNP(det_ellipses, cam, 1, rvec, tvec, distCoeff);
        }
        // drawing axis
        drawAxis(image_undistorted, rvec, tvec, cam, distCoeff);
        //saving the estimated pose
        tvecs.push_back(tvec); rvecs.push_back(rvec);
        cv::imshow("prueba", image_undistorted);
        cv::waitKey(1);
    }
    // writing file of estimated vectors
    writeYaml(tvecs, rvecs);
}
