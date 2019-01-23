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

using namespace FlyCapture2;

//desired projected radius on the camera.
double r_soll = 50;

std::vector<cv::RotatedRect> det_ellipses;//list of ellipses in the image
cv::Mat cam = cv::Mat::zeros(3,3, CV_32F);//camera intrinsic parameters
cv::Mat distCoeff = cv::Mat::zeros(1,5, CV_32F);
cv::Vec3d tvec, rvec;//translation and rotation vectors

//
circleController controller(520,5);//varying size marker
//circleController controller(110,110);//fixed marker

//absolut path to file
std::string filename = "/home/victor94/project_dynamic_marker/image_grabbing/image_grabbing/camera_parameters.yaml";


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

int main()
{
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
    std::cout << cam << std::endl;
    //input for distance
    double z;
    std::cout << "distance z: " << std::endl;
    std::cin >> z;

    //calculate Radius of the displayed circles on the monitor
    double Radius = controller.calculate(z, r_soll, cam);

    if(cam.empty())
    {
        std::cout << "Failed to read camera parameters" << std::endl;
        return 0;
    }

    ///Marker options
    char marker = 0;
    std::cout << "choose with what marker to estimate pose (a)ruco, (b)oard, (c)onic, 6(x)circles" << std::endl;
    std::cin >> marker;


    // capture loop
    char key = 0;
    while(key != 'q')
    {
        // Get the image
        Image rawImage;
        Error error = camera.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK )
        {
            std::cout << "capture error" << std::endl;
            continue;
        }

        // convert to rgb
        Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
        cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        switch (marker) {
        case 'b':
            estimatePoseArucoBoard(image, cam, distCoeff, rvec, tvec);
            break;

        case 'c':
            //detect ellipses and save them
            centerDetector(image, det_ellipses);

            //calculate centers on the monitor
            if(Radius>190)
            {

                hexagonalPackingContinous(520);//one conic
                //pose estimation conic
                estimatePosePNP(det_ellipses, cam, 1, rvec, tvec);
            }
            else
            {
                hexagonalPackingContinous(Radius);
                estimatePosePNP(det_ellipses, cam, 1, rvec, tvec);
            }
            break;

        case 'x':
            //detect ellipses and save them
            centerDetector(image, det_ellipses);

            //calculate centers on the monitor
            if(Radius>190)
            {
                squarePacking(Radius);//6 circles

                //pose estimation 6 circles
                estimatePosePNP(det_ellipses, cam, 0, rvec, tvec);
            }
            else
            {
                hexagonalPackingContinous(Radius);
                estimatePosePNP(det_ellipses, cam, 1, rvec, tvec);
            }
            break;
        default:
            return 0;
            break;
        }

        cv::imshow("image", image);
        key = cv::waitKey(30);
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
