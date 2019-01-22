//Own files
#include "center_detector.h"

//Camera files
#include <flycapture/FlyCapture2.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//Others
#include <iostream>

using namespace FlyCapture2;


std::vector<cv::RotatedRect> det_ellipses;//list of ellipses in the image
cv::Mat cam;//camera intrinsic parameters

std::string pathtoimgs = "/home/victor94/catkin_ws/src/dynamic_marker/imgs/";
std::string pathtofile = "/home/victor94/catkin_ws/src/dynamic_marker";



void readCameraParameters(cv::Mat &cam){

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

        //detect ellipses and save them
        centerDetector(image, det_ellipses);



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
