#include "circlepacking.h"

using namespace cv;


int mon_rows = 1080;
int mon_cols = 1920;
int gap = 10;

//List of centers on the monitor
std::vector<Point> centers_sq;
std::vector<Point> centers_hx;
std::vector<Point> centers_hx_real;

///this funtion packs the cirlce in the image with the given radius in a square form.

void squarePacking(double radius){
    int m = mon_rows/(2*radius);//number of circles in the rows
    int n = mon_cols/(2*radius);//number of circle in the columns
    static int n_old = n;
    static int m_old = m;


    ///check if the number of circles has changed.
    /// if true then save new circle centers until the numbers m or n have changed
    if((n_old != n)||(m_old != m)||centers_sq.empty()){
        n_old = n;
        m_old = m;
        centers_sq.clear();
        for(int i = 0; i < n; i++){// columns
            for(int j = 0; j < m; j++){//rows
                centers_sq.push_back(Point((2*i+1)*mon_cols/(2*n), (2*j+1)*mon_rows/(2*m)));//-10 para dar espacio entre circulos
            }
        }
    }
}

void hexagonalPackingContinous(double radius){
    //clear lists
    centers_hx.clear();
    centers_hx_real.clear();

    //add point in monitor center
    centers_hx.push_back(Point(mon_cols/2,mon_rows/2));

    //int a = mon_rows/(sqrt(3)*radius);//number of circles in rows
    int b = mon_cols /(2 * radius);//number of circles in columns

    double t; // parameter for curve

    //scaling factor lamda;
    for(int lamda = 1;lamda<b;lamda++){
        for(t=0;t<radius;t=t+radius/lamda){
            //first curve [R + t, sqrt(3)*(t - R)];
            double x1 = lamda*(radius + t) + mon_cols/2;
            double y1 = lamda*(sqrt(3)*(t - radius)) + mon_rows/2;


            //pushing points
            if((x1-gap<0)||(x1+gap>mon_cols)||(y1-gap<0||y1+gap>mon_rows)){//checking if center fits in the monitor
                continue;
            }
            //mirrored point
            double x1_m = mon_cols - x1;

            centers_hx.push_back(Point(x1,y1));
            centers_hx.push_back(Point(x1_m,y1));
        }
    }
    //scaling factor lamda;
    for(int lamda = 1;lamda<b;lamda++){
        for(t=0;t<=radius;t=t+radius/lamda){
            //second curve [2R - t, sqrt(3)*t];
            double x2 = lamda*(2*radius - t) + mon_cols/2;
            double y2 = lamda*(sqrt(3))*(t) + mon_rows/2;
            //mirrored point
            double x2_m = mon_cols - x2;

            //pushing points
            if((x2-gap<0)||(x2+gap>mon_cols)||(y2-gap<0||y2+gap>mon_rows))//checking if center fits in the monitor
                continue;
            centers_hx.push_back(Point(x2,y2));
            centers_hx.push_back(Point(x2_m,y2));


        }
    }

    //scaling factor lamda;
    for(int lamda = 2;lamda<b;lamda++){
        for(t=2*radius/lamda;t<2*radius;t=t+2*radius/lamda){
            //third curve [ R - t, sqrt(3)*R];
            double x3 = lamda*(radius - t) + mon_cols/2;
            double y3 = lamda*(sqrt(3)*radius) + mon_rows/2;
            //mirrored points;
            //double x3_m = mon_cols - x3;
            double y3_m = mon_rows - y3;
            //pushing points
            if((x3-gap<0)||(x3+gap>mon_cols)||(y3-gap<0||y3+gap>mon_rows))//checking if center fits in the monitor
                continue;
            centers_hx.push_back(Point(x3,y3));
            centers_hx.push_back(Point(x3,y3_m));
        }
    }

    //check if circles center are more than once in the list
    for(uint i=0; i<centers_hx.size();i++){

        uint count = 0;
        for(uint j = 0; j <centers_hx.size();j++){

            double distance = cv::norm(centers_hx.at(i) - centers_hx.at(j));

            if(distance>radius){
                count++;
            }
        }
        if(count==centers_hx.size()-1){
            centers_hx_real.push_back(centers_hx.at(i));
        }
    }

    centers_hx.clear();
    centers_hx=centers_hx_real;
    centers_hx_real.clear();

    for(uint i = 0; i < centers_hx.size(); i++){
        ///Checking if the circles with coordinates x, y fits in the display
        /// if not draw circle with an smaller radius.
        if((centers_hx.at(i).x - radius - gap < 0)||(centers_hx.at(i).y - radius-gap < 0)||(centers_hx.at(i).x + radius+gap > mon_cols)||(centers_hx.at(i).y + radius+gap > mon_rows)){
            //do simply nothing
        }
        else{
            centers_hx_real.push_back(centers_hx.at(i));
        }
    }
    centers_hx.clear();
    centers_hx=centers_hx_real;
}

std::vector<Point> getCentersHX(){
    return centers_hx;
}
std::vector<Point> getCentersSQ(){
    return centers_sq;
}
