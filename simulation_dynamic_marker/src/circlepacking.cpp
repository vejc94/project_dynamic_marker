#include "circlepacking.h"

using namespace cv;

int mon_rows = 1080;
int mon_cols = 1920;
int gap = 10;
std::vector<Point> centers_sq;
std::vector<Point> centers_hx;
std::vector<Point> centers_hx_real;

///this funtion packs the cirlce in the image with the given radius in a square form.

void squarePacking(Mat image, double radius){
    image.setTo(0);
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
    ///draw circles
    for(uint i = 0; i < centers_sq.size(); i++){// columns
        if((n==1)&&(m==1)){
            circle(image, Point(mon_cols/2,mon_rows/2),radius - 30,Scalar(255,255,255),-1);
        }
        else{
            circle(image, centers_sq.at(i), (int)radius-30, Scalar(255,255,255), -1);//-10 para dar espacio entre circulos


        }
    }
}



///this function packs the circle in the image with the given radius in a hexagonal form.
/// The hexagonal form has the highest packing density.

void hexagonalPacking(Mat image, double radius){
    image.setTo(0);
    int m = mon_rows /(sqrt(3.3) * radius); //number of circles in the rows
    int n = mon_cols /(2 * radius); //number of circle in the columns
//    static int n_old = n;
//    static int m_old = m;

/*    if((n_old != n)||(m_old != m)||centers_hx.empty()){
        n_old = n;
        m_old = m;
        */
        centers_hx.clear();

        for(int i = 0; i < m; i++){//rows
            for(int j = 0; j < n; j++){//columns
                if(i % 2 !=0){
                    if(n-j==1)continue;
                    centers_hx.push_back(Point(2*(j+1)*mon_cols/(n*2), (1+i*sqrt(3))*mon_rows/(m*sqrt(3.3))));
                }
                else{
                   centers_hx.push_back(Point((2*j+1)*mon_cols/(2*n),(1+i*sqrt(3))*mon_rows/(sqrt(3.3)*m)));
                }
            }
        }
    //}
    for(uint i=0; i<centers_hx.size(); i++){
        if(i<(uint)n) circle(image, centers_hx.at(i), (int)radius-15, Scalar(255,255,255), -1);
        else circle(image, centers_hx.at(i), (int)radius-15, Scalar(255,255,255), -1);

    }



}

void squarePackingContinous(Mat image, double radius){
    image.setTo(0);
    int m = mon_rows/(2*radius);//number of circles in the rows
    int n = mon_cols/(2*radius);//number of circle in the columns



    ///check if the number of circles has changed.
    /// if true then save new circle centers until the numbers m or n have changed
    //if((n_old != n)||(m_old != m)||centers_sq.empty()){
    //n_old = n;
    //m_old = m;
    centers_sq.clear();
    if(n <= 4 && m<=2){
        centers_sq.push_back(Point(radius,mon_rows/2));//left circle: 1. circle
        centers_sq.push_back(Point(2*radius + (mon_cols-2*radius)/2, mon_rows/2));//right circle: 2. circle

        double r_2 = mon_cols - (2*radius + (mon_cols-2*radius)/2); //2. circle radius
        double r_3 =  mon_cols/2 - 2*radius; //3. circle radius
        double r_bottom;
        ///draw circles

        if( n == 1 ){
            circle(image, centers_sq.at(0), radius-30, Scalar(255,255,255), -1);
            circle(image, centers_sq.at(1), r_2 - 30, Scalar(255,255,255), -1 );
        }
        else if( n <= 4 ){

            centers_sq.at(1) = Point(mon_cols-radius, mon_rows/2);//right circle needs to move to the right again.
            //new circle in the middle: 3. circle
            centers_sq.push_back(Point(mon_cols/2, mon_rows/2));

            ///y-coordinate of all circles need to change
            for(uint i = 0; i<centers_sq.size(); i++ ){
                centers_sq.at(i).y = 10 + radius;
            }
            circle(image, centers_sq.at(0), radius-30, Scalar(255,255,255), -1);//left
            circle(image, centers_sq.at(1), radius-30, Scalar(255,255,255), -1);//right

            if(r_3>radius-3){
                image.setTo(0);
                for(uint i = 0; i<centers_sq.size(); i++ ){
                    centers_sq.at(i).x = (2*i+1)*radius;//move circles to the left
                    circle(image, centers_sq.at(i), radius-30, Scalar(255,255,255), -1 );//middle
                }

            }
            else{circle(image, centers_sq.at(2), r_3 - r_3/20, Scalar(255,255,255), -1 );}//middle

            ///bottom circles
            r_bottom = (mon_rows - 2*radius)/2;
            for(int i = 0; i < 3; i++){
                centers_sq.push_back(Point(centers_sq.at(i).x, mon_rows-r_bottom -10 ));
            }
            for(uint i = centers_sq.size()-1; i> centers_sq.size()-4;i--){
                if(r_bottom < radius-20) circle(image, centers_sq.at(i), r_bottom - r_bottom/20, Scalar(255,255,255), -1);
                else{
                    centers_sq.at(i).y = 3*radius - 25 ;
                    circle(image, centers_sq.at(i), radius - 30, Scalar(255,255,255), -1);
                }
            }


            if(m<=2){

                if(6*radius < mon_cols){
                    ///wide right top and bottom circles
                    double r_4 = (mon_cols - 6*radius)/2;
                    centers_sq.push_back(Point(mon_cols-r_4-10, radius ));
                    centers_sq.push_back(Point(mon_cols-r_4-10, 3*radius ));
                    //double r_4 = centers_sq.at(6).x - 6*radius - 10;


                    if(r_4>0 && r_4 < radius){
                        circle(image, centers_sq.at(6), r_4 -r_4/10 , Scalar(255,255,255), -1);
                        circle(image, centers_sq.at(7), r_4 -r_4/10, Scalar(255,255,255), -1);
                    }
                    else if (r_4>radius) {
                        centers_sq.at(6).x = (7)*radius-10; centers_sq.at(6).y = radius + 10;
                        centers_sq.at(7).x = (7)*radius-10; centers_sq.at(7).y = 3*radius - 10;
                        circle(image, centers_sq.at(6), radius -30 , Scalar(255,255,255), -1);
                        circle(image, centers_sq.at(7), radius -30 , Scalar(255,255,255), -1);
                        //std::cout << radius << std::endl;
                    }

                    ///new wide right circles
                    double r_6 = (mon_cols-8*radius)/2;
                    if(r_6>0){
                        for(int i = 0; i < 2; i++){
                            centers_sq.push_back(Point(mon_cols-r_6 -10, (2*i+1)*radius));
                        }
                        for(uint i = centers_sq.size()-1; i > centers_sq.size()-3; i--){
                            circle(image, centers_sq.at(i), r_6 -r_6/9 , Scalar(255,255,255), -1);
                        }
                    }
                    ///new bottom circles
                    double r_5 = (mon_rows - 2*m*radius)/2;
                    if(m==2){

                        for(int i = 0; i < 4; i++){
                            centers_sq.push_back(Point((2*i+1)*radius, mon_rows-r_5-15));
                        }
                        for(uint i = centers_sq.size()-1; i > centers_sq.size()-5; i--){
                            circle(image, centers_sq.at(i), r_5 -r_5/10 , Scalar(255,255,255), -1);
                        }
                        //bottom right corner
                        centers_sq.push_back(Point(mon_cols-r_6, mon_rows-r_5-15));
                        if(r_6>0)circle(image, centers_sq.at(centers_sq.size()-1), r_6 -r_6/3 , Scalar(255,255,255), -1);

                    }
                }
            }
        }
    }
}

void hexagonalPackingContinous(cv::Mat image, double radius){
    image.setTo(0);
    centers_hx.clear();
    centers_hx_real.clear();
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
        }

        else{
            centers_hx_real.push_back(centers_hx.at(i));
             }
    }
    centers_hx.clear();
    centers_hx=centers_hx_real;
    for(uint i = 0; i < centers_hx.size(); i++){       
        circle(image,centers_hx.at(i), radius-gap, Scalar(255,255,255), -1);

    }

}

std::vector<Point> getCentersHX(){
    return centers_hx;
}
std::vector<Point> getCentersSQ(){
    return centers_sq;
}
