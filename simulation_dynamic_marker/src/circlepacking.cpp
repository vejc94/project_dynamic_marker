#include "circlepacking.h"

using namespace cv;

int mon_rows = 1080;
int mon_cols = 1920;
std::vector<Point> centers_sq;
std::vector<Point> centers_hx;

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

            if(n==4 && m==2)std::cout << radius << std::endl;
        }
    }
}



///this function packs the circle in the image with the given radius in a hexagonal form.
/// The hexagonal form has the highest packing density.

void hexagonalPacking(Mat image, double radius){
    image.setTo(0);
    int m = mon_rows /(sqrt(3.2) * radius); //number of circles in the rows
    int n = mon_cols /(2 * radius); //number of circle in the columns
    static int n_old = n;
    static int m_old = m;

    if((n_old != n)||(m_old != m)||centers_hx.empty()){
        n_old = n;
        m_old = m;
        centers_hx.clear();
//        for(int i = 0; i < n; i++){// columns
//            for(int j = 0; j < m; j++){//rows
//                if(j % 2 != 0 ){
//                    if(n-i==1) continue;
//                    centers_hx.push_back(Point(2*(i+1)*mon_cols/(n*2), (1+j*sqrt(3))*mon_rows/(m*sqrt(3.3))));
//                    //centers_hx.push_back(Point(2*(i+1)*radius, (1+j*sqrt(3))*radius));
//                }
//                else{
//                    centers_hx.push_back(Point((2*i+1)*mon_cols/(2*n),(1+j*sqrt(3))*mon_rows/(sqrt(3.3)*m)));
//                    //centers_hx.push_back(Point((2*i+1)*radius,(1+j*sqrt(3))*radius));
//                }

//            }
//        }
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
    }
    for(uint i=0; i<centers_hx.size(); i++){
        if(i<n) circle(image, centers_hx.at(i), (int)radius-15, Scalar(0,0,255), -1);
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
            for(int i = centers_sq.size()-1; i> centers_sq.size()-4;i--){
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
                        for(int i = centers_sq.size()-1; i > centers_sq.size()-3; i--){
                            circle(image, centers_sq.at(i), r_6 -r_6/9 , Scalar(255,255,255), -1);
                        }
                    }
                    ///new bottom circles
                    double r_5 = (mon_rows - 2*m*radius)/2;
                    if(m==2){

                        for(int i = 0; i < 4; i++){
                            centers_sq.push_back(Point((2*i+1)*radius, mon_rows-r_5-15));
                        }
                        for(int i = centers_sq.size()-1; i > centers_sq.size()-5; i--){
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

std::vector<Point> getCentersHX(){
    return centers_hx;
}
std::vector<Point> getCentersSQ(){
    return centers_sq;
}
