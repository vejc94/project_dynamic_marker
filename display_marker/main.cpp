// Project includes
#include "openglwindow.h"

#include <math.h>
//OpenCV related includes
#include "opencv2/core/opengl.hpp"
#include "opencv2/highgui.hpp"

// QT related includes
#include <QtGui/QGuiApplication>
#include <QDir>

// QT OpenGL related includes
#include <QOpenGLPaintDevice>
#include <QOpenGLTexture>

int mon_rows = 1200;
int mon_cols = 1920;
std::vector<cv::Point> centers_sq;
std::vector<cv::Point> centers_hx;


QImage loadTexture2(char *filename, GLuint &textureID);
void hexagonalPacking(double radius);
void squarePackingContinous(double radius);
std::vector<cv::Point> getCentersHX();
std::vector<cv::Point> getCentersSQ();

class ImageDisplayWindow : public OpenGLWindow
{
public:
    ImageDisplayWindow();

    void initialize() override;
    void render() override;
    int window_width;
    int window_height;

private:

};

ImageDisplayWindow::ImageDisplayWindow(): window_width(mon_cols), window_height(mon_rows)
{
}

using namespace cv;

int main(int argc, char **argv)
{
    QSurfaceFormat format;
    format.setSwapInterval(1);
    QSurfaceFormat::setDefaultFormat(format);

    /// QguiApplication takes care of the input arguments, the event loop, ..
    QGuiApplication app(argc, argv);
    ImageDisplayWindow window;

    window.setFormat(format);
    window.resize(window.window_width, window.window_height);
    window.showFullScreen();
    //window.show();

    /// app.exec() launches the event loop, loop that waits for user input in gui application
    /// the event loop is running and waiting for events.
    /// It is necessary to call this function to start event handling.
    /// The main event loop receives events from the window system and dispatches these to the application widgets.
    return app.exec();
}


void ImageDisplayWindow::initialize()
{
  glEnable(GL_POLYGON_SMOOTH);
  setAnimating(true);
}



void drawCircle(float x, float y, float radius, float r, float g, float b)
{
  float theta;
  glColor3f(r,g,b);
   glBegin(GL_POLYGON);
   for(float i=0; i<360; i=i+0.1){
       theta = i*3.1416/180;
       glVertex2f(x + radius*cos(theta), y + radius*sin(theta));
   }
   glEnd();
   glFlush();
}

void ImageDisplayWindow::render(){
    //glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //Projectionstransformation
    // These next lines are needed to display 3D figures as 2D.
    // glOrtho with these paremeters allows you to draw using directly screen coordinates
    // origin of the coordinate system on the top left corner of the image (as in OpenCV)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, width(), height());
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glOrtho(0.0f, width(), height(), 0.0f, -1.0f, 1.0f);


    // Here you draw the circles, I created a basic function
    // to draw a single circle in a given coordinate
    // you can call this function inside your own circle packing functions

    static double inc = 540;
    if(inc > 192) squarePackingContinous(inc);
    else hexagonalPacking(inc);
    inc = inc -0.5;
}
void hexagonalPacking(double radius){
    cv::Mat image(mon_rows, mon_cols, CV_8UC3);
    image.setTo(0);
    int m = mon_rows /(sqrt(3.2) * radius); //number of circles in the rows
    int n = mon_cols /(2 * radius); //number of circle in the columns
    static int n_old = n;
    static int m_old = m;

    if((n_old != n)||(m_old != m)||centers_hx.empty()){
        n_old = n;
        m_old = m;
        centers_hx.clear();

        for(int i = 0; i < m; i++){//rows
            for(int j = 0; j < n; j++){//columns
                if(i % 2 !=0){
                    if(n-j==1)continue;
                    centers_hx.push_back(cv::Point(2*(j+1)*mon_cols/(n*2), (1+i*sqrt(3))*mon_rows/(m*sqrt(3.3))));
                }
                else{
                   centers_hx.push_back(cv::Point((2*j+1)*mon_cols/(2*n),(1+i*sqrt(3))*mon_rows/(sqrt(3.3)*m)));
                }
            }
        }
    }
    for(uint i=0; i<centers_hx.size(); i++){
        if(i==0){
            drawCircle(centers_hx.at(i).x, centers_hx.at(i).y, radius - 10, 0.0f, 1.0f, 0.0f);
            continue;
        }
        else if(i == centers_hx.size()-1){
            drawCircle(centers_hx.at(i).x, centers_hx.at(i).y, radius - 10,1.0f,0.0f,0.0f);
            continue;
        }
        drawCircle(centers_hx.at(i).x, centers_hx.at(i).y, radius - 10,1.0f,1.0f,1.0f);//circle(image, centers_hx.at(i), (int)radius-15, Scalar(0,0,255), -1);


    }
}

void squarePackingContinous(double radius){

    int m = mon_rows/(2*radius);//number of circles in the rows
    int n = mon_cols/(2*radius);//number of circle in the columns


    centers_sq.clear();
    if(n <= 4 && m<=2){
        centers_sq.push_back(cv::Point(radius,mon_rows/2));//left circle: 1. circle
        centers_sq.push_back(cv::Point(2*radius + (mon_cols-2*radius)/2, mon_rows/2));//right circle: 2. circle

        double r_2 = mon_cols - (2*radius + (mon_cols-2*radius)/2); //2. circle radius
        double r_3 =  mon_cols/2 - 2*radius; //3. circle radius
        double r_bottom;
        ///draw circles

        if( n == 1 ){
            for(uint i = 0; i<centers_sq.size(); i++ ){
                centers_sq.at(i).y = 10 + radius;
            }

            drawCircle(centers_sq.at(0).x,centers_sq.at(0).y, radius-30,0.0f,1.0f,0.0f);//circle(image, centers_sq.at(0), radius-30, Scalar(255,255,255), -1);
            drawCircle(centers_sq.at(1).x,centers_sq.at(1).y, r_2-30,1,1,1) ; //circle(image, centers_sq.at(1), r_2 - 30, Scalar(255,255,255), -1 );

                ///three bottom circles
                r_bottom = (mon_rows-2*radius)/2 ;
                centers_sq.push_back(cv::Point(centers_sq.at(0).x, mon_rows - r_bottom));
                centers_sq.push_back(cv::Point(centers_sq.at(1).x, mon_rows - r_bottom));
                centers_sq.push_back(cv::Point(mon_cols/2, mon_rows - r_bottom));


                drawCircle(centers_sq.at(2).x,centers_sq.at(2).y, r_bottom - r_bottom/10,1,1,1);
                drawCircle(centers_sq.at(3).x,centers_sq.at(3).y, r_bottom - r_bottom/10,1,1,1);
                drawCircle(centers_sq.at(4).x,centers_sq.at(4).y, r_bottom - r_bottom/10,1,1,1);


        }
        else if( n <= 4 ){

            centers_sq.at(1) = cv::Point(mon_cols-radius, mon_rows/2);//right circle needs to move to the right again.
            //new circle in the middle: 3. circle
            centers_sq.push_back(cv::Point(mon_cols/2, mon_rows/2));

            ///y-coordinate of all circles need to change
            for(uint i = 0; i<centers_sq.size(); i++ ){
                centers_sq.at(i).y = 10 + radius;
            }
            drawCircle(centers_sq.at(0).x,centers_sq.at(0).y, radius-30,0.0f,1.0f,0.0f);//circle(image, centers_sq.at(0), radius-30, Scalar(255,255,255), -1);//left
            drawCircle(centers_sq.at(1).x,centers_sq.at(1).y, radius-30,1,1,1) ;//circle(image, centers_sq.at(1), radius-30, Scalar(255,255,255), -1);//right

            if(r_3>radius-5){
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                for(uint i = 0; i<centers_sq.size(); i++ ){
                    centers_sq.at(i).x = (2*i+1)*radius;//move circles to the left
                    if(i==0){
                        drawCircle(centers_sq.at(i).x, centers_sq.at(i).y, radius-30, 0.0f, 1.0f, 0.0f);
                        continue;
                    }

                    drawCircle(centers_sq.at(i).x, centers_sq.at(i).y, radius-30, 1.0f, 1.0f, 1.0f);//circle(image, centers_sq.at(i), radius-30, Scalar(255,255,255), -1 );//middle
                }

            }
            else{drawCircle(centers_sq.at(2).x,centers_sq.at(2).y, r_3 - r_3/20,1,1,1);}//circle(image, centers_sq.at(2), r_3 - r_3/20, Scalar(255,255,255), -1 );}//middle

            ///bottom circles
            r_bottom = (mon_rows - 2*radius)/2;
            for(int i = 0; i < 3; i++){
                centers_sq.push_back(cv::Point(centers_sq.at(i).x, mon_rows-r_bottom -10 ));
            }
            for(uint i = centers_sq.size()-1; i> centers_sq.size()-4;i--){
                if(r_bottom < radius-20){
                    //circle(image, centers_sq.at(i), r_bottom - r_bottom/20, Scalar(255,255,255), -1);
                    drawCircle(centers_sq.at(i).x,centers_sq.at(i).y ,r_bottom - r_bottom/20,1,1,1);
                }
                else{
                    centers_sq.at(i).y = 3*radius - 25 ;
                    drawCircle(centers_sq.at(i).x, centers_sq.at(i).y, radius-30,1,1,1);//circle(image, centers_sq.at(i), radius - 30, Scalar(255,255,255), -1);
                }
            }


            if(m<=2){

                if(6*radius < mon_cols){
                    ///wide right top and bottom circles
                    double r_4 = (mon_cols - 6*radius)/2;
                    centers_sq.push_back(cv::Point(mon_cols-r_4-10, radius ));
                    centers_sq.push_back(cv::Point(mon_cols-r_4-10, 3*radius ));
                    //double r_4 = centers_sq.at(6).x - 6*radius - 10;


                     if(r_4>0 && r_4 < radius){
                        drawCircle(centers_sq.at(6).x, centers_sq.at(6).y, r_4 -r_4/10,1,1,1);//circle(image, centers_sq.at(6), r_4 -r_4/10 , Scalar(255,255,255), -1);
                        drawCircle(centers_sq.at(7).x, centers_sq.at(7).y, r_4 -r_4/10,1,1,1);//circle(image, centers_sq.at(7), r_4 -r_4/10, Scalar(255,255,255), -1);
                    }
                    else if (r_4>radius) {
                        centers_sq.at(6).x = (7)*radius-10; centers_sq.at(6).y = radius + 10;
                        centers_sq.at(7).x = (7)*radius-10; centers_sq.at(7).y = 3*radius - 10;
                        drawCircle(centers_sq.at(6).x, centers_sq.at(6).y, radius -30,1,1,1);//circle(image, centers_sq.at(6), radius -30 , Scalar(255,255,255), -1);
                        drawCircle(centers_sq.at(7).x, centers_sq.at(7).y, radius-30,1,1,1);//circle(image, centers_sq.at(7), radius -30 , Scalar(255,255,255), -1);
                        //std::cout << radius << std::endl;
                    }

                    ///new wide right circles
                    double r_6 = (mon_cols-8*radius)/2;
                    if(r_6>0){
                        for(int i = 0; i < 2; i++){
                            centers_sq.push_back(cv::Point(mon_cols-r_6 -10, (2*i+1)*radius));
                        }
                        for(uint i = centers_sq.size()-1; i > centers_sq.size()-3; i--){
                            drawCircle(centers_sq.at(i).x, centers_sq.at(i).y, r_6 -r_6/9,1,1,1);//circle(image, centers_sq.at(i), r_6 -r_6/9 , Scalar(255,255,255), -1);
                        }
                    }
                    ///new bottom circles
                    double r_5 = (mon_rows - 2*m*radius)/2;
                    if(m==2){

                        for(int i = 0; i < 4; i++){
                            centers_sq.push_back(cv::Point((2*i+1)*radius, mon_rows-r_5-15));
                        }
                        for(uint i = centers_sq.size()-1; i > centers_sq.size()-5; i--){
                            //circle(image, centers_sq.at(i), r_5 -r_5/10 , Scalar(255,255,255), -1);
                            drawCircle(centers_sq.at(i).x, centers_sq.at(i).y, r_5 -r_5/10,1,1,1);
                        }
                        //bottom right corner
                        centers_sq.push_back(cv::Point(mon_cols-r_6, mon_rows-r_5-15));
                        //circle(image, centers_sq.at(centers_sq.size()-1), r_6 -r_6/3 , Scalar(255,255,255), -1);
                        if(r_6>0) drawCircle(centers_sq.at(centers_sq.size()-1).x,centers_sq.at(centers_sq.size()-1).y, r_6 -r_6/3,1,1,1 );

                    }
                }
            }
        }

    }
}

std::vector<cv::Point> getCentersHX(){
    return centers_hx;
}
std::vector<cv::Point> getCentersSQ(){
    return centers_sq;
}


