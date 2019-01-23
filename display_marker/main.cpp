// Project includes
#include "openglwindow.h"
#include "circlecontroller.h"
#include <math.h>
//OpenCV related includes
#include <opencv2/core/opengl.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

// QT related includes
#include <QtGui/QGuiApplication>
#include <QDir>

// QT OpenGL related includes
#include <QOpenGLPaintDevice>
#include <QOpenGLTexture>

using namespace cv;

int mon_rows = 1080;
int mon_cols = 1920;
int gap = 30;
std::vector<Point> centers_sq;
std::vector<Point> centers_hx;
std::vector<Point> centers_hx_real;
double Radius;
double z;

//desired projected radius on the camera.
double r_soll = 50;

QImage loadTexture2(char *filename, GLuint &textureID);
void squarePacking(double radius);
void hexagonalPackingContinous(double radius);
std::vector<cv::Point> getCentersHX();
std::vector<cv::Point> getCentersSQ();

class ImageDisplayWindow : public OpenGLWindow
{
public:
    ImageDisplayWindow();

    void initialize() override;
    void render() override;
    void drawAruco();
    int window_width;
    int window_height;

private:
    QOpenGLTexture *texture;
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

//    ///varying size marker
//    circleController controller(520,5);
//    ///fixed marker
//    //circleController controller(110,110);

//    std::cout<< "give distance z"<< std::endl;
//    std::cin >> z;
//    Radius = controller.calculate(z,r_soll);


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


void ImageDisplayWindow::drawAruco(){

    cv::Mat marker_image;

    //Marker dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(10));

    //draw one marker
//    cv::aruco::drawMarker(dictionary, 23, 1080, marker_image, 1);
//    cv::imwrite("aruco_marker.png", marker_image);

    //create board
    //parameters for aruco grid board 32 corners = 4x2 markers
    int markers_x = 4; //Numbers of markers in X direction
    int markers_y= 2; //Numbers of markers in Y direction
    float markers_length = 410*0.000375; //markers side lenght in meter (number of pixels * pixel pitch)
    float markers_gap = 50*0.000375; // Separation between two consecutive markers in the grid in meter (number of pixels * pixel pitch)
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markers_x, markers_y, markers_length, markers_gap, dictionary);

    //image for marker board
    board->draw(cv::Size(1920,1080), marker_image, 50 , 1);
    cv::imwrite("aruco_board.png", marker_image);


    QImage q_image = QImage(QString::fromStdString("aruco_board.png"));
    q_image.bits();


    texture = new QOpenGLTexture(q_image.mirrored());
    texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
    texture->setMagnificationFilter(QOpenGLTexture::Linear);
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

/*
 * With this render function the aruco marker and the aruco board are displayed!
*/

void ImageDisplayWindow::render(){
    glViewport(0,0,1920,1080);
    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_TEXTURE_2D);

    //draw Aruco
    drawAruco();
    texture->bind();

    //Projectionstransformation
    // These next lines are needed to display 3D figures as 2D.
    // glOrtho with these paremeters allows you to draw using directly screen coordinates
    // origin of the coordinate system on the top left corner of the image (as in OpenCV)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, width(), height());
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //for rendering one marker
//    glOrtho(0.0f, 1920, 0.0, 1080, -1.0f, 1.0f);
//    glBegin(GL_QUADS);
//          glTexCoord2f(0,0);
//          glVertex2f(420, 0);
//          glTexCoord2f(1,0);
//          glVertex2f(1500, 0);
//          glTexCoord2f(1,1);
//          glVertex2f(1500, 1080);
//          glTexCoord2f(0,1);
//          glVertex2f(420, 1080);
//     glEnd();

    //for rendering a board
    glBegin(GL_QUADS);
    glTexCoord2f(0,0);
          glVertex2f(-1.0f, -1.0f);
          glTexCoord2f(1,0);
          glVertex2f(1.0f, -1.0f);
          glTexCoord2f(1,1);
          glVertex2f(1.0f, 1.0f);
          glTexCoord2f(0,1);
    glVertex2f(-1.0f, 1.0f);
    glEnd();

    glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);

    glDisable(GL_TEXTURE_2D);
}

/*
 * With this render function the circles are displayed!
*/
//void ImageDisplayWindow::render(){
//    //glClearColor(0,0,0,1);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    //Projectionstransformation
//    // These next lines are needed to display 3D figures as 2D.
//    // glOrtho with these paremeters allows you to draw using directly screen coordinates
//    // origin of the coordinate system on the top left corner of the image (as in OpenCV)
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    glViewport(0, 0, width(), height());
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//    glOrtho(0.0f, width(), height(), 0.0f, -1.0f, 1.0f);


//    // Here you draw the circles

////     hexagonalPackingContinous(Radius);

//    if(Radius > 190){//190 is the maximal radius to display a hexagon
//      //squarePacking(270);//Radius = 270 to display 6 circles
//      hexagonalPackingContinous(520);//one conic
//    }
//    else hexagonalPackingContinous(Radius);

//}




void hexagonalPackingContinous(double radius){
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
    for(uint i = 0; i < centers_hx.size(); i++){
        drawCircle(centers_hx.at(i).x, centers_hx.at(i).y, radius-gap, 255,255,255);
    }

}

std::vector<cv::Point> getCentersHX(){
    return centers_hx;
}
std::vector<cv::Point> getCentersSQ(){
    return centers_sq;
}
void squarePacking(double radius){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    centers_sq.clear();
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
            drawCircle(mon_cols/2,mon_rows/2,radius-gap, 255,255,255);
            }
        else{
            drawCircle(centers_sq.at(i).x, centers_sq.at(i).y, radius-gap, 255,255,255);//-10 para dar espacio entre circulos
        }
    }
}

