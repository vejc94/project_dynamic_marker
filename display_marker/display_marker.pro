TARGET = display_marker

CONFIG += c++11 console


SOURCES += main.cpp \
    openglwindow.cpp \



INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu -lopencv_core3 -lopencv_imgcodecs3 -lopencv_highgui3 -lopencv_imgproc3 -lopencv_calib3d3 -lopencv_structured_light3

HEADERS += \
    openglwindow.h \


