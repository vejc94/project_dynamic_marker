QT -= gui
QT += core
QMAKE_CXXFLAGS+= -std=c++11
CONFIG -= app_bundle
CONFIG += console
TARGET=image_grabbing


INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/  /usr/include/flycapture/
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu -lopencv_core3 -lopencv_imgcodecs3 -lopencv_highgui3 -lopencv_calib3d3 -lopencv_imgproc3 -lopencv_videoio3 -lopencv_aruco3
LIBS += -L/usr/lib/ -lflycapture
SOURCES += main.cpp \
    center_detector.cpp \
    pose_estimation.cpp \
    circlepacking.cpp \
    circlecontroller.cpp

HEADERS += \
    center_detector.h \
    pose_estimation.h \
    circlepacking.h \
    circlecontroller.h
