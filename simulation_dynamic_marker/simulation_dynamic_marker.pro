QT -= gui
QT += core
QMAKE_CXXFLAGS+= -std=c++11
CONFIG -= app_bundle
CONFIG += console
TARGET=sim_dynamic_marker


INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/  $$PWD/include
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu -lopencv_core3 -lopencv_imgcodecs3 -lopencv_highgui3 -lopencv_calib3d3 -lopencv_imgproc3 -lopencv_videoio3
LIBS += -L/usr/local/lib -lvisp_ar -lvisp_blob -lvisp_core -lvisp_detection -lvisp_gui  -lvisp_robot -lvisp_io -lvisp_vision -lvisp_sensor

SOURCES += src/main.cpp \
    src/circlecontroller.cpp \
    src/circlepacking.cpp

HEADERS += include/circlecontroller.h \
           include/circlepacking.h
