TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += /home/12341234/Downloads/OpenNI-Bin-Dev-Linux-x64-v1.5.4.0/Include
LIBS += -lopencv_video -lopencv_ml -lopencv_objdetect -lopencv_imgproc -lopencv_highgui -lopencv_core -lGL -lglut -lGLU -lGLEW -lOpenNI
SOURCES += main.cpp \
    SceneDrawer.cpp

HEADERS += \
	SceneDrawer.h

