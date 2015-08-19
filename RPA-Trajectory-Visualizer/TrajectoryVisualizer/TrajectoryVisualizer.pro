#-------------------------------------------------
#
# Project created by QtCreator 2015-05-05T07:13:08
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TrajectoryVisualizer
TEMPLATE = app


SOURCES += main.cpp\
        trajectoryvisualizer.cpp \
    loader.cpp \
    trajectory.cpp \
    graph.cpp \
    pointinfo.cpp \
    graphmodel.cpp

HEADERS  += trajectoryvisualizer.h \
    loader.h \
    trajectory.h \
    graph.h \
    pointinfo.h \
    graphmodel.h

FORMS    += trajectoryvisualizer.ui

RESOURCES += \
    trajectoryvisualizer.qrc
