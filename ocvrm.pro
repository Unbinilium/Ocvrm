SOURCES += main.cpp

INCLUDEPATH += \
    /usr/local/include/opencv4 \
    /usr/local/include/opencv4/opencv2

CONFIG += \
    c++17

LIBS += \
    /usr/local/lib/libopencv_*.so \
    /usr/lib/libwiringPi*.so

QMAKE_CXXFLAGS += \
    std:c++17 \
    -O3

HEADERS += \ 
    color_area.hpp
