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
    -O3

HEADERS += \
    serial.hpp \
    color_area.hpp
