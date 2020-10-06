SOURCES += \
    src/main.cpp

INCLUDEPATH += \
    src/headers \
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
    src/headers/serial.hpp \
    src/headers/color_area.hpp \
    src/headers/line_follow.hpp \
    src/headers/ball_track.hpp
