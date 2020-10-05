SOURCES += \
    src/main.cpp

INCLUDEPATH += \
    src \
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
    src/serial.hpp \
    src/color_area.hpp \
    src/line_follow.hpp \
    src/ball_track.hpp

