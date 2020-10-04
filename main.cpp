/* Written by Unbinilium - Jan, 12, 2020 :( */

#include <iostream>
using namespace std;

#include <serial.hpp>
#include <color_area.hpp>
using namespace ubn;

int main()
{
    ubn::SerialProp serial_prop = { "/dev/ttyUSB0", //TTY device name
                                    115200,         //Baudrate
                                    0,              //Opened serial
                                    10000,          //Recieve delay (micro second)
                                    false,          /*Serial status
                                                        false->serial not opened
                                                        ture->serial opend at fd
                                                    */
                                    true            /*Serial keepalive
                                                        false->close serial after using
                                                        ture->keepalive serial
                                                    */
                                  };

    ubn::StreamProp stream_prop = {
        0,                  //Camera index
        cv::CAP_ANY,        //API id
        640,                //Frame width
        360,                //Frame height
        1,                  /*Camera v4l2 exposure mode
                             1->manually exposure
                             3->auto exposure
                             */
        5000,               //Camera v4l2 exposure value (78-10000)
        { 0,                //ROI rect strat x (left->rignt)
            640,            //ROI rect end x (left->rignt)
            0,              //ROI rect strat y (up->down)
            360             //ROI rect end y (up->down)
        },
        90,                 /*Camera angle of vectical
                             0->camera face to the ground
                             90->camera face to the front
                             */
        3,                   //Delay of waitkey between processing 2 frame
        80                   //Frame read count

    };
    
    ubn::RectProp rect_prop = {
        6,                  //Contour detect depth
        100,                //Canny threshold max
        1000,               //Squart area threshold max
        0.3,                //Cosine max threshold of each edge line
        0.02,               //Approx epsilon of lenth calculating
    };
    
    ubn::ColorRange color[] = {
        { "red",                //Color name
            { 0,180,180 },      //HSV lowerb
            { 60,255,255 },     //HSV upperb
            150,                //Minimal area threshold
            2                   //Color at what the crossing tag number is
        },
        { "blue",{ 120,120,100 },{ 200,255,255 },150,3 },
        { "green",{ 30,100,100 },{ 120,255,255 },150,4 },
        { "black",{ 0,50,0 },{ 360,255,90 },150,5 }
    };
    
    std::vector<ubn::ColorRange> color_range;
    for(auto &_color : color) color_range.push_back(_color);
    
    std::cout << "main->start color area recognition" << std::endl;

    unsigned int max_color_area_index = ubn::colorArea(stream_prop, rect_prop, color_range);

    std::cout << "max->" << color[max_color_area_index].color_name << std::endl;
    
    return 0;
}
