/**
  * @name serial.hpp
  * @namespace ubn
  * @brief void initUart(ubn::SerialProp *serial_prop)
  * @brief void closeUart(ubn::SerialProp *serial_prop)
  * @brief void inUart(ubn::SerialProp *serial_prop, std::vector<int> *output_data)
  * @brief void outUart(ubn::SerialProp *serial_prop, char input_data[])
  * @brief void moveAction(ubn::SerialProp *serial_prop, ubn::MoveData *input_movedata)
  * @brief bool fallBack(ubn::SerialProp *serial_prop, bool wait_callback)
  * @brief Initialize serial port using wiringPi, basic commiuncation function included
  * @param *StreamProp, *MoveData, wait_callback
  * @author Unbinilium(iChizer0, Chen Xiangjie)
  * @date 2020-01-12
 **/

#ifndef UBN_SERIAL_HPP
#define UBN_SERIAL_HPP

#include <vector>
#include <iostream>

#include <unistd.h>
#include <wiringSerial.h>

namespace ubn {

#ifndef UBN_SERIAL_STRUCT
#define UBN_SERIAL_STRUCT

  struct SerialProp
  {
    const char dev[32];
    const int baud;
    int fd;
    unsigned int delay;
    bool serial_status;
    bool serial_keepalive;
  };

  struct MoveData
  {
    char move_type;             //Move type
    int move_argv;              //Move arguements
    char move_direction;        //Move direction
    int move_direction_offset;  //Move direction offset
    char move_rotation;         //Move rotation
    int move_rotation_offset;   //Move rotation offset
  };

  struct LineData
  {
    int area_threshold_min;     //Ball center vibrate (pixels), when larger then send ball location
    int area_threshold_max;     //Line area threshold max (pixels), larger as crossing
    int line_axis_H;            //Line horizental axis (pixels), determined by camera's location
    int line_axis_H_vibrate;    //Line horizental axis vibrate (pixels), when larger then make movement
    int line_main_V_s;          //ROI of 1st line follow area start y (up->down)
    int line_main_V_e;          //ROI of 1st line follow area end y (up->down)
    int line_dist_V_s;          //ROI of 2nd line follow area start y (up->down)
    int line_dist_V_e;          //ROI of 2nd line follow area end y (up->down)
    int line_dist_R_vibrate;    //Line angle rotation vibrate (pixels), when larger then make rotatement
    int line_mark_cnt;          //Count the crossing tag on the line (not including turns)
    int line_mark_cnt_limit;    //When crossing tag count eqaled, end the current function
  };

  struct BallData
  {
    int area_threshold_max;     //Ball area max threshold, when larger then exit ballTrack as ball already collected
    int ball_ctr_x;             //Ball center x
    int ball_ctr_y;             //Ball center y
    int ball_ctr_vibrate;       //Ball center vibrate (pixels), when larger then send ball location
  };

  struct Location
  {
    int location_cross_tag_cnt; //Current period crossing tag count
    int location_cur;           //Current crossing number the car at
  };

#endif

  void initUart(ubn::SerialProp *serial_prop)
  {
    if((serial_prop->fd = serialOpen(serial_prop->dev, serial_prop->baud)) >= 0)
    {
      serialFlush(serial_prop->fd);
      serial_prop->serial_status = true;

      std::cout << "initUart: uart open succeed->" << serial_prop->fd << std::endl;
    }
    else
    {
      serial_prop->serial_status = false;

      std::cerr << "initUart: uart open failed->" << serial_prop->fd << std::endl;
    }
  }

  void closeUart(ubn::SerialProp *serial_prop)
  {
    serialFlush(serial_prop->fd);
    serialClose(serial_prop->fd);

    std::cout << "closeUart: uart closed->" << serial_prop->fd << std::endl;
  }

  static void outUart(ubn::SerialProp *serial_prop, char input_data[])
  {
      if(serial_prop->serial_status == false)
      {
        std::cout << "outUart: uart not opened, try to init uart->" << serial_prop->fd << std::endl;

        initUart(serial_prop);
      }

    serialPrintf(serial_prop->fd, input_data);

    std::cout << "outUart: sent->" << input_data << std::endl;

    serialFlush(serial_prop->fd);

    if(serial_prop->serial_keepalive == true)
    {
      std::cout << "outUart: uart keepalive->" << serial_prop->fd << std::endl;
    }
    else
    {
      ubn::closeUart(serial_prop);
    }
  }

  void inUart(ubn::SerialProp *serial_prop, std::vector<int> *rec_data)
  {
      if(serial_prop->serial_status == false)
      {
        std::cout << "inUart: uart not opened, try to init uart->" << serial_prop->fd << std::endl;

        initUart(serial_prop);
      }

    while(1)
    {
      usleep(serial_prop->delay);

      int in_cnt = serialDataAvail(serial_prop->fd);

      for(int i = 0; i < in_cnt; i++)
      {
        int ch_tmp = serialGetchar(serial_prop->fd);
        rec_data->push_back(ch_tmp);

        std::cout << ch_tmp << " ";
      }

      if(in_cnt > 0)
      {
        std::cout << std::endl;

        serialFlush(serial_prop->fd);

        break;
      }
    }

    std::cout << "inUart: recieved->";

    for(unsigned int i = 0; i < rec_data->size(); i++)
    {
      std::cout << rec_data->at(i) << " ";
    }
    std::cout << std::endl;

    if(serial_prop->serial_keepalive == true)
    {
      std::cout << "inUart: uart keepalive->" << serial_prop->fd << std::endl;
    }
    else
    {
      ubn::closeUart(serial_prop);
    }
  }

  void moveAction(ubn::SerialProp *serial_prop, ubn::MoveData *input_move_data)
  {
    char ch_tmp[16];

    switch(input_move_data->move_type)
    {
    case 'M':
      {
        std::sprintf(ch_tmp, "%c%c%c%d:%d", 'M', input_move_data->move_direction, input_move_data->move_rotation, input_move_data->move_direction_offset, input_move_data->move_rotation_offset);
        break;
      }

    case 'B':
      {
        std::sprintf(ch_tmp, "%c%c%c%d:%d", 'B', input_move_data->move_direction, input_move_data->move_rotation, input_move_data->move_direction_offset, input_move_data->move_rotation_offset);
        break;
      }

    case 'L':
      {
        std::sprintf(ch_tmp, "%c%c%d", 'L', input_move_data->move_direction, input_move_data->move_argv);
        break;
      }

    case 'C':
      {
        std::sprintf(ch_tmp, "%c%d", 'C', input_move_data->move_argv);
        break;
      }

    default:
      {
        std::cerr << "moveAction: unknown move type" << std::endl;
      }

    }

    char out[18];

    std::sprintf(out, "%s%c", ch_tmp, 'E');

    outUart(serial_prop, out);

    std::cout << "moveAction: " << out << std::endl;
  }

  bool fallBack(ubn::SerialProp *serial_prop, bool wait_callback)
  {
    if(serial_prop->serial_status == false)
    {
      std::cout << "fallBack: uart not opened, try to init uart->" << serial_prop->fd << std::endl;

      initUart(serial_prop);
    }

    if(wait_callback == false)
    {
      if(serialDataAvail(serial_prop->fd) == 0)
      {
        return true;
      }
      else
      {
        serialFlush(serial_prop->fd);

        return false;
      }
    }
    else
    {
      std::cout << "fallBack: waiting for fallback->" << serial_prop->fd << std::endl;

      while(serialDataAvail(serial_prop->fd) == 0);

      serialFlush(serial_prop->fd);

      return false;
    }
  }
}

#endif
