/**
  * @name main.cpp
  * @namespace ubn
  * @brief OpenCV in RoboMaster. colorArea, lineFollow and ballTrack
  * @author Unbinilium(iChizer0, Chen Xiangjie)
  * @date 2020-01-12
 **/

#include<cmath>
#include<vector>
#include<string>
#include<iostream>
using namespace std;

#include <serial.hpp>
#include <color_area.hpp>
#include <line_follow.hpp>
#include <ball_track.hpp>

int main(void)
{
  vector<int> in_data; //Init varible using for store recieved data

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

  ubn::StreamProp stream_prop = { 0,           //Camera index
                                  cv::CAP_ANY, //API id
                                  640,         //Frame width
                                  360,         //Frame height
                                  1,           /*Camera v4l2 exposure mode
                                                 1->manually exposure
                                                 3->auto exposure
                                               */
                                  5000,        //Camera v4l2 exposure value (78-10000)
                                  { 0,         //ROI rect strat x (left->rignt)
                                    640,       //ROI rect end x (left->rignt)
                                    0,         //ROI rect strat y (up->down)
                                    360        //ROI rect end y (up->down)
                                  },
                                  90,          /*Camera angle of vectical
                                                   0->camera face to the ground
                                                   90->camera face to the front
                                               */
                                  1,           //Delay of waitkey between processing 2 frame
                                  80           //Fream read count, as same as triers
                                };

  ubn::RectProp rect_prop = { 3,     //Contour detect depth
                              100,   //Canny threshold max
                              1000,  //Squart area threshold max
                              0.3,   //Cosine max threshold of each edge line
                              0.02,  //Approx epsilon of lenth calculating
                            };

  ubn::ColorRange color[] = { { "red",          //Color name
                                { 0,180,180 },  //HSV lowerb
                                { 60,255,255 }, //HSV upperb
                                150,            //Minimal area threshold
                                2               //Color at what the crossing tag number is
                              },
                              { "blue",{ 120,120,100 },{ 200,255,255 },150,3 },
                              { "green",{ 30,100,100 },{ 120,255,255 },150,4 },
                              { "black",{ 0,50,0 },{ 360,255,90 },150,5 }
                            };

  std::vector<ubn::ColorRange> color_range;
  for(auto &_color : color) color_range.push_back(_color);


  ubn::ColorRange line_color = { "black",
                                 { 0,50,0 },
                                 { 360,255,90 },

                                 /* Not used in lineFollow */
                                 0,
                                 0
                               };

  ubn::ColorRange ball_color = { "blue",
                                 { 120,120,100 },
                                 { 200,255,255 },

                                 /* Not used in ballTrack */
                                 0,
                                 0
                               };

  ubn::LineData line_data = { 1000,                             //Area threshold min (pixels), smaller as noisy
                              15000,                            //Line area threshold max (pixels), larger as crossing
                              int(stream_prop.frame_width / 2), //Line horizental axis (pixels), determined by camera's location
                              20,                               //Line horizental axis vibrate (pixels), when larger then make movement
                              200,                              //ROI of 1st line follow area start y (up->down)
                              260,                              //ROI of 1st line follow area end y (up->down)
                              310,                              //ROI of 2nd line follow area start y (up->down)
                              360,                              //ROI of 2nd line follow area end y (up->down)
                              20,                               //Line angle rotation vibrate (pixels), when larger then make rotatement
                              0,                                //Count the crossing tag on the line (not including turns)
                              6                                 //When crossing tag count eqaled, end the current function
                            };

  ubn::BallData input_ball_data = { 100000,                            //Ball area max threshold, when larger then exit ballTrack as ball already collected
                                    int(stream_prop.frame_width / 2),  //Ball center x
                                    int(stream_prop.frame_height / 2), //Ball center y
                                    20                                 //Ball center vibrate (pixels), when larger then send ball location
                                  };

  ubn::MoveData move_data = { 'N',                        /*Move type
                                                              N->NULL (used for initlize)
                                                              M->linefollow
                                                              B->balltrack
                                                              L->stop at some crossing count
                                                              C->change camera angel
                                                          */

                              /* Move type 'L' and 'C' used varible */
                              0,                          /*Move arguements
                                                              (L)->crossing tag count (0~n)
                                                              (C)->camera angel (0~180)
                                                          */

                              /* Move type 'L', 'M' and 'B' used varible */
                              'D',                        /*Move direction
                                                              (L)->
                                                                U->move forward
                                                                B->move backward
                                                                D->keep current direction or not move
                                                              (M)->
                                                                L->move left
                                                                R->move right
                                                                D->keep current direction or not move
                                                              (B)->
                                                                L->move left
                                                                R->move right
                                                                N->keep current location or not move
                                                          */

                              /* Move type 'M' and 'B' used varible */
                              0,                          /*Move direction offset
                                                              >0->position offset between car body and line, only trigger when move direction set
                                                              =0->keep current direction or not move
                                                          */
                              'D',                        /*Move rotation
                                                              (M)->
                                                                 L->rotate left
                                                                 R->rotate right
                                                                 D->keep current rotation or not rotate
                                                              (B)->
                                                                 U->move up
                                                                 B->move down
                                                                 N->keep current direction or not move
                                                          */
                              0,                          /*Move rotation offset
                                                              >0->rotation angle offset between car body and line, only trigger when move rotation set
                                                              =0->keep current rotationa or not rotate
                                                          */
                            };

  ubn::Location location_data = { 6, //Current period crossing tag count
                                  0  //Current crossing number the car at
                                };

  ubn::initUart(&serial_prop); //Initlize UART serial

  cout << "main->initlized and waiting for singal to start" << endl;

  /* 1th maxium color area recognition */
  ubn::inUart(&serial_prop, &in_data);
  if(in_data[0] == '1')//When recieved char '1', start 1th maxium color area recognition
  {
    in_data.clear();

    cout << "main->start 1th maxium color area recognition" << endl;

    /* Change camera angle and waiting for serial fallback */
    ubn::MoveData move_data_camera = { 'C',stream_prop.camera_angle_V,'D',0,'D',0 };
    ubn::moveAction(&serial_prop, &move_data_camera);
    ubn::fallBack(&serial_prop, true);

    unsigned int max_color_area_index = ubn::colorArea( stream_prop, rect_prop, color_range);   //1th try to find maxium color area

    /* Reset line data */
    line_data.line_mark_cnt = 0;
    line_data.line_mark_cnt_limit = color[max_color_area_index].color_location;

    /* Set move data */
    move_data.move_type = 'L';
    move_data.move_argv = color[max_color_area_index].color_location;
    move_data.move_direction = 'U';


    ubn::moveAction(&serial_prop, &move_data);                        //Send stop crossing tag number and move direction
    ubn::fallBack(&serial_prop, true);                                //Waiting for ready fallback

    ubn::lineFollow(serial_prop, stream_prop, line_color, line_data); //Driving car to the maiumx color area crossing, automatically exit when fallback recieved

    /* Update current location */
    location_data.location_cur = color[max_color_area_index].color_location;

    /* Unlock
         requires car return to the last location after unlocking, keep the camera to the screen
    */

  }

  /* 2th maxium color area recognition */
  ubn::inUart(&serial_prop, &in_data);
  if(in_data[0] == '2') //When recieved char '2', start 2th maxium color area recognition
  {
    in_data.clear();

    cout << "main->start 2th maxium color area recognition" << endl;

    /* Change camera angle and waiting for serial fallback */
    ubn::MoveData move_data_camera = { 'C',stream_prop.camera_angle_V,'D',0,'D',0 };
    ubn::moveAction(&serial_prop, &move_data_camera);
    ubn::fallBack(&serial_prop, true);

    unsigned int max_color_area_index = ubn::colorArea( stream_prop, rect_prop, color_range);   //2th try to find maxium color area

    /* Calculate move direction by current location */
    int mv_tmp = color[max_color_area_index].color_location - location_data.location_cur;

    /* Reset line data */
    line_data.line_mark_cnt = 0;
    line_data.line_mark_cnt_limit = int(fabs(mv_tmp));

    /* Set move data */
    move_data.move_type = 'L';
    move_data.move_argv = line_data.line_mark_cnt_limit;
    move_data.move_direction = mv_tmp > 0 ? 'U' : 'B';

    ubn::moveAction(&serial_prop, &move_data);                        //Send stop crossing tag number and move direction
    ubn::fallBack(&serial_prop, true);                                //Waiting for ready fallback
    ubn::lineFollow(serial_prop, stream_prop, line_color, line_data); //Driving car to the maxium color area crossing, automatically exit when fallback recieved

    /* Update current location */
    location_data.location_cur = color[max_color_area_index].color_location;

    /* Unlock
         requires car return to the last location after unlocking, keep the camera to the screen
    */

  }

  /* Line follow to 1st ball set area */
  ubn::inUart(&serial_prop, &in_data);
  if(in_data[0] == '3') //If recieved char '3', start line follow to 1st ball set area
  {
    in_data.clear();

    cout << "main->start line follow to 1st ball set area" << endl;

    /* Reset line data */
    line_data.line_mark_cnt = 0;
    line_data.line_mark_cnt_limit = location_data.location_cross_tag_cnt - location_data.location_cur;

    /* Set move data */
    move_data.move_type = 'L';
    move_data.move_argv = line_data.line_mark_cnt_limit;
    move_data.move_direction = 'U';

    ubn::moveAction(&serial_prop, &move_data);                        //Send stop crossing tag number and move direction
    ubn::fallBack(&serial_prop, true);                                //Waiting for ready fallback
    ubn::lineFollow(serial_prop, stream_prop, line_color, line_data); //Driving car to the 1st ball set area crossing tag, automatically exit when fallback recieved

    /* Update location data */
    location_data.location_cross_tag_cnt = 1;
    location_data.location_cur = 0;

    /* Ball track */

  }

  /* Ball 1st track */
  ubn::inUart(&serial_prop, &in_data);
  if(in_data[0] == '4') //If recieved char '4', start 1st ball track
  {
    in_data.clear();

    cout << "main->start 1st ball track" << endl;

    ubn::fallBack(&serial_prop, true);                                     //Waiting for fallback to start 1st ball track
    ubn::ballTrack(serial_prop, stream_prop, ball_color, input_ball_data); //Track the ball location, automatically exit when fallback recieved

    /* Line follow
         requires car return to the last location after collected the ball, keep the last rotation
    */
  }

  /* Line follow to 2nd ball set area */
  ubn::inUart(&serial_prop, &in_data);
  if(in_data[0] == '5') //If recieved char '5', start line follow to 2nd ball set area
  {
    in_data.clear();

    cout << "main->start line follow to 2nd ball set area" << endl;

    /* Reset line data */
    line_data.line_mark_cnt = 0;
    line_data.line_mark_cnt_limit = location_data.location_cross_tag_cnt - location_data.location_cur;

    /* Set move data */
    move_data.move_type = 'L';
    move_data.move_argv = line_data.line_mark_cnt_limit;
    move_data.move_direction = 'U';

    ubn::moveAction(&serial_prop, &move_data);                        //Send stop crossing tag number and move direction
    ubn::fallBack(&serial_prop, true);                                //Waiting for ready fallback
    ubn::lineFollow(serial_prop, stream_prop, line_color, line_data); //Driving car to the 2nd ball set area crossing tag, automatically exit when fallback recieved

    /* Update location data */
    location_data.location_cross_tag_cnt = 2;
    location_data.location_cur = 0;

    /* Ball track */

  }

  /* Ball 2nd track */
  ubn::inUart(&serial_prop, &in_data);
  if(in_data[0] == '6') //If recieved char '6', start 2nd ball track
  {
    in_data.clear();

    cout << "main->start 2nd ball track" << endl;

    ubn::fallBack(&serial_prop, true);                                     //Waiting for fallback to start 2nd ball track
    ubn::ballTrack(serial_prop, stream_prop, ball_color, input_ball_data); //Track the ball location, automatically exit when fallback recieved

    /* Line follow
         requires car return to the last location after collected the ball, keep the last rotation
    */

  }

  /* Line follow to endpoint */
  ubn::inUart(&serial_prop, &in_data);
  if(in_data[0] == '7') //If recieved char '7', start line follow to the endpoint
  {
    in_data.clear();

    cout << "main->start line follow to the endpoint" << endl;

    /* Reset line data */
    line_data.line_mark_cnt = 0;
    line_data.line_mark_cnt_limit = location_data.location_cross_tag_cnt - location_data.location_cur;

    /* Set move data */
    move_data.move_type = 'L';
    move_data.move_argv = line_data.line_mark_cnt_limit;
    move_data.move_direction = 'U';

    ubn::moveAction(&serial_prop, &move_data);                        //Send stop crossing tag number and move direction
    ubn::fallBack(&serial_prop, true);                                //Waiting for ready fallback
    ubn::lineFollow(serial_prop, stream_prop, line_color, line_data); //Driving car to the endpoint crossing tag, automatically exit when fallback recieved

    /* end */
  }

  ubn::closeUart(&serial_prop);
  cout << "main->end" << endl;

  return 0;
}
