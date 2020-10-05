/**
  * @name ball_track.hpp
  * @namespace ubn
  * @brief void ballTrack(ubn::SerialProp serial_prop, ubn::StreamProp stream_prop, ubn::ColorRange input_color, ubn::BallData input_ball_data)
  * @brief Ball track using OpenCV
  * @param SerialProp, StreamProp, RectProp, ColorRange, LineData
  * @author Unbinilium(iChizer0, Chen Xiangjie)
  * @date 2020-01-12
 **/

#define DEBUG

#ifndef UBN_BALL_TRACK_HPP
#define UBN_BALL_TRACK_HPP

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <serial.hpp>

namespace ubn {

#ifndef UBN_STRUCT
#define UBN_STRUCT

    struct StreamProp
    {
        const int camera_id;             //Camera index
        const int api_id;                //API id
        const int frame_width;           //Frame width
        const int frame_height;          //Frame height
        const int camrea_exposure_mode;  /*Camera v4l2 exposure mode
                                          1->manually exposure
                                          3->auto exposure
                                          */
        const int camera_exposure;       //Camera v4l2 exposure value (78-10000)
        int frame_threshold[4];          //ROI rect (start_x, end_x, start_y, end_y)
        int camera_angle_V;              //Camera angle of vectical
        int waitkey_delay;               //Delay of waitkey between processing 2 frame
        int read_count;                  //Frame read count
    };

    struct RectProp
    {
        int contour_depth;               //Contour detect depth
        double canny_threshold;          //Canny threshold max
        double area_threshold;           //Squart area threshold max
        double cosine_max_threshold;     //Cosine max threshold of each edge line
        double approx_epsilon;           //Approx epsilon of lenth calculating
    };

    struct ColorRange
    {
        const std::string color_name;    //Color name
        int lowerb[3];                   //HSV lowerb
        int upperb[3];                   //HSV upperb
        int threshold;                   //Minimal area threshold
        int color_location;              //Color at what the crossing tag number is
    };

#endif

#ifndef UBN_BALL_STRUCT
#define UBN_BALL_STRUCT

  struct BallData
  {
    int area_threshold_max;             //Ball area max threshold, when larger then exit ballTrack as ball already collected
    int ball_ctr_x;                     //Ball center x
    int ball_ctr_y;                     //Ball center y
    int ball_ctr_vibrate;               //Ball center vibrate (pixels), when larger then send ball location
  };

#endif

#ifndef UBN_CALLBACK
#define UBN_CALLBACK

  void callBack(int, void *)
  {}

#endif

  static void ballPosition(cv::Mat *p_input_img, cv::Scalar *p_input_lowerb, cv::Scalar *p_input_upperb, cv::Point2f *p_line_ctr, double *p_area_max)
  {
    double p_area_tmp;
    unsigned int area_max_tag = 0;

    cv::Mat p_input_img_tmp = p_input_img->clone();

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::cvtColor(p_input_img_tmp, p_input_img_tmp, cv::COLOR_BGR2HSV_FULL);
    cv::inRange(p_input_img_tmp, *p_input_lowerb, *p_input_upperb, p_input_img_tmp);
    cv::findContours(p_input_img_tmp, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(contours.size() > 0)
    {
      *p_area_max = contourArea(contours[area_max_tag]);
    }

    for(unsigned int i = 0; i != contours.size(); i++)
    {
      p_area_tmp = contourArea(contours[i]);

      if(p_area_tmp > *p_area_max)
      {
        *p_area_max = p_area_tmp;
        area_max_tag = i;
      }
    }

    if(contours.size() > 0)
    {
      cv::Moments momentum = cv::moments(contours[area_max_tag], false);
      *p_line_ctr = cv::Point2f(float(momentum.m10 / momentum.m00), float(momentum.m01 / momentum.m00));

#ifdef DEBUG
      cv::drawContours(*p_input_img, contours, int(area_max_tag), cv::Scalar(255, 255, 255), 2, 8, hierarchy, 0, cv::Point());
      cv::circle(*p_input_img, *p_line_ctr, 4, cv::Scalar(255, 255, 255), -1, 8, 0);
      cv::imshow("ball", p_input_img_tmp);
#endif
    }

    contours.clear();
    hierarchy.clear();
  }

  void ballTrack(ubn::SerialProp serial_prop, ubn::StreamProp stream_prop, ubn::ColorRange input_color, ubn::BallData input_ball_data)
  {
    bool serial_action = false;
    double area_max = 0;

    ubn::MoveData move_data_tmp = { 'C',stream_prop.camera_angle_V,'D',0,'D',0 };
    ubn::moveAction(&serial_prop, &move_data_tmp);
    ubn::fallBack(&serial_prop, true);

    move_data_tmp = { 'B',0,'N',0,'N',0 };

    cv::VideoCapture capture;
    cv::Mat frame;

    capture.open(stream_prop.camera_id + stream_prop.api_id);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, stream_prop.frame_width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, stream_prop.frame_height);
    capture.set(cv::CAP_PROP_AUTO_EXPOSURE, stream_prop.camrea_exposure_mode);
    capture.set(cv::CAP_PROP_EXPOSURE, stream_prop.camera_exposure);

    std::cout << "ballTrack: capture set->" << stream_prop.camera_id + stream_prop.api_id << "->" << stream_prop.frame_width << ":" << stream_prop.frame_height << std::endl;

#ifdef DEBUG
    int slider_threshold_limit = stream_prop.frame_width * stream_prop.frame_height;
    std::string slider_console_name = std::string(input_color.color_name + " slider");

    cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ball", cv::WINDOW_AUTOSIZE);
    cv::namedWindow(slider_console_name, cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("ctr_x", slider_console_name, &input_ball_data.ball_ctr_x, slider_threshold_limit, ubn::callBack);
    cv::createTrackbar("ctr_y", slider_console_name, &input_ball_data.ball_ctr_y, slider_threshold_limit, ubn::callBack);
    cv::createTrackbar("ctr_V", slider_console_name, &input_ball_data.ball_ctr_vibrate, slider_threshold_limit, ubn::callBack);
    cv::createTrackbar("T_x_s", slider_console_name, &stream_prop.frame_threshold[0], stream_prop.frame_width, ubn::callBack);
    cv::createTrackbar("T_x_e", slider_console_name, &stream_prop.frame_threshold[1], stream_prop.frame_width, ubn::callBack);
    cv::createTrackbar("T_y_s", slider_console_name, &stream_prop.frame_threshold[2], stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("T_y_e", slider_console_name, &stream_prop.frame_threshold[3], stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("H_L", slider_console_name, &input_color.lowerb[0], 360, ubn::callBack);
    cv::createTrackbar("H_U", slider_console_name, &input_color.upperb[0], 360, ubn::callBack);
    cv::createTrackbar("S_L", slider_console_name, &input_color.lowerb[1], 255, ubn::callBack);
    cv::createTrackbar("S_U", slider_console_name, &input_color.upperb[1], 255, ubn::callBack);
    cv::createTrackbar("V_L", slider_console_name, &input_color.lowerb[2], 255, ubn::callBack);
    cv::createTrackbar("V_U", slider_console_name, &input_color.upperb[2], 255, ubn::callBack);

    while((capture.isOpened()) && ubn::fallBack(&serial_prop, false))

#else
    while((capture.isOpened()) && ubn::fallBack(&serial_prop, false) && (int(area_max) < input_ball_data.area_threshold_max))
#endif

    {
      capture.read(frame);

      if(frame.empty())
      {
        continue;
      }

#ifdef DEBUG
      double t = cv::getTickCount();
#endif

      frame = frame(cv::Range(stream_prop.frame_threshold[2], stream_prop.frame_threshold[3]), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));
      cv::Scalar input_lowerb_tmp = cv::Scalar(input_color.lowerb[0], input_color.lowerb[1], input_color.lowerb[2]);
      cv::Scalar input_upperb_tmp = cv::Scalar(input_color.upperb[0], input_color.upperb[1], input_color.upperb[2]);
      cv::Point2f ball_ctr;

      ubn::ballPosition(&frame, &input_lowerb_tmp, &input_upperb_tmp, &ball_ctr, &area_max);

      int ball_ctr_x_displacement = int(ball_ctr.x) - input_ball_data.ball_ctr_x;
      if(ball_ctr_x_displacement > input_ball_data.ball_ctr_vibrate)
      {
        serial_action = true;
        move_data_tmp.move_direction = 'L';
        move_data_tmp.move_direction_offset = ball_ctr_x_displacement;

        std::cout << "lineFollow: move left->" << ball_ctr_x_displacement << std::endl;
      }
      if(ball_ctr_x_displacement < -input_ball_data.ball_ctr_vibrate)
      {
        serial_action = true;
        move_data_tmp.move_direction = 'R';
        move_data_tmp.move_direction_offset = -ball_ctr_x_displacement;

        std::cout << "lineFollow: move right->" << ball_ctr_x_displacement << std::endl;
      }

      int ball_ctr_y_displacement = int(ball_ctr.y) - input_ball_data.ball_ctr_y;
      if(ball_ctr_y_displacement > input_ball_data.ball_ctr_vibrate)
      {
        serial_action = true;
        move_data_tmp.move_direction = 'U';
        move_data_tmp.move_direction_offset = ball_ctr_y_displacement;

        std::cout << "lineFollow: move up->" << ball_ctr_y_displacement << std::endl;
      }
      if(ball_ctr_y_displacement < -input_ball_data.ball_ctr_vibrate)
      {
        serial_action = true;
        move_data_tmp.move_direction = 'D';
        move_data_tmp.move_direction_offset = -ball_ctr_y_displacement;

        std::cout << "lineFollow: move down->" << ball_ctr_y_displacement << std::endl;
      }

      if(serial_action == true)
      {
        ubn::moveAction(&serial_prop, &move_data_tmp);

        serial_action = false;
        move_data_tmp = { 'B',0,'N',0,'N',0 };
      }

#ifdef DEBUG
      t = (cv::getTickCount() - t) / cv::getTickFrequency();

      double fps = 1.0 / t;
      char fps_string[10];

      std::sprintf(fps_string, "%.2f", fps);
      std::string put_fps_string("FPS:");
      put_fps_string += fps_string;

      cv::putText(frame, put_fps_string, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
      cv::circle(frame, cv::Point(input_ball_data.ball_ctr_x, input_ball_data.ball_ctr_y), input_ball_data.ball_ctr_vibrate, cv::Scalar(255, 255, 255), 2, 8, 0);
      cv::imshow("frame", frame);

      if(cv::waitKey(stream_prop.waitkey_delay) >= 0)
      {
        break;
      }
#endif

    }

#ifdef DEBUG
    cv::destroyAllWindows();
#endif

    capture.release();

    std::cout << "lineFollow: end" << std::endl;
  }

}

#endif
