/**
  * @name line_follow.hpp
  * @namespace ubn
  * @brief void lineFollow(ubn::SerialProp serial_prop, ubn::StreamProp stream_prop, ubn::ColorRange input_color, ubn::LineData input_line_data)
  * @brief Line follow using OpenCV
  * @param SerialProp, StreamProp, RectProp, ColorRange, LineData
  * @author Unbinilium(iChizer0, Chen Xiangjie)
  * @date 2020-01-12
 **/

#define DEBUG

#ifndef UBN_LINE_FOLLOW_HPP
#define UBN_LINE_FOLLOW_HPP

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

#ifndef UBN_LINE_STRUCT
#define UBN_LINE_STRUCT

    struct LineData
    {
      int area_threshold_min;            //Ball center vibrate (pixels), when larger then send ball location
      int area_threshold_max;            //Line area threshold max (pixels), larger as crossing
      int line_axis_H;                   //Line horizental axis (pixels), determined by camera's location
      int line_axis_H_vibrate;           //Line horizental axis vibrate (pixels), when larger then make movement
      int line_main_V_s;                 //ROI of 1st line follow area start y (up->down)
      int line_main_V_e;                 //ROI of 1st line follow area end y (up->down)
      int line_dist_V_s;                 //ROI of 2nd line follow area start y (up->down)
      int line_dist_V_e;                 //ROI of 2nd line follow area end y (up->down)
      int line_dist_R_vibrate;           //Line angle rotation vibrate (pixels), when larger then make rotatement
      int line_mark_cnt;                 //Count the crossing tag on the line (not including turns)
      int line_mark_cnt_limit;           //When crossing tag count eqaled, end the current function
    };

#endif

#ifndef UBN_CALLBACK
#define UBN_CALLBACK

    void callBack(int, void *)
    {}

#endif

  static void linePosition(cv::Mat *p_input_img, cv::Scalar *p_input_lowerb, cv::Scalar *p_input_upperb, cv::Point2f *p_line_ctr, double *p_area_max)
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
#endif
    }

    contours.clear();
    hierarchy.clear();
  }

  void lineFollow(ubn::SerialProp serial_prop, ubn::StreamProp stream_prop, ubn::ColorRange input_color, ubn::LineData input_line_data)
  {
    bool crossing_flag = false;
    bool straight_flag = false;
    bool serial_action = false;

    ubn::MoveData move_data_tmp = { 'C',stream_prop.camera_angle_V,'D',0,'D',0 };
    ubn::moveAction(&serial_prop, &move_data_tmp);
    ubn::fallBack(&serial_prop, true);

    move_data_tmp = { 'M',0,'D',0,'D',0 };

    cv::VideoCapture capture;
    cv::Mat frame;

    capture.open(stream_prop.camera_id + stream_prop.api_id);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, stream_prop.frame_width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, stream_prop.frame_height);
    capture.set(cv::CAP_PROP_AUTO_EXPOSURE, stream_prop.camrea_exposure_mode);
    capture.set(cv::CAP_PROP_EXPOSURE, stream_prop.camera_exposure);

    std::cout << "lineFolllow: capture set->" << stream_prop.camera_id + stream_prop.api_id << "->" << stream_prop.frame_width << ":" << stream_prop.frame_height << std::endl;

#ifdef DEBUG
    int slider_threshold_limit = stream_prop.frame_width * stream_prop.frame_height;
    std::string slider_console_name = std::string(input_color.color_name + " slider");

    cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
    cv::namedWindow(slider_console_name, cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("a_T_min", slider_console_name, &input_line_data.area_threshold_min, slider_threshold_limit, ubn::callBack);
    cv::createTrackbar("a_T_max", slider_console_name, &input_line_data.area_threshold_max, slider_threshold_limit, ubn::callBack);
    cv::createTrackbar("axis_H", slider_console_name, &input_line_data.line_axis_H, stream_prop.frame_width, ubn::callBack);
    cv::createTrackbar("axis_H_v", slider_console_name, &input_line_data.line_axis_H_vibrate, stream_prop.frame_width, ubn::callBack);
    cv::createTrackbar("T_x_s", slider_console_name, &stream_prop.frame_threshold[0], stream_prop.frame_width, ubn::callBack);
    cv::createTrackbar("T_x_e", slider_console_name, &stream_prop.frame_threshold[1], stream_prop.frame_width, ubn::callBack);
    cv::createTrackbar("T_y_s", slider_console_name, &stream_prop.frame_threshold[2], stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("T_y_e", slider_console_name, &stream_prop.frame_threshold[3], stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("m_V_s", slider_console_name, &input_line_data.line_main_V_s, stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("m_V_e", slider_console_name, &input_line_data.line_main_V_e, stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("d_V_s", slider_console_name, &input_line_data.line_dist_V_s, stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("d_V_e", slider_console_name, &input_line_data.line_dist_V_e, stream_prop.frame_height, ubn::callBack);
    cv::createTrackbar("H_L", slider_console_name, &input_color.lowerb[0], 360, ubn::callBack);
    cv::createTrackbar("H_U", slider_console_name, &input_color.upperb[0], 360, ubn::callBack);
    cv::createTrackbar("S_L", slider_console_name, &input_color.lowerb[1], 255, ubn::callBack);
    cv::createTrackbar("S_U", slider_console_name, &input_color.upperb[1], 255, ubn::callBack);
    cv::createTrackbar("V_L", slider_console_name, &input_color.lowerb[2], 255, ubn::callBack);
    cv::createTrackbar("V_U", slider_console_name, &input_color.upperb[2], 255, ubn::callBack);

    while((capture.isOpened()) && (ubn::fallBack(&serial_prop, false)))

#else
    while((capture.isOpened()) && (ubn::fallBack(&serial_prop, false)) && (input_line_data.line_mark_cnt < input_line_data.line_mark_cnt_limit))
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

      cv::Mat frame_tmp[2];
      cv::Scalar input_lowerb_tmp = cv::Scalar(input_color.lowerb[0], input_color.lowerb[1], input_color.lowerb[2]);
      cv::Scalar input_upperb_tmp = cv::Scalar(input_color.upperb[0], input_color.upperb[1], input_color.upperb[2]);
      cv::Point2f line_ctr[2];
      double area_max[2];

      frame = frame(cv::Range(stream_prop.frame_threshold[2], stream_prop.frame_threshold[3]), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));
      frame_tmp[0] = frame(cv::Range(input_line_data.line_main_V_s, input_line_data.line_main_V_e), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));
      frame_tmp[1] = frame(cv::Range(input_line_data.line_dist_V_s, input_line_data.line_dist_V_e), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));

      ubn::linePosition(&frame_tmp[0], &input_lowerb_tmp, &input_upperb_tmp, &line_ctr[0], &area_max[0]);
      ubn::linePosition(&frame_tmp[1], &input_lowerb_tmp, &input_upperb_tmp, &line_ctr[1], &area_max[1]);

      if((area_max[0] < area_max[1] ? area_max[0] : area_max[1])  > input_line_data.area_threshold_min)
      {
        if(area_max[0] > input_line_data.area_threshold_max)
        {
          if(crossing_flag == false)
          {
            crossing_flag = true;
            straight_flag = false;

            std::cout << "lineFollow: crossing detected->" << ++input_line_data.line_mark_cnt << "->" << area_max[0] << std::endl;
          }
        }
        else
        {
          if(straight_flag == false)
          {
            crossing_flag = false;
            straight_flag = true;

            std::cout << "lineFollow: straight detected" << std::endl;
          }
          else
          {
            int line_axis_H_displacement = int(line_ctr[0].x - input_line_data.line_axis_H);

            if(line_axis_H_displacement > input_line_data.line_axis_H_vibrate)
            {
              serial_action = true;
              move_data_tmp.move_direction = 'L';
              move_data_tmp.move_direction_offset = line_axis_H_displacement;

              std::cout << "lineFollow: move left->" << line_axis_H_displacement << std::endl;
            }

            if(line_axis_H_displacement < -input_line_data.line_axis_H_vibrate)
            {
              serial_action = true;
              move_data_tmp.move_direction = 'R';
              move_data_tmp.move_direction_offset = -line_axis_H_displacement;

              std::cout << "lineFollow: move right->" << line_axis_H_displacement << std::endl;
            }
          }

          int line_dist_R_displacement = int(line_ctr[0].x - line_ctr[1].x);

          if(line_dist_R_displacement > input_line_data.line_axis_H_vibrate)
          {
            serial_action = true;
            move_data_tmp.move_rotation = 'L';
            move_data_tmp.move_rotation_offset = line_dist_R_displacement;

            std::cout << "lineFollow: routate left->" << line_dist_R_displacement << std::endl;
          }

          if(line_dist_R_displacement < -input_line_data.line_axis_H_vibrate)
          {
            serial_action = true;
            move_data_tmp.move_rotation = 'L';
            move_data_tmp.move_rotation_offset = -line_dist_R_displacement;

            std::cout << "lineFollow: routate right->" << line_dist_R_displacement << std::endl;
          }

        }
      }

      if(serial_action == true)
      {
        ubn::moveAction(&serial_prop, &move_data_tmp);

        serial_action = false;
        move_data_tmp = { 'M',0,'D',0,'D',0 };
      }

#ifdef DEBUG
      t = (cv::getTickCount() - t) / cv::getTickFrequency();

      double fps = 1.0 / t;
      char fps_string[10];

      std::sprintf(fps_string, "FPS:%.2f", fps);

      cv::putText(frame, fps_string, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
      cv::line(frame, cv::Point(input_line_data.line_axis_H), cv::Point(input_line_data.line_axis_H, frame.rows), cv::Scalar(255, 255, 255), 2);
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
