/**
  * @name color_area.hpp
  * @namespace ubn
  * @brief unsigned int colorArea(ubn::StreamProp stream_prop, ubn::RectProp rect_prop, std::vector<ubn::ColorRange> color)
  * @brief Find the maxium color area from camera
  * @param StreamProp, RectProp, ColorRange
  * @return Unsigned integer matches the color name index
  * @author Unbinilium(iChizer0, Chen Xiangjie)
  * @date 2020-01-12
 **/

#define DEBUG

#ifndef UBN_COLOR_AREA_HPP
#define UBN_COLOR_AREA_HPP

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

#ifndef UBN_CALLBACK
#define UBN_CALLBACK

    void callBack(int, void *)
    {}

#endif

    template<typename T>
    static auto calPointDist(T &p1, T &p2)
    {
        int a = int(p1.x - p2.x), b = int(p1.y - p2.y);

        return std::sqrt(a * a + b * b);
    }

    template<typename T>
    static auto calIntersect(T &a, T &b)
    {
        cv::Point2f pt;
        int S[4] = { a[0] - a[2], a[1] - a[3], b[0] - b[2], b[1] - b[3] };
        int A = a[0] * a[3] - a[1] * a[2], B = b[0] * b[3] - b[1] * b[2], C = S[0] * S[3] - S[1] * S[2];

        pt.x = (A * S[2] - B * S[0]) / C;
        pt.y = (A * S[3] - B * S[1]) / C;

        return pt;
    }

    template<typename T>
    static auto calAngle(T &pt1, T &pt2, T &pt0)
    {
        double dx1 = pt1.x - pt0.x, dy1 = pt1.y - pt0.y, dx2 = pt2.x - pt0.x, dy2 = pt2.y - pt0.y;

        return (dx1 * dx2 + dy1 * dy2) / std::sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
    }

    static auto findRects(cv::Mat &input_image, cv::Mat &output_image, std::vector<std::vector<cv::Point>> &rects, ubn::RectProp &rect_prop)
    {
        cv::Mat input_image_tmp, input_image_pyr, input_image_gray, input_image_channel(input_image.size(), CV_8U);

        cv::pyrDown(input_image, input_image_pyr, cv::Size(input_image.cols / 2, input_image.rows / 2));
        cv::pyrUp(input_image_pyr, input_image_pyr, input_image.size());

        std::vector<std::vector<cv::Point>> contours;

        for(int i = 0; i != 3; i++)
        {
            int channel[] = {i, 0};

            cv::mixChannels(&input_image_pyr, 1, &input_image_channel, 1, channel, 1);
            cv::Canny(input_image_channel, input_image_gray, 0, rect_prop.canny_threshold, 5);
            cv::dilate(input_image_gray, input_image_gray, cv::Mat(), cv::Point(-1, -1));

            for(int j = 0; j != rect_prop.contour_depth; j++)
            {
                input_image_gray = cv::Mat(input_image_channel >= (j + 1) * 255 / rect_prop.contour_depth);

                cv::findContours(input_image_gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

                std::vector<cv::Point> approx;

                for(auto& contour : contours)
                {
                    cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * rect_prop.approx_epsilon, true);

                    if(approx.size() == 4 && std::fabs(cv::contourArea(approx)) > rect_prop.area_threshold && cv::isContourConvex(approx))
                    {
                        double cosine_max = 0;

                        for(int l = 2; l != 5; l++)
                        {
                            double cosine = std::fabs(ubn::calAngle(approx[uint(l % 4)], approx[uint(l - 2)], approx[uint(l - 1)]));

                            cosine_max = MAX(cosine_max, cosine);
                        }

                        if(cosine_max < rect_prop.cosine_max_threshold)
                        {
                            rects.push_back(approx);
                        }
                    }
                }

                contours.clear();
                approx.clear();
            }
        }

       if(rects.size() == 0)
        {
            return false;
        }

        unsigned int area_max_tag = 0;
        double area_cur_tmp = 0, area_max_tmp = 0;

        for(unsigned int i = 0; i != rects.size(); i++)
        {
            area_cur_tmp = cv::contourArea(rects[i]);

            if(area_cur_tmp > area_max_tmp)
            {
                area_max_tmp = area_cur_tmp;
                area_max_tag = i;
            }
        }

        std::vector<cv::Vec4i> edges;

        for(unsigned int i = 0; i != rects[area_max_tag].size(); i++)
        {
            cv::Point p1 = rects[area_max_tag][i];
            cv::Point p2 = rects[area_max_tag][(i + 1) % rects[area_max_tag].size()];

            if(ubn::calPointDist(p1, p2) > 2 * cv::arcLength(rects[area_max_tag], true) * rect_prop.approx_epsilon)
            {
                edges.push_back(cv::Vec4i(p1.x, p1.y, p2.x, p2.y));
            }
        }

        std::vector<cv::Point> corners;

        for(unsigned int i = 0; i != edges.size(); i++)
        {
            cv::Point cornor = ubn::calIntersect(edges[i], edges[(i + 1) % edges.size()]);

            corners.push_back(cornor);
        }

        edges.clear();

        std::vector<cv::Point2f> corners2f;
        std::transform(corners.begin(), corners.end(), std::back_inserter(corners2f), [](const cv::Point &p) { return cv::Point2f(p); });

        corners.clear();

        if(corners2f.size() != 0)
        {
            unsigned int corner_index = 0;
            cv::Point2f corner_tmp = corners2f[0];

            for(unsigned int i = 1; i != corners2f.size(); i++)
            {
                if(corner_tmp.x > corners2f[i].x)
                {
                    corner_tmp = corners2f[i];
                    corner_index = i;
                }
            }

            corners2f[corner_index] = corners2f[0];
            corners2f[0] = corner_tmp;

            for(unsigned int i = 1; i != corners2f.size(); i++)
            {
                for(unsigned int j = i + 1; j != corners2f.size(); j++)
                {
                    if((corners2f[i].y - corners2f[0].y) / (corners2f[i].x - corners2f[0].x) > (corners2f[j].y - corners2f[0].y) / (corners2f[j].x - corners2f[0].x))
                    {
                        cv::Point2f tmp = corners2f[i];
                        corners2f[i] = corners2f[j];
                        corners2f[j] = tmp;
                    }
                }
            }
        }

        cv::Point2f anchor[4] = { corners2f[0],
                                  corners2f[1],
                                  corners2f[2],
                                  corners2f[3]
                                };
        double anchor_dist[4] = { ubn::calPointDist(anchor[0], anchor[1]),
                                  ubn::calPointDist(anchor[1], anchor[2]),
                                  ubn::calPointDist(anchor[2], anchor[3]),
                                  ubn::calPointDist(anchor[3], anchor[0])
                                 };
        int width = int(MAX(anchor_dist[1], anchor_dist[3]));
        int height = int(MAX(anchor_dist[0], anchor_dist[2]));

        if(width >= input_image.cols || height >= input_image.rows)
        {
            return false;
        }

        input_image_tmp = input_image.clone();
        output_image = cv::Mat::zeros(height, width, CV_8UC3);

        std::vector<cv::Point2f> output_image_anchor;

        output_image_anchor.push_back(cv::Point2f(0, output_image.rows));
        output_image_anchor.push_back(cv::Point2f(0, 0));
        output_image_anchor.push_back(cv::Point2f(output_image.cols, 0));
        output_image_anchor.push_back(cv::Point2f(output_image.cols, output_image.rows));

        cv::warpPerspective(input_image_tmp, output_image, cv::getPerspectiveTransform(corners2f , output_image_anchor), output_image.size());

        corners2f.clear();
        output_image_anchor.clear();

        return true;
    }

    static void squareofColorArea(cv::Mat &p_input_img, cv::Scalar &p_input_lowerb, cv::Scalar &p_input_upperb, double &p_area_sum, int &p_threshold)
    {
        p_area_sum = 0;
        double area_tmp;

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::inRange(p_input_img, p_input_lowerb, p_input_upperb, p_input_img);
        cv::findContours(p_input_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for(auto &contour : contours)
        {
            area_tmp = cv::contourArea(contour);
            if(area_tmp > p_threshold)
            {
                p_area_sum += area_tmp;
            }
        }

        contours.clear();
        hierarchy.clear();
    }

    class ColorArea
    {
    public:
        double area_sum = 0;

        void inRangeColorArea(cv::Mat &input_img, ubn::ColorRange &input_color)
        {
            input_img_tmp = input_img.clone();
            input_lowerb_tmp = cv::Scalar(input_color.lowerb[0], input_color.lowerb[1], input_color.lowerb[2]);
            input_upperb_tmp = cv::Scalar(input_color.upperb[0], input_color.upperb[1], input_color.upperb[2]);

            ubn::squareofColorArea(input_img_tmp, input_lowerb_tmp, input_upperb_tmp, area_sum, input_color.threshold);
        }

        void inRangeColorView(const std::string window_name)
        {
            cv::imshow(window_name, input_img_tmp);
        }

    private:
        cv::Mat input_img_tmp;
        cv::Scalar input_lowerb_tmp;
        cv::Scalar input_upperb_tmp;
    };

    unsigned int colorArea(ubn::StreamProp stream_prop, ubn::RectProp rect_prop, std::vector<ubn::ColorRange> color)
    {
        unsigned int color_area_max_tag = 0;

        cv::VideoCapture capture;
        cv::Mat frame;

        std::vector<ubn::ColorArea> color_area(color.size());

        capture.open(stream_prop.camera_id + stream_prop.api_id);
        capture.set(cv::CAP_PROP_FRAME_WIDTH, stream_prop.frame_width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, stream_prop.frame_height);
        capture.set(cv::CAP_PROP_AUTO_EXPOSURE, stream_prop.camrea_exposure_mode);
        capture.set(cv::CAP_PROP_EXPOSURE, stream_prop.camera_exposure);

        std::cout << "colorArea: capture set->" << stream_prop.camera_id + stream_prop.api_id << "->" << stream_prop.frame_width << ":" << stream_prop.frame_height << std::endl;

#ifdef DEBUG
        int cur_color = 0;
        int cur_color_tag = int(color.size() - 1);

        std::string cur_name;
        std::string cur_slider_name;

        cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("capture", cv::WINDOW_AUTOSIZE);
        cv::namedWindow(color[uint(cur_color)].color_name, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(std::string(color[uint(cur_color)].color_name + " slider"), cv::WINDOW_AUTOSIZE);

        while(capture.isOpened())
#else
        std::vector<unsigned int> color_area_freq(color.size(), 0);

        while(capture.isOpened() && stream_prop.read_count--)
#endif

        {
            capture.read(frame);

            if(frame.empty())
            {
                continue;
            }

            frame = frame(cv::Range(stream_prop.frame_threshold[2], stream_prop.frame_threshold[3]), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));

#ifdef DEBUG
            cv::imshow("capture", frame);
#endif

            std::vector<std::vector<cv::Point>> rects;

            if(findRects(frame, frame, rects, rect_prop) == false)
            {
                continue;
            }

            rects.clear();

            cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV_FULL);

            double color_area_max = 0;

            for(unsigned int i = 0; i != color.size(); i++)
            {
                color_area[i].inRangeColorArea(frame, color[i]);
            }

            for(unsigned int i = 0; i != color.size(); i++)
            {
                if(color_area[i].area_sum > color_area_max)
                {
                    color_area_max = color_area[i].area_sum;
                    color_area_max_tag = i;
                }
            }

            if(color_area[color_area_max_tag].area_sum < color[color_area_max_tag].threshold)
            {
                break;
            }

            std::cout << "colorArea: " << color[color_area_max_tag].color_name << "->" << color_area[color_area_max_tag].area_sum << std::endl;

#ifdef DEBUG
            if(cur_color_tag != cur_color)
            {
                cur_name = color[u_int(cur_color)].color_name;
                cur_slider_name = cur_name + " slider";

                if(int(cv::getWindowProperty(color[u_int(cur_color_tag)].color_name, cv::WINDOW_AUTOSIZE)) != -1)
                {
                    cv::destroyWindow(std::string(color[u_int(cur_color_tag)].color_name));
                    cv::namedWindow(cur_name, cv::WINDOW_AUTOSIZE);
                }

                if(int(cv::getWindowProperty(std::string(color[u_int(cur_color_tag)].color_name + " slider"), cv::WINDOW_AUTOSIZE)) != -1)
                {
                    cv::destroyWindow(std::string(color[u_int(cur_color_tag)].color_name + " slider"));
                    cv::namedWindow(cur_slider_name, cv::WINDOW_AUTOSIZE);
                }

                cv::createTrackbar("Color", cur_slider_name, &cur_color, int(color.size() - 1), ubn::callBack);

                cv::createTrackbar("T_Area", cur_slider_name, &color[u_int(cur_color)].threshold, stream_prop.frame_width * stream_prop.frame_height, ubn::callBack);

                cv::createTrackbar("T_x_s", cur_slider_name, &stream_prop.frame_threshold[0], stream_prop.frame_width, ubn::callBack);
                cv::createTrackbar("T_x_e", cur_slider_name, &stream_prop.frame_threshold[1], stream_prop.frame_width, ubn::callBack);

                cv::createTrackbar("T_y_s", cur_slider_name, &stream_prop.frame_threshold[2], stream_prop.frame_height, ubn::callBack);
                cv::createTrackbar("T_y_e", cur_slider_name, &stream_prop.frame_threshold[3], stream_prop.frame_height, ubn::callBack);

                cv::createTrackbar("H_L", cur_slider_name, &color[u_int(cur_color)].lowerb[0], 360, ubn::callBack);
                cv::createTrackbar("H_U", cur_slider_name, &color[u_int(cur_color)].upperb[0], 360, ubn::callBack);

                cv::createTrackbar("S_L", cur_slider_name, &color[u_int(cur_color)].lowerb[1], 255, ubn::callBack);
                cv::createTrackbar("S_U", cur_slider_name, &color[u_int(cur_color)].upperb[1], 255, ubn::callBack);

                cv::createTrackbar("V_L", cur_slider_name, &color[u_int(cur_color)].lowerb[2], 255, ubn::callBack);
                cv::createTrackbar("V_U", cur_slider_name, &color[u_int(cur_color)].upperb[2], 255, ubn::callBack);
            }

            cur_color_tag = cur_color;

            cv::cvtColor(frame, frame, cv::COLOR_HSV2BGR_FULL);

            color_area[uint(cur_color)].inRangeColorView(cur_name);
            cv::putText(frame, std::string("max:" + color[color_area_max_tag].color_name), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::imshow("frame", frame);
#else
            color_area_freq[color_area_max_tag]++;
#endif

            if(cv::waitKey(stream_prop.waitkey_delay) >= 0)
            {
                break;
            }
        }

        cv::destroyAllWindows();
        color_area.clear();

#ifdef DEBUG
        if(capture.isOpened())
        {
            capture.release();

            std::cout << "colorArea: " << stream_prop.read_count << "->" << color_area_max_tag << std::endl;
            std::cout << "colorArea: ended->" << stream_prop.read_count << std::endl;

            return color_area_max_tag;
        }
        else
        {
            std::cerr << "colorArea: capture not opened" << std::endl;
            std::cout << "colorArea: ended->" << stream_prop.read_count << std::endl;

            return color_area_max_tag;
        }
#else
        if(capture.isOpened())
        {
            capture.release();

            unsigned int color_area_freq_max_tag = 0;

            for(unsigned int i = 0; i < color_area_freq.size(); i++)
            {
                if(color_area_freq[i] > color_area_freq[color_area_freq_max_tag])
                {
                    color_area_freq_max_tag = i;
                }
            }

            std::cout << "colorArea: " << "maxarea" << "->" << color[color_area_freq_max_tag].color_name << ":" << color_area_freq[color_area_freq_max_tag] << std::endl;
            std::cout << "colorArea: ended->" << stream_prop.read_count << std::endl;

            color_area_freq.clear();

            return color_area_freq_max_tag;
        }
        else
        {
            std::cerr << "colorArea: capture not opened" << std::endl;
            std::cout << "colorArea: ended->" << stream_prop.read_count << std::endl;

            return color_area_max_tag;
        }
#endif

    }
}

#endif
