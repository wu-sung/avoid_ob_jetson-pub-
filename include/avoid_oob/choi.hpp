#ifndef _CHOI_HPP_
#define _CHOI_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include "avoid_oob/dxl.hpp"
#include <functional>
#include <memory>
#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;
class Pub : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
        void publish_msg();
        
        std::string src = "nvarguscamerasrc sensor-id=0 ! \
            video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
            format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
            width=(int)640, height=(int)360, format=(string)BGRx ! \
            videoconvert ! video/x-raw, format=(string)BGR ! appsink"; 
        
        std_msgs::msg::Header hdr;
        sensor_msgs::msg::CompressedImage::SharedPtr msg;
        cv::VideoCapture cap;
        cv::Mat frame,real_frame;

        
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
        void mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg);
        
        double gain = 0.65;
    public:
        Pub();
        Dxl dxl;
        int lvel, rvel, err;
};
#endif //_PUB_HPP_

