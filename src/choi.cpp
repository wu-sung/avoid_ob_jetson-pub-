#include "avoid_oob/choi.hpp"

// Pub 클래스 생성자
Pub::Pub() : Node("campub")
{
    // QoS 프로파일 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // 이미지 퍼블리셔 생성
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    
    // 타이머 설정: 25ms마다 publish_msg 함수를 호출
    timer_ = this->create_wall_timer(25ms, std::bind(&Pub::publish_msg, this));
    
    // 비디오 캡처 초기화 (파일에서 읽기)
    cap.open("/home/jetson/ros2_ws/src/avoid_oob/src/avav.mp4");
    
    // GStreamer 파이프라인을 사용한 비디오 캡처 초기화 (주석처리됨)
    // cap.open(src, cv::CAP_GSTREAMER);
    
    // 비디오 캡처가 열리지 않으면 에러 로그를 남기고 ROS2 시스템 종료
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return;
    }

    // 다이나믹셀 초기화
    if(!dxl.open())
    {
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        //return -1;
    } 
    
    // 구독 콜백 함수 설정
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(&Pub::mysub_callback, this, dxl, _1);
    
    // 구독자 생성
    sub_ = this->create_subscription<std_msgs::msg::Int32>("err", qos_profile, fn);
}

// publish_msg 함수: 비디오 프레임을 읽고 퍼블리시
void Pub::publish_msg()
{
    cap >> frame;
    
    // 프레임이 비어 있으면 에러 로그를 남기고 함수 종료
    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "frame empty");
        return;
    }
    
    // 프레임을 압축 이미지 메시지로 변환하여 퍼블리시
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    pub_->publish(*msg);
}

// 구독 콜백 함수: 메시지를 수신하여 다이나믹셀 모터 속도를 제어
void Pub::mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg)
{
    printf("RECEIVED!");
    
    // 수신한 에러 메시지에 따른 속도 제어
    err = intmsg->data;
    if(err == 100){
        lvel = 100; // 왼쪽 바퀴 속도
        rvel = 100; // 오른쪽 바퀴 속도
    }
    else if(err == 200){
        lvel = 50; // 왼쪽 바퀴 속도
        rvel = 50; // 오른쪽 바퀴 속도
    }
    else if(err == 300){
        lvel = -50; // 왼쪽 바퀴 속도
        rvel = -50; // 오른쪽 바퀴 속도
    }
    else if(err == 400){
        lvel = 50; // 왼쪽 바퀴 속도
        rvel = 50; // 오른쪽 바퀴 속도
    }
    else if(err == 500){
        lvel = -50; // 왼쪽 바퀴 속도
        rvel = -50; // 오른쪽 바퀴 속도
    }
    else {
        // 에러 값에 비례하여 속도 조정
        lvel = 100 - gain * -err; // 왼쪽 바퀴 속도
        rvel = -(101 + gain * -err); // 오른쪽 바퀴 속도
    }

    // 로그 메시지 출력
    RCLCPP_INFO(this->get_logger(), "Received message: %d %d", lvel, rvel);
    
    // 다이나믹셀 모터 속도 설정
    mdxl.setVelocity(lvel, rvel);
}
