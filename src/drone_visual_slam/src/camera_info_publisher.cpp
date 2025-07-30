#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher() : Node("camera_info_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // 30 FPS
            std::bind(&CameraInfoPublisher::publish_camera_info, this));
    }

private:
    void publish_camera_info()
    {
        auto msg = sensor_msgs::msg::CameraInfo();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "camera_link";
        
        // Camera parameters for 640x480 test image
        msg.width = 640;
        msg.height = 480;
        msg.distortion_model = "plumb_bob";
        
        // Camera matrix (K)
        msg.k = {525.0, 0.0, 320.0,
                 0.0, 525.0, 240.0,
                 0.0, 0.0, 1.0};
        
        // Distortion coefficients
        msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Rectification matrix (R)
        msg.r = {1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0};
        
        // Projection matrix (P)
        msg.p = {525.0, 0.0, 320.0, 0.0,
                 0.0, 525.0, 240.0, 0.0,
                 0.0, 0.0, 1.0, 0.0};
        
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}
