#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ObstacleDetectorDebug : public rclcpp::Node
{
public:
    ObstacleDetectorDebug() : Node("obstacle_detector_debug")
    {
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&ObstacleDetectorDebug::depth_callback, this, std::placeholders::_1));
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/fmu/in/setpoint_velocity/cmd_vel_unstamped", 10);
        
        min_distance_ = 2.0;
        
        RCLCPP_INFO(this->get_logger(), "🚁 DEBUG Obstacle Detector started - Safety distance: %.1fm", min_distance_);
    }

private:
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "📸 Received depth image: %dx%d, encoding: %s", 
                   msg->width, msg->height, msg->encoding.c_str());
        
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_image = cv_ptr->image;
            
            RCLCPP_INFO(this->get_logger(), "🔍 OpenCV image size: %dx%d, type: %d", 
                       depth_image.cols, depth_image.rows, depth_image.type());
            
            // Print some pixel values for debugging
            if (depth_image.rows > 0 && depth_image.cols > 0) {
                uint16_t pixel_val = depth_image.at<uint16_t>(0, 0);
                RCLCPP_INFO(this->get_logger(), "🎯 First pixel value: %d", pixel_val);
                
                double min_val, max_val;
                cv::minMaxLoc(depth_image, &min_val, &max_val);
                RCLCPP_INFO(this->get_logger(), "📏 Min distance: %.2f, Max distance: %.2f", min_val, max_val);
                
                // Convert to meters (assuming raw values are mm)
                double min_distance_m = min_val / 1000.0;
                RCLCPP_INFO(this->get_logger(), "📐 Min distance in meters: %.3fm", min_distance_m);
                
                if (min_distance_m > 0.001 && min_distance_m < min_distance_) {
                    RCLCPP_ERROR(this->get_logger(), "🚨 OBSTACLE DETECTED! Distance: %.3fm", min_distance_m);
                    
                    // Send emergency stop
                    auto cmd = geometry_msgs::msg::Twist();
                    cmd.linear.x = 0.0;
                    cmd.linear.y = 0.0; 
                    cmd.linear.z = 0.0;
                    cmd_pub_->publish(cmd);
                    
                    RCLCPP_ERROR(this->get_logger(), "🛑 EMERGENCY STOP COMMAND SENT!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "✅ No obstacles detected (distance: %.3fm)", min_distance_m);
                }
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ CV Bridge error: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    double min_distance_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectorDebug>());
    rclcpp::shutdown();
    return 0;
}
