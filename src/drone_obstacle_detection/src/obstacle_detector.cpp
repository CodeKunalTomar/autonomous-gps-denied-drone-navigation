#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector() : Node("obstacle_detector")
    {
        // Subscribe to camera depth data
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&ObstacleDetector::depth_callback, this, std::placeholders::_1));
        
        // Publish avoidance commands to PX4
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/fmu/in/setpoint_velocity/cmd_vel_unstamped", 10);
        
        // Obstacle detection parameters
        min_distance_ = 2.0;  // meters - minimum safe distance
        max_distance_ = 10.0; // meters - maximum detection range
        safety_margin_ = 0.5; // meters - additional safety buffer
        
        RCLCPP_INFO(this->get_logger(), "Obstacle Detector started - Safety distance: %.1fm", min_distance_);
    }

private:
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_image = cv_ptr->image;
            
            // Analyze depth data for obstacles
            auto obstacle_info = detect_obstacles(depth_image);
            
            // Generate avoidance command if obstacle detected
            if (obstacle_info.obstacle_detected) {
                generate_avoidance_command(obstacle_info);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }
    
    struct ObstacleInfo {
        bool obstacle_detected = false;
        double min_distance = 0.0;
        std::string direction = "";
        int obstacle_pixels = 0;
    };
    
    ObstacleInfo detect_obstacles(const cv::Mat& depth_image)
    {
        ObstacleInfo info;
        
        // Define detection zones in image
        int height = depth_image.rows;
        int width = depth_image.cols;
        
        // Central detection zone (critical path ahead)
        cv::Rect center_zone(width/4, height/4, width/2, height/2);
        cv::Mat center_roi = depth_image(center_zone);
        
        // Find minimum distance in detection zone
        double min_val, max_val;
        cv::minMaxLoc(center_roi, &min_val, &max_val);
        
        // Convert from mm to meters (assuming depth in mm)
        double min_distance_m = min_val / 1000.0;
        
        // Check if obstacle is too close
        if (min_distance_m > 0.1 && min_distance_m < min_distance_) {
            info.obstacle_detected = true;
            info.min_distance = min_distance_m;
            info.direction = "center";
            
            // Count obstacle pixels for severity assessment
            cv::Mat obstacle_mask;
            cv::inRange(center_roi, 1, min_distance_ * 1000, obstacle_mask);
            info.obstacle_pixels = cv::countNonZero(obstacle_mask);
            
            RCLCPP_WARN(this->get_logger(), 
                "OBSTACLE DETECTED! Distance: %.2fm, Pixels: %d", 
                min_distance_m, info.obstacle_pixels);
        }
        
        return info;
    }
    
    void generate_avoidance_command(const ObstacleInfo& obstacle)
    {
        auto cmd = geometry_msgs::msg::Twist();
        
        // Emergency stop if very close
        if (obstacle.min_distance < 1.0) {
            cmd.linear.x = 0.0;  // Stop forward motion
            cmd.linear.y = 0.0;  // Stop lateral motion  
            cmd.linear.z = 0.0;  // Stop vertical motion
            
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP - Obstacle at %.2fm!", obstacle.min_distance);
        }
        // Slow approach if moderately close
        else if (obstacle.min_distance < min_distance_) {
            cmd.linear.x = -0.2; // Slow backward motion
            cmd.linear.y = 0.1;  // Slight lateral movement
            cmd.linear.z = 0.0;
            
            RCLCPP_WARN(this->get_logger(), "Obstacle avoidance - Backing away slowly");
        }
        
        cmd_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    double min_distance_;
    double max_distance_;
    double safety_margin_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}
