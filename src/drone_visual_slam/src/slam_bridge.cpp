#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class SLAMBridge : public rclcpp::Node
{
public:
    SLAMBridge() : Node("slam_bridge")
    {
        // Subscribe to rtabmap pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/rtabmap/mapData", 10,
            std::bind(&SLAMBridge::pose_callback, this, std::placeholders::_1));
        
        // Subscribe to visual odometry
        vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/visual_odometry/odom", 10,
            std::bind(&SLAMBridge::vo_callback, this, std::placeholders::_1));
        
        // Publish to PX4 - FIXED: using VehicleOdometry
        px4_vo_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_odometry", 10);
    }

private:
    void vo_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // FIXED: using VehicleOdometry
        auto px4_vo = px4_msgs::msg::VehicleOdometry();
        px4_vo.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        // Convert pose
        px4_vo.position[0] = msg->pose.pose.position.x;
        px4_vo.position[1] = msg->pose.pose.position.y;
        px4_vo.position[2] = msg->pose.pose.position.z;
        
        px4_vo.q[0] = msg->pose.pose.orientation.w;
        px4_vo.q[1] = msg->pose.pose.orientation.x;
        px4_vo.q[2] = msg->pose.pose.orientation.y;
        px4_vo.q[3] = msg->pose.pose.orientation.z;
        
        px4_vo_pub_->publish(px4_vo);
    }
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "SLAM pose updated: [%.2f, %.2f, %.2f]",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
    // FIXED: using VehicleOdometry
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_vo_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMBridge>());
    rclcpp::shutdown();
    return 0;
}
