#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <vector>
#include <cmath>

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        // Subscribe to current position from PX4
        position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 10,
            std::bind(&PathPlanner::position_callback, this, std::placeholders::_1));
        
        // Subscribe to obstacle avoidance commands
        obstacle_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/fmu/in/setpoint_velocity/cmd_vel_unstamped", 10,
            std::bind(&PathPlanner::obstacle_callback, this, std::placeholders::_1));
        
        // Publish trajectory setpoints to PX4
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        
        // Publish planned path for visualization
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        
        // Path planning timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz planning
            std::bind(&PathPlanner::plan_path, this));
        
        // Initialize waypoints for GPS-denied navigation
        initialize_waypoints();
        current_waypoint_index_ = 0;
        obstacle_detected_ = false;
        
        RCLCPP_INFO(this->get_logger(), "🗺️ Path Planner started - GPS-denied navigation mode");
    }

private:
    void initialize_waypoints()
    {
    // Define waypoints for GPS-denied navigation (relative positions in meters)
    waypoints_.clear();
    
    // Square pattern navigation for testing - FIXED: Create proper Point objects
    geometry_msgs::msg::Point wp1, wp2, wp3, wp4, wp5;
    
    // Start position (2m altitude)
    wp1.x = 0.0; wp1.y = 0.0; wp1.z = -2.0;
    waypoints_.push_back(wp1);
    
    // Move 5m forward
    wp2.x = 5.0; wp2.y = 0.0; wp2.z = -2.0;
    waypoints_.push_back(wp2);
    
    // Move 5m right
    wp3.x = 5.0; wp3.y = 5.0; wp3.z = -2.0;
    waypoints_.push_back(wp3);
    
    // Move 5m back
    wp4.x = 0.0; wp4.y = 5.0; wp4.z = -2.0;
    waypoints_.push_back(wp4);
    
    // Return to start
    wp5.x = 0.0; wp5.y = 0.0; wp5.z = -2.0;
    waypoints_.push_back(wp5);
    
    RCLCPP_INFO(this->get_logger(), "📍 Initialized %zu waypoints for GPS-denied navigation", waypoints_.size());
    }
    
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_position_.x = msg->x;
        current_position_.y = msg->y;
        current_position_.z = msg->z;
        
        // Check if reached current waypoint
        if (current_waypoint_index_ < waypoints_.size()) {
            double distance = calculate_distance(current_position_, waypoints_[current_waypoint_index_]);
            
            if (distance < 0.5) { // 50cm waypoint tolerance
                current_waypoint_index_++;
                if (current_waypoint_index_ < waypoints_.size()) {
                    RCLCPP_INFO(this->get_logger(), "✅ Waypoint %zu reached! Moving to waypoint %zu", 
                               current_waypoint_index_ - 1, current_waypoint_index_);
                } else {
                    RCLCPP_INFO(this->get_logger(), "🎯 All waypoints completed! GPS-denied navigation successful!");
                    current_waypoint_index_ = 0; // Restart pattern
                }
            }
        }
    }
    
    void obstacle_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Detect if obstacle avoidance is active (emergency stop commands)
        if (msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->linear.z == 0.0) {
            if (!obstacle_detected_) {
                obstacle_detected_ = true;
                RCLCPP_WARN(this->get_logger(), "⚠️ Obstacle detected! Pausing path planning for safety");
            }
        } else {
            if (obstacle_detected_) {
                obstacle_detected_ = false;
                RCLCPP_INFO(this->get_logger(), "✅ Path clear! Resuming navigation to waypoint %zu", current_waypoint_index_);
            }
        }
    }
    
    void plan_path()
    {
        // Don't plan if obstacles are detected (safety override)
        if (obstacle_detected_) {
            return;
        }
        
        // Don't plan if no waypoints or completed
        if (current_waypoint_index_ >= waypoints_.size()) {
            return;
        }
        
        // Get current target waypoint
        auto target = waypoints_[current_waypoint_index_];
        
        // Calculate trajectory to waypoint
        auto trajectory_msg = px4_msgs::msg::TrajectorySetpoint();
        trajectory_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        // Set position setpoint
        trajectory_msg.position[0] = target.x;
        trajectory_msg.position[1] = target.y;
        trajectory_msg.position[2] = target.z;
        
        // Calculate velocity towards target
        double dx = target.x - current_position_.x;
        double dy = target.y - current_position_.y;
        double dz = target.z - current_position_.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        // Normalize and scale velocity (max 1 m/s)
        double max_velocity = 1.0;
        if (distance > 0.1) {
            trajectory_msg.velocity[0] = (dx / distance) * std::min(max_velocity, distance);
            trajectory_msg.velocity[1] = (dy / distance) * std::min(max_velocity, distance);
            trajectory_msg.velocity[2] = (dz / distance) * std::min(max_velocity, distance);
        } else {
            trajectory_msg.velocity[0] = 0.0;
            trajectory_msg.velocity[1] = 0.0;
            trajectory_msg.velocity[2] = 0.0;
        }
        
        // Publish trajectory
        trajectory_pub_->publish(trajectory_msg);
        
        // Publish path for visualization
        publish_path();
    }
    
    void publish_path()
    {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";
        
        // Add all remaining waypoints to path
        for (size_t i = current_waypoint_index_; i < waypoints_.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = waypoints_[i].x;
            pose.pose.position.y = waypoints_[i].y;
            pose.pose.position.z = waypoints_[i].z;
            pose.pose.orientation.w = 1.0; // No rotation
            path_msg.poses.push_back(pose);
        }
        
        path_pub_->publish(path_msg);
    }
    
    double calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // Member variables
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<geometry_msgs::msg::Point> waypoints_;
    geometry_msgs::msg::Point current_position_;
    size_t current_waypoint_index_;
    bool obstacle_detected_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}

