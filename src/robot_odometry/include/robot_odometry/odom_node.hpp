#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode();


private:
    void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time last_time_;
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
};


#endif