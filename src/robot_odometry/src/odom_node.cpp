#include "robot_odometry/odom_node.hpp"

OdometryNode::OdometryNode() : Node("odometry_node") {
    sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "velocity", 10, std::bind(&OdometryNode::velocityCallback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = this->get_clock()->now();
}

void OdometryNode::velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    rclcpp::Time current_time = msg->header.stamp;
    double dt = (current_time - last_time_).seconds();

    if (dt <= 0) { last_time_ = current_time; return; }
    double vx = msg->twist.linear.x;
    double vth = msg->twist.angular.z;

    double delta_x = (vx * cos(th_)) * dt;
    double delta_y = (vx * sin(th_)) * dt;
    double delta_th = vth * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;

    odom_pub_->publish(odom);

    
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

    last_time_ = current_time;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}