#include "robot_odometry/odom_node.hpp"

OdometryNode::OdometryNode() : Node("odometry_node") {
    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "velocity", 10, std::bind(&OdometryNode::velocityCallback, this, std::placeholders::_1)
    );

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&OdometryNode::imuCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = this->get_clock()->now();
}

void OdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Quaternion q(
        msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    th_ = yaw;
}

void OdometryNode::velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    rclcpp::Time current_time = msg->header.stamp;
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0) { last_time_ = current_time; return; }

    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;

    x_ += (vx * cos(th_) - vy * sin(th_)) * dt;
    y_ += (vx * sin(th_) + vy * cos(th_)) * dt;

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

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.02;
    odom.pose.covariance[35] = 0.001;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped t;
    t.header = odom.header;
    t.child_frame_id = odom.child_frame_id;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);

    last_time_ = current_time;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}