#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/convert.h>
#include <tf2/time.h>


class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher_node")
    {
        std::cout << "LAUNCHED ODOM NODE!" << std::endl;

        ips_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/autodrive/f1tenth_1/ips", 10, std::bind(&OdomPublisher::ips_callback, this, std::placeholders::_1));
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/speed", 10, std::bind(&OdomPublisher::speed_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10, std::bind(&OdomPublisher::imu_callback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&OdomPublisher::publish_odom, this));
    }

private:
    void ips_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        std::cout << "IPS\n";
        position_x_ = msg->x;
        position_y_ = msg->y;
        position_z_ = msg->z;
    }

    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // std::cout << "Speed\n";
        linear_speed_ = msg->data;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // std::cout << "IMU\n";
        orientation_ = msg->orientation;
        angular_velocity_ = msg->angular_velocity;
    }

    void publish_odom()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "f1tenth_1"; // Set the child frame

        odom_msg.pose.pose.position.x = position_x_;
        odom_msg.pose.pose.position.y = position_y_;
        odom_msg.pose.pose.position.z = position_z_;
        odom_msg.pose.pose.orientation = orientation_;

        odom_msg.twist.twist.linear.x = linear_speed_;
        odom_msg.twist.twist.angular = angular_velocity_;

        odom_pub_->publish(odom_msg);
        // std::cout << "ODOM\n";

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "f1tenth_1";
        transform.transform.translation.x = position_x_;
        transform.transform.translation.y = position_y_;
        transform.transform.translation.z = position_z_;
        transform.transform.rotation = orientation_;

        tf_broadcaster_->sendTransform(transform);

    }

    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double position_z_ = 0.0;
    double linear_speed_ = 0.0;

    geometry_msgs::msg::Quaternion orientation_;
    geometry_msgs::msg::Vector3 angular_velocity_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
