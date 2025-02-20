#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class OdomTfPublisher : public rclcpp::Node
{
public:
  OdomTfPublisher()
  : Node("odom_tf_publisher")
  {
    // Create a TransformBroadcaster to send transforms over tf2
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to the odom topic
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdomTfPublisher::odom_callback, this, _1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    // Use the timestamp from the odometry message
    transformStamped.header.stamp = msg->header.stamp;
    // The parent frame is odom
    transformStamped.header.frame_id = "odom";
    // The child frame is base_link
    transformStamped.child_frame_id = "base_link";

    // Copy the position from the odometry message
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    // Copy the orientation from the odometry message
    transformStamped.transform.rotation = msg->pose.pose.orientation;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transformStamped);

    RCLCPP_DEBUG(this->get_logger(), "Broadcasted tf from odom to base_link");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTfPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
