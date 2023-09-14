#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// using namespace std::chrono_literals;

class TF2PosePulisher : public rclcpp::Node
{
  public:
    TF2PosePulisher()
    : Node("tf2pose_publisher")
    {
      // Declare and acquire `target_frame` parameter
      target_frame_ = this->declare_parameter<std::string>("target_frame", "rov1");
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
      // Call on_timer function every second
      // timer_ = this->create_wall_timer(500ms, std::bind(&TF2PosePulisher::on_timer, this));

      timer_ = create_wall_timer(
      std::chrono::milliseconds(1000/20), [this]
      {
        on_timer();
      });
      
    }

  private:
    // Parameters
    // BaseContext cxt_;
    rclcpp::Duration transform_expiration_{std::chrono::milliseconds{0}};
    void on_timer()
    {
      
      // Store frame names in variables that will be used to
      // compute transformations
      std::string fromFrameRel = target_frame_.c_str();
      std::string toFrameRel = "world";

      geometry_msgs::msg::TransformStamped transform;
      
      try {
        if(tf_buffer_->canTransform(toFrameRel,fromFrameRel,tf2::TimePointZero)){
          transform = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
        }
            
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
      }


      geometry_msgs::msg::PoseStamped pose_msg;
      
      pose_msg.header.frame_id = "world";
      pose_msg.pose.position.x = 0.0;
      pose_msg.pose.position.y = 0.0;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation.x = 0.0;
      pose_msg.pose.orientation.y = 0.0;
      pose_msg.pose.orientation.z = 0.0;
      pose_msg.pose.orientation.w = 1.0;

      // RCLCPP_INFO(this->get_logger(),"the current pose frame id is %s",pose_msg.header.frame_id.c_str() );
      tf2::doTransform(pose_msg, pose_msg, transform);

      transform_expiration_ = {std::chrono::milliseconds{100}};
      // baselink to left_camera static tf hardcoding
      // 'translation.x': -0.15,
      // 'translation.y': 0.18,
      // 'translation.z': -0.0675,
      // 'rotation.x': 0.0,
      // 'rotation.y': 0.7071067811865475,
      // 'rotation.z': 0.0,
      // 'rotation.w': 0.7071067811865476,
      pose_msg.header.stamp = now() + transform_expiration_;
      pose_msg.header.frame_id = "world";
      // pose_msg.pose.position.x += -0.15;
      // pose_msg.pose.position.y += 0.18;
      // pose_msg.pose.position.z += -0.0675;
      // pose_msg.pose.orientation.x += 0.0;
      // pose_msg.pose.orientation.y += 0.7071067811865475;
      // pose_msg.pose.orientation.z += 0.0;
      // pose_msg.pose.orientation.w += 0.7071067811865476;

      // RCLCPP_INFO(this->get_logger(),"the pose frame id is %s",pose_msg.header.frame_id.c_str() );

      // RCLCPP_INFO(this->get_logger(), "Try to publish");
      pose_publisher_->publish(pose_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
};

int main(int argc, char * argv[])
{ 
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TF2PosePulisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}