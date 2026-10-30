#include <array>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

namespace gazebo {
// The exhibition needs the moving actors, not hundreds of decorative models.
// Publish their exact physics state at the driving controller's 50 Hz cadence.
class ExhibitionCompactState : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    publisher_ = node_->create_publisher<gazebo_msgs::msg::ModelStates>(
      "/exhibition/model_states", rclcpp::SensorDataQoS().keep_last(1));
    connection_ = event::Events::ConnectWorldUpdateEnd([this] { Publish(); });
  }
 private:
  void Publish() {
    const auto now = world_->SimTime();
    // Reset-safe, without repeatedly publishing stale poses between steps.
    if (now >= last_ && (now - last_).Double() < .02) return;
    last_ = now;
    gazebo_msgs::msg::ModelStates message;
    for (const auto *name : names_) {
      // Resolve each time: obstacles and the venue camera may be respawned.
      const auto model = world_->ModelByName(name);
      if (!model) continue;
      const auto pose = model->WorldPose();
      const auto linear = model->WorldLinearVel();
      const auto angular = model->WorldAngularVel();
      geometry_msgs::msg::Pose p;
      p.position.x = pose.Pos().X(); p.position.y = pose.Pos().Y(); p.position.z = pose.Pos().Z();
      p.orientation.x = pose.Rot().X(); p.orientation.y = pose.Rot().Y();
      p.orientation.z = pose.Rot().Z(); p.orientation.w = pose.Rot().W();
      geometry_msgs::msg::Twist t;
      t.linear.x = linear.X(); t.linear.y = linear.Y(); t.linear.z = linear.Z();
      t.angular.x = angular.X(); t.angular.y = angular.Y(); t.angular.z = angular.Z();
      message.name.emplace_back(name);
      message.pose.push_back(p);
      message.twist.push_back(t);
    }
    publisher_->publish(message);
  }
  const std::array<const char *, 7> names_{{"ego_vehicle", "ai_rival", "ai_ghost",
    "exhibition_venue_view", "obstacle1", "obstacle2", "obstacle3"}};
  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<gazebo_msgs::msg::ModelStates>::SharedPtr publisher_;
  event::ConnectionPtr connection_;
  common::Time last_;
};
GZ_REGISTER_WORLD_PLUGIN(ExhibitionCompactState)
}
