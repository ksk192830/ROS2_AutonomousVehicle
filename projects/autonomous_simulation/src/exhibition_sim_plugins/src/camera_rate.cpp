#include <atomic>
#include <cmath>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo {
// Change capture cadence at a render boundary, without unloading the camera
// or doing a request/response round trip for every individual image.
class ExhibitionCameraRate : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
    if (!sensor_) return;
    requested_ = sdf->Get<double>("initial_rate", sensor_->UpdateRate()).first;
    node_ = gazebo_ros::Node::Get(sdf);
    subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
      sdf->Get<std::string>("rate_topic"), rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Float64::ConstSharedPtr message) {
        if (std::isfinite(message->data) && message->data >= 0 && message->data <= 240)
          requested_.store(message->data);
      });
    connection_ = event::Events::ConnectPreRender([this] {
      const double rate = requested_.load();
      if (rate == applied_) return;
      // Zero means disabled here; Gazebo itself interprets a zero rate as
      // unlimited, so only pass positive rates to SetUpdateRate.
      if (rate > 0) sensor_->SetUpdateRate(rate);
      sensor_->SetActive(rate > 0);
      applied_ = rate;
    });
  }
 private:
  sensors::CameraSensorPtr sensor_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  event::ConnectionPtr connection_;
  std::atomic<double> requested_{0};
  double applied_{-1};
};
GZ_REGISTER_SENSOR_PLUGIN(ExhibitionCameraRate)
}
