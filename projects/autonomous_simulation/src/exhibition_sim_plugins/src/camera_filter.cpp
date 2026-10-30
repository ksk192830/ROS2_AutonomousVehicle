#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/ogre_gazebo.h>

namespace gazebo {
// Human and AI share the road, but the human is not part of the AI's obstacle
// course. Physics uses separate collision categories; only the AI camera omits
// the human visual. The participant camera and overhead camera still see both.
class ExhibitionCameraFilter : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr) override {
    sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
    if (!sensor_) { gzerr << "ExhibitionCameraFilter needs a camera\n"; return; }
    connection_ = event::Events::ConnectPreRender([this] { Filter(); });
  }
 private:
  static constexpr uint32_t human_flag = 1u << 24;
  sensors::CameraSensorPtr sensor_;
  event::ConnectionPtr connection_;
  void Tag(const rendering::VisualPtr &visual) {
    visual->SetVisibilityFlags(human_flag);
    for (unsigned int i = 0; i < visual->GetChildCount(); ++i) Tag(visual->GetChild(i));
  }
  void Filter() {
    auto camera = sensor_->Camera();
    if (!camera || !camera->OgreViewport()) return;
    auto visual = camera->GetScene()->GetVisual("ego_vehicle");
    if (visual) Tag(visual);
    auto viewport = camera->OgreViewport();
    viewport->setVisibilityMask(viewport->getVisibilityMask() & ~human_flag);
  }
};
GZ_REGISTER_SENSOR_PLUGIN(ExhibitionCameraFilter)
}
