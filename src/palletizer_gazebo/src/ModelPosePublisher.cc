// World system: publishes ground-truth world poses of all models whose name starts with
// <model_prefix> as one gz.msgs.Pose_V. Each pose carries header data "frame_id" (world
// frame) and "child_frame_id" (model name), so ros_gz_bridge's Pose_V -> TFMessage
// conversion keeps the names. (SceneBroadcaster's pose/info only fills pose.name, which
// the bridge drops.)
//
// Parameters:
//   <model_prefix>  default "box_"
//   <topic>         default "/sim/model_poses"
//   <frame_id>      default "world"
//   <update_rate>   Hz, sim time, default 30

#include <chrono>
#include <memory>
#include <string>

#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace palletizer_gazebo
{
class ModelPosePublisher
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->world = _entity;
    this->prefix = _sdf->Get<std::string>("model_prefix", "box_").first;
    this->frameId = _sdf->Get<std::string>("frame_id", "world").first;
    const auto topic = _sdf->Get<std::string>("topic", "/sim/model_poses").first;
    const double rate = _sdf->Get<double>("update_rate", 30.0).first;
    this->period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / (rate > 0 ? rate : 30.0)));
    this->pub = this->node.Advertise<gz::msgs::Pose_V>(topic);
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.simTime - this->last < this->period && _info.simTime >= this->last) {
      return;
    }
    this->last = _info.simTime;

    gz::msgs::Pose_V msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(gz::msgs::Convert(_info.simTime));
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name,
      gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity & _model, const gz::sim::components::Model *,
      const gz::sim::components::Name * _name,
      const gz::sim::components::ParentEntity * _parent) -> bool
      {
        if (_parent->Data() != this->world || _name->Data().rfind(this->prefix, 0) != 0) {
          return true;
        }
        auto * pose = msg.add_pose();
        gz::msgs::Set(pose, gz::sim::worldPose(_model, _ecm));
        pose->set_name(_name->Data());
        pose->set_id(_model);
        auto * header = pose->mutable_header();
        header->mutable_stamp()->CopyFrom(msg.header().stamp());
        auto * frame = header->add_data();
        frame->set_key("frame_id");
        frame->add_value(this->frameId);
        auto * child = header->add_data();
        child->set_key("child_frame_id");
        child->add_value(_name->Data());
        return true;
      });
    this->pub.Publish(msg);
  }

private:
  gz::sim::Entity world{gz::sim::kNullEntity};
  std::string prefix;
  std::string frameId;
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  std::chrono::steady_clock::duration period{};
  std::chrono::steady_clock::duration last{0};
};
}  // namespace palletizer_gazebo

GZ_ADD_PLUGIN(
  palletizer_gazebo::ModelPosePublisher,
  gz::sim::System,
  palletizer_gazebo::ModelPosePublisher::ISystemConfigure,
  palletizer_gazebo::ModelPosePublisher::ISystemPostUpdate)
