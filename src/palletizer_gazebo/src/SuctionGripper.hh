#ifndef PALLETIZER_GAZEBO__SUCTION_GRIPPER_HH_
#define PALLETIZER_GAZEBO__SUCTION_GRIPPER_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace palletizer_gazebo
{
class SuctionGripperPrivate;

/// \brief Suction gripper that attaches to boxes spawned at runtime.
///
/// Unlike gz::sim::systems::DetachableJoint, the child model is not fixed at load time:
/// on an "on" command the plugin searches all models whose name starts with
/// <model_prefix>, picks the box whose top face is right under the cup, and creates a
/// fixed joint (components::DetachableJoint, built by the Physics system) between the
/// cup link and that box. "off" removes the joint.
///
/// Parameters (all optional except link_name):
///   <link_name>       cup link; its origin is the cup face center, +z points out of the cup
///   <model_prefix>    only models with this name prefix are graspable (default "box_")
///   <max_gap>         max distance [m] from cup face to the box top face (default 0.008)
///   <max_offset>      max lateral distance [m] from cup axis to the top-face center (default 0.02)
///   <max_tilt>        max angle [rad] between cup axis and top-face normal (default 0.15)
///   <cmd_topic>       gz.msgs.Boolean, true = suction on (default /suction/cmd)
///   <grasped_topic>   gz.msgs.Boolean state output (default /suction/grasped)
///   <attached_topic>  gz.msgs.StringMsg, attached model name or "" (default /suction/attached_model)
///   <publish_rate>    state publish rate [Hz], also published on every change (default 10)
class SuctionGripper
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  SuctionGripper();
  ~SuctionGripper() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  std::unique_ptr<SuctionGripperPrivate> dataPtr;
};
}  // namespace palletizer_gazebo

#endif  // PALLETIZER_GAZEBO__SUCTION_GRIPPER_HH_
