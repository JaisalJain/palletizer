#include "SuctionGripper.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

namespace palletizer_gazebo
{
namespace
{
constexpr gz::sim::Entity kNull = gz::sim::kNullEntity;

struct Candidate
{
  gz::sim::Entity model{kNull};
  gz::sim::Entity link{kNull};
  std::string name;
  double gap{0.0};
  double offset{0.0};
  double tilt{0.0};
};
}  // namespace

class SuctionGripperPrivate
{
public:
  void OnCmd(const gz::msgs::Boolean & _msg)
  {
    this->requested = _msg.data();
    this->pendingCmd = true;
  }

  /// Returns the best box under the cup, or nullopt. Logs why each nearby box was rejected.
  std::optional<Candidate> FindCandidate(const gz::sim::EntityComponentManager & _ecm) const;

  void Publish();

  gz::sim::Entity model{kNull};
  gz::sim::Entity cupLink{kNull};
  std::string modelName;
  std::string linkName;
  std::string modelPrefix{"box_"};
  double maxGap{0.008};
  double maxOffset{0.02};
  double maxTilt{0.15};
  // Box tops slightly pushed into by the cup still count (contact compliance).
  double minGap{-0.01};

  gz::transport::Node node;
  gz::transport::Node::Publisher graspedPub;
  gz::transport::Node::Publisher attachedPub;

  std::atomic<bool> requested{false};
  std::atomic<bool> pendingCmd{false};

  // Only touched from the simulation thread.
  gz::sim::Entity jointEntity{kNull};
  gz::sim::Entity attachedLink{kNull};
  std::string attachedName;
  bool stateChanged{true};
  std::chrono::steady_clock::duration publishPeriod{std::chrono::milliseconds(100)};
  std::chrono::steady_clock::duration lastPublish{0};
};

std::optional<Candidate> SuctionGripperPrivate::FindCandidate(
  const gz::sim::EntityComponentManager & _ecm) const
{
  const gz::math::Pose3d cupPose = gz::sim::worldPose(this->cupLink, _ecm);
  // Unit vector pointing out of the cup face.
  const gz::math::Vector3d axis = cupPose.Rot().RotateVector(gz::math::Vector3d::UnitZ);

  std::optional<Candidate> best;
  _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
    [&](const gz::sim::Entity & _model, const gz::sim::components::Model *,
    const gz::sim::components::Name * _name) -> bool
    {
      const std::string & name = _name->Data();
      if (name.rfind(this->modelPrefix, 0) != 0) {
        return true;
      }
      const auto links = _ecm.ChildrenByComponents(_model, gz::sim::components::Link());
      if (links.empty()) {
        return true;
      }
      const gz::sim::Entity link = links.front();
      const gz::math::Pose3d linkPose = gz::sim::worldPose(link, _ecm);

      for (const auto & collision :
        _ecm.ChildrenByComponents(link, gz::sim::components::Collision()))
      {
        const auto * geom = _ecm.Component<gz::sim::components::Geometry>(collision);
        if (!geom || geom->Data().Type() != sdf::GeometryType::BOX) {
          continue;
        }
        const auto * colPoseComp = _ecm.Component<gz::sim::components::Pose>(collision);
        const gz::math::Pose3d boxPose =
          linkPose * (colPoseComp ? colPoseComp->Data() : gz::math::Pose3d::Zero);
        const gz::math::Vector3d half = geom->Data().BoxShape()->Size() / 2.0;

        // Quick reject: box center far from the cup.
        if (boxPose.Pos().Distance(cupPose.Pos()) > half.Length() + 0.1) {
          continue;
        }

        // The face facing the cup is the one whose outward normal is most anti-parallel
        // to the cup axis.
        const gz::math::Vector3d axes[3] = {
          boxPose.Rot().XAxis(), boxPose.Rot().YAxis(), boxPose.Rot().ZAxis()};
        int bestAxis = 0;
        double bestDot = 0.0;
        for (int i = 0; i < 3; ++i) {
          const double d = axes[i].Dot(axis);
          if (std::abs(d) > std::abs(bestDot)) {
            bestDot = d;
            bestAxis = i;
          }
        }
        const gz::math::Vector3d normal = bestDot < 0 ? axes[bestAxis] : -axes[bestAxis];
        const gz::math::Vector3d faceCenter = boxPose.Pos() + normal * half[bestAxis];

        const gz::math::Vector3d delta = faceCenter - cupPose.Pos();
        Candidate c;
        c.model = _model;
        c.link = link;
        c.name = name;
        c.gap = delta.Dot(axis);
        c.offset = (delta - axis * c.gap).Length();
        c.tilt = std::acos(std::clamp(-normal.Dot(axis), -1.0, 1.0));

        std::ostringstream why;
        if (c.gap > this->maxGap || c.gap < this->minGap) {
          why << "gap " << c.gap << " m outside [" << this->minGap << ", " << this->maxGap << "]";
        } else if (c.offset > this->maxOffset) {
          why << "off-center by " << c.offset << " m (max " << this->maxOffset << ")";
        } else if (c.tilt > this->maxTilt) {
          why << "tilted " << c.tilt << " rad (max " << this->maxTilt << ")";
        }
        if (!why.str().empty()) {
          gzmsg << "[SuctionGripper] reject [" << name << "]: " << why.str() << std::endl;
          continue;
        }
        if (!best || c.offset < best->offset) {
          best = c;
        }
      }
      return true;
    });
  return best;
}

void SuctionGripperPrivate::Publish()
{
  gz::msgs::Boolean grasped;
  grasped.set_data(this->jointEntity != kNull);
  this->graspedPub.Publish(grasped);

  gz::msgs::StringMsg attached;
  attached.set_data(this->attachedName);
  this->attachedPub.Publish(attached);
}

SuctionGripper::SuctionGripper()
: dataPtr(std::make_unique<SuctionGripperPrivate>())
{
}

SuctionGripper::~SuctionGripper() = default;

void SuctionGripper::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  auto & d = *this->dataPtr;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm)) {
    gzerr << "[SuctionGripper] must be attached to a model." << std::endl;
    return;
  }
  d.model = _entity;
  d.modelName = model.Name(_ecm);

  d.linkName = _sdf->Get<std::string>("link_name", "").first;
  d.cupLink = model.LinkByName(_ecm, d.linkName);
  if (d.cupLink == kNull) {
    gzerr << "[SuctionGripper] link [" << d.linkName << "] not found in model ["
          << d.modelName << "]." << std::endl;
    return;
  }

  d.modelPrefix = _sdf->Get<std::string>("model_prefix", d.modelPrefix).first;
  d.maxGap = _sdf->Get<double>("max_gap", d.maxGap).first;
  d.maxOffset = _sdf->Get<double>("max_offset", d.maxOffset).first;
  d.maxTilt = _sdf->Get<double>("max_tilt", d.maxTilt).first;
  const double rate = _sdf->Get<double>("publish_rate", 10.0).first;
  if (rate > 0) {
    d.publishPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate));
  }

  const auto cmdTopic = _sdf->Get<std::string>("cmd_topic", "/suction/cmd").first;
  const auto graspedTopic = _sdf->Get<std::string>("grasped_topic", "/suction/grasped").first;
  const auto attachedTopic =
    _sdf->Get<std::string>("attached_topic", "/suction/attached_model").first;

  d.node.Subscribe(cmdTopic, &SuctionGripperPrivate::OnCmd, &d);
  d.graspedPub = d.node.Advertise<gz::msgs::Boolean>(graspedTopic);
  d.attachedPub = d.node.Advertise<gz::msgs::StringMsg>(attachedTopic);

  gzmsg << "[SuctionGripper] link [" << d.linkName << "], prefix [" << d.modelPrefix
        << "], max_gap " << d.maxGap << ", max_offset " << d.maxOffset
        << ", cmd [" << cmdTopic << "]" << std::endl;
}

void SuctionGripper::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager & _ecm)
{
  auto & d = *this->dataPtr;
  if (d.cupLink == kNull) {
    return;
  }

  // The attached box was deleted from the world: drop the joint.
  if (d.jointEntity != kNull && !_ecm.HasEntity(d.attachedLink)) {
    gzmsg << "[SuctionGripper] attached model [" << d.attachedName << "] disappeared."
          << std::endl;
    _ecm.RequestRemoveEntity(d.jointEntity, true);
    d.jointEntity = kNull;
    d.attachedLink = kNull;
    d.attachedName.clear();
    d.stateChanged = true;
  }

  if (!d.pendingCmd.exchange(false)) {
    return;
  }

  if (d.requested && d.jointEntity == kNull) {
    const auto candidate = d.FindCandidate(_ecm);
    if (!candidate) {
      gzmsg << "[SuctionGripper] suction on, but no box within tolerance." << std::endl;
      d.stateChanged = true;
      return;
    }
    d.jointEntity = _ecm.CreateEntity();
    _ecm.CreateComponent(
      d.jointEntity,
      gz::sim::components::DetachableJoint({d.cupLink, candidate->link, "fixed"}));
    d.attachedLink = candidate->link;
    d.attachedName = candidate->name;
    d.stateChanged = true;
    gzmsg << "[SuctionGripper] attached [" << candidate->name << "] (gap " << candidate->gap
          << " m, offset " << candidate->offset << " m)" << std::endl;
  } else if (!d.requested && d.jointEntity != kNull) {
    _ecm.RequestRemoveEntity(d.jointEntity, true);
    gzmsg << "[SuctionGripper] released [" << d.attachedName << "]" << std::endl;
    d.jointEntity = kNull;
    d.attachedLink = kNull;
    d.attachedName.clear();
    d.stateChanged = true;
  } else {
    // Repeated command: re-publish so callers always get an answer.
    d.stateChanged = true;
  }
}

void SuctionGripper::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager &)
{
  auto & d = *this->dataPtr;
  if (d.cupLink == kNull) {
    return;
  }
  const bool periodic = _info.simTime - d.lastPublish >= d.publishPeriod ||
    _info.simTime < d.lastPublish;
  if (d.stateChanged || periodic) {
    d.Publish();
    d.stateChanged = false;
    d.lastPublish = _info.simTime;
  }
}
}  // namespace palletizer_gazebo

GZ_ADD_PLUGIN(
  palletizer_gazebo::SuctionGripper,
  gz::sim::System,
  palletizer_gazebo::SuctionGripper::ISystemConfigure,
  palletizer_gazebo::SuctionGripper::ISystemPreUpdate,
  palletizer_gazebo::SuctionGripper::ISystemPostUpdate)
