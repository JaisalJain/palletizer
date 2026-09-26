// World system: kinematic mover for the simulated pallet conveyor (plant equipment, used by
// palletizer_tools/pallet_exchanger). It moves a set of models (a pallet and the boxes on
// it) rigidly together by a planar displacement with a trapezoidal velocity profile, by
// teleporting every model every physics step (WorldPoseCmd), so the stack keeps its
// relative configuration.
//
// Models are driven ONLY while a move is active. Between moves the plugin touches nothing:
// boxes on the pallet are free physics bodies (collapse / shift detection depends on it).
//
// Command (gz.msgs.StringMsg on <cmd_topic>):
//   "move <id> <dx> <dy> <speed> <accel> <model> [<model> ...]"   start a move
//   "stop"                                                         abort (models stay)
// State (gz.msgs.StringMsg on <state_topic>, 10 Hz sim time):
//   "idle <last id>"  |  "moving <id> <progress 0..1> <n models>"
//
// Parameters: <cmd_topic> default /sim/pallet_mover/cmd,
//             <state_topic> default /sim/pallet_mover/state

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace palletizer_gazebo
{
struct MoveCmd
{
  long id{0};
  double dx{0}, dy{0}, speed{0.3}, accel{0.5};
  std::vector<std::string> names;
};

class PalletMover
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->world = _entity;
    const auto cmd = _sdf->Get<std::string>("cmd_topic", "/sim/pallet_mover/cmd").first;
    const auto state = _sdf->Get<std::string>("state_topic", "/sim/pallet_mover/state").first;
    this->node.Subscribe(cmd, &PalletMover::OnCmd, this);
    this->pub = this->node.Advertise<gz::msgs::StringMsg>(state);
  }

  void OnCmd(const gz::msgs::StringMsg & _msg)
  {
    std::istringstream in(_msg.data());
    std::string verb;
    in >> verb;
    std::lock_guard<std::mutex> lock(this->mutex);
    if (verb == "stop") {
      this->stopRequested = true;
      return;
    }
    if (verb != "move") {
      gzerr << "PalletMover: unknown command [" << _msg.data() << "]\n";
      return;
    }
    MoveCmd c;
    in >> c.id >> c.dx >> c.dy >> c.speed >> c.accel;
    std::string n;
    while (in >> n) {c.names.push_back(n);}
    if (!in.eof() || c.speed <= 0 || c.accel <= 0 || c.names.empty()) {
      gzerr << "PalletMover: bad move command [" << _msg.data() << "]\n";
      return;
    }
    this->pending = c;
  }

  void PreUpdate(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
  override
  {
    const double t = std::chrono::duration<double>(_info.simTime).count();
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->stopRequested) {
        this->stopRequested = false;
        this->Finish(_ecm);
      }
      if (this->pending) {
        this->Start(*this->pending, t, _ecm);
        this->pending.reset();
      }
    }
    if (this->active && !_info.paused) {
      const double s = this->Distance(t - this->t0);
      const double f = this->total > 0 ? s / this->total : 1.0;
      for (auto & [e, p0] : this->models) {
        gz::math::Pose3d p = p0;
        p.Pos().X() += f * this->cmd.dx;
        p.Pos().Y() += f * this->cmd.dy;
        gz::sim::Model(e).SetWorldPoseCmd(_ecm, p);
      }
      if (t - this->t0 >= this->duration) {
        this->Finish(_ecm);
      }
    }
    if (t - this->lastState >= 0.1 || t < this->lastState) {
      this->lastState = t;
      gz::msgs::StringMsg m;
      std::ostringstream out;
      if (this->active) {
        const double s = this->Distance(t - this->t0);
        out << "moving " << this->cmd.id << " " << (this->total > 0 ? s / this->total : 1.0)
            << " " << this->models.size();
      } else {
        out << "idle " << this->lastId;
      }
      m.set_data(out.str());
      this->pub.Publish(m);
    }
  }

private:
  void Start(const MoveCmd & c, double t, gz::sim::EntityComponentManager & _ecm)
  {
    this->models.clear();
    for (const auto & name : c.names) {
      auto e = _ecm.EntityByComponents(
        gz::sim::components::Model(), gz::sim::components::Name(name),
        gz::sim::components::ParentEntity(this->world));
      if (e == gz::sim::kNullEntity) {
        gzwarn << "PalletMover: no model [" << name << "]\n";
        continue;
      }
      this->models.emplace_back(e, gz::sim::worldPose(e, _ecm));
    }
    this->cmd = c;
    this->total = std::hypot(c.dx, c.dy);
    // trapezoid (or triangle if the distance is too short to reach full speed)
    this->vmax = std::min(c.speed, std::sqrt(this->total * c.accel));
    this->tacc = this->vmax / c.accel;
    const double dacc = 0.5 * c.accel * this->tacc * this->tacc;
    this->duration = 2 * this->tacc + (this->total - 2 * dacc) / std::max(this->vmax, 1e-9);
    this->t0 = t;
    this->active = !this->models.empty();
    if (!this->active) {this->lastId = c.id;}
    gzmsg << "PalletMover: move " << c.id << " of " << this->models.size() << " models by ("
          << c.dx << ", " << c.dy << ") in " << this->duration << " s\n";
  }

  void Finish(gz::sim::EntityComponentManager & _ecm)
  {
    // Release: from now on the models are never touched again (free physics bodies).
    (void)_ecm;
    this->models.clear();
    this->active = false;
    this->lastId = this->cmd.id;
  }

  double Distance(double dt) const
  {
    const double a = this->cmd.accel;
    if (dt <= 0) {return 0.0;}
    if (dt >= this->duration) {return this->total;}
    if (dt < this->tacc) {return 0.5 * a * dt * dt;}
    const double dacc = 0.5 * a * this->tacc * this->tacc;
    if (dt < this->duration - this->tacc) {return dacc + this->vmax * (dt - this->tacc);}
    const double r = this->duration - dt;
    return this->total - 0.5 * a * r * r;
  }

  gz::sim::Entity world{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  std::mutex mutex;
  std::optional<MoveCmd> pending;
  bool stopRequested{false};

  bool active{false};
  MoveCmd cmd;
  long lastId{0};
  std::vector<std::pair<gz::sim::Entity, gz::math::Pose3d>> models;
  double total{0}, vmax{0}, tacc{0}, duration{0}, t0{0};
  double lastState{-1.0};
};
}  // namespace palletizer_gazebo

GZ_ADD_PLUGIN(
  palletizer_gazebo::PalletMover,
  gz::sim::System,
  palletizer_gazebo::PalletMover::ISystemConfigure,
  palletizer_gazebo::PalletMover::ISystemPreUpdate)
