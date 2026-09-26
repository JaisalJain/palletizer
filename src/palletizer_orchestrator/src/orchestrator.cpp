// Palletizing orchestrator: BehaviorTree.CPP v4 tree driving perception, packing and motion.
//
// The tree (trees/palletize.xml, main tree per `mode`: mixed | single_sku) runs: wait for a
// box at the pick eye -> detect -> pre-check fit (pallet full / SKU change -> close pallet +
// exchange) -> pick (+ weigh) IN PARALLEL with verifying the previous placement (pallet
// camera) and planning this one -> resume conveyor -> re-check the plan with the measured mass
// -> place -> verify release -> commit -> report -> "pallet full?" -> repeat.
// The orchestrator decides FULL itself (pattern complete, or no placement: height limit / no
// feasible placement, SKU change, operator), parks the arm, publishes FULL on /pallet/status,
// runs the PalletExchange action, waits for READY, has the new pallet measured, hands it to
// packing and the planning scene, and continues.
//
// Groot2: the tree is published on port `groot_port` (default 1667).

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "palletizer_interfaces/action/pallet_exchange.hpp"
#include "palletizer_interfaces/action/pick_box.hpp"
#include "palletizer_interfaces/action/place_box.hpp"
#include "palletizer_interfaces/msg/cycle_report.hpp"
#include "palletizer_interfaces/msg/detected_box.hpp"
#include "palletizer_interfaces/msg/detected_pallet.hpp"
#include "palletizer_interfaces/msg/packing_state.hpp"
#include "palletizer_interfaces/msg/pallet_report.hpp"
#include "palletizer_interfaces/msg/pallet_status.hpp"
#include "palletizer_interfaces/msg/placement.hpp"
#include "palletizer_interfaces/msg/stack_event.hpp"
#include "palletizer_interfaces/srv/commit_placement.hpp"
#include "palletizer_interfaces/srv/detect_box.hpp"
#include "palletizer_interfaces/srv/measure_pallet.hpp"
#include "palletizer_interfaces/srv/plan_placement.hpp"
#include "palletizer_interfaces/srv/set_pallet.hpp"
#include "palletizer_interfaces/srv/verify_placement.hpp"
#include "palletizer_interfaces/srv/weigh.hpp"

using namespace std::chrono_literals;
namespace pi = palletizer_interfaces;
using DetectedBox = pi::msg::DetectedBox;
using PlacementMsg = pi::msg::Placement;
using PalletStatus = pi::msg::PalletStatus;
using Trigger = std_srvs::srv::Trigger;

// --------------------------------------------------------------------------- shared context
struct Context
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<pi::srv::DetectBox>::SharedPtr detect;
  rclcpp::Client<pi::srv::PlanPlacement>::SharedPtr plan;
  rclcpp::Client<pi::srv::CommitPlacement>::SharedPtr commit_packing;
  rclcpp::Client<pi::srv::CommitPlacement>::SharedPtr commit_scene;
  rclcpp::Client<pi::srv::Weigh>::SharedPtr weigh;
  rclcpp::Client<pi::srv::MeasurePallet>::SharedPtr measure;
  rclcpp::Client<pi::srv::SetPallet>::SharedPtr set_pallet_packing;
  rclcpp::Client<pi::srv::SetPallet>::SharedPtr set_pallet_scene;
  rclcpp::Client<pi::srv::VerifyPlacement>::SharedPtr verify;
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> triggers;
  rclcpp_action::Client<pi::action::PickBox>::SharedPtr pick;
  rclcpp_action::Client<pi::action::PlaceBox>::SharedPtr place;
  rclcpp_action::Client<pi::action::PalletExchange>::SharedPtr exchange;
  rclcpp::Publisher<pi::msg::CycleReport>::SharedPtr cycle_pub;
  rclcpp::Publisher<pi::msg::PalletReport>::SharedPtr pallet_pub;
  rclcpp::Publisher<PalletStatus>::SharedPtr status_pub;

  std::atomic<bool> box_ready{false};
  std::atomic<bool> operator_close{false};
  std::atomic<uint32_t> shift_events{0};

  // bookkeeping
  std::string mode{"mixed"};
  std::string planner;
  uint32_t pallet_index{0};
  uint32_t box_index{0};
  uint32_t pallet_boxes{0};
  double pallet_volume{0.0};
  double pallet_max_height{0.0};
  rclcpp::Time pallet_start;
  rclcpp::Time cycle_start;
  uint32_t detect_attempts{0};
  uint32_t grasp_attempts{0};
  int max_pallets{0};          // 0 = unlimited
  bool finish_with_next_box{false};
  int completed_pallets{0};

  // pallet state (all measured)
  pi::msg::DetectedPallet pallet;
  pi::msg::PackingState packing;
  uint8_t state{PalletStatus::EMPTY};
  std::string reason;
  // last committed placement, verified by the pallet camera during the next pick
  bool unverified{false};
  PlacementMsg last_placement;
  DetectedBox last_box;
  std::string last_sim_name;
  uint32_t verified{0};

  rclcpp::Time now() {return node->now();}
  rclcpp::Logger log() {return node->get_logger();}

  void publish_status(uint8_t st, const std::string & why = "")
  {
    state = st;
    if (!why.empty() || st != PalletStatus::FULL) {reason = why;}
    static const char * names[] = {"EMPTY", "IN_PROGRESS", "FULL", "EXCHANGING", "READY"};
    PalletStatus m;
    m.stamp = now();
    m.pallet_id = pallet_index;
    m.state = st;
    m.state_name = names[st];
    m.mode = mode;
    m.pallet_length = pallet.valid ? pallet.size_u : 0.0;
    m.pallet_width = pallet.valid ? pallet.size_v : 0.0;
    if (packing.has_sku) {
      m.sku_length = packing.sku_length;
      m.sku_width = packing.sku_width;
      m.sku_height = packing.sku_height;
    }
    m.boxes_placed = pallet_boxes;
    m.boxes_in_pattern = packing.pattern_count;
    m.layer = packing.layer;
    m.layers = packing.layers;
    m.fill_percent = packing.fill_percent;
    m.reason = reason;
    m.shift_events = shift_events;
    status_pub->publish(m);
  }

  pi::msg::PalletReport pallet_report(const std::string & why)
  {
    pi::msg::PalletReport r;
    r.stamp = now();
    r.pallet_index = pallet_index;
    r.planner = planner;
    r.boxes = pallet_boxes;
    r.box_volume = pallet_volume;
    r.fill_percent = packing.fill_percent;
    r.max_height = pallet_max_height;
    r.duration = (now() - pallet_start).seconds();
    r.end_reason = why;
    r.mode = mode;
    r.pallet_length = pallet.size_u;
    r.pallet_width = pallet.size_v;
    r.usable_height = packing.usable_height;
    r.sku_length = packing.sku_length;
    r.sku_width = packing.sku_width;
    r.sku_height = packing.sku_height;
    r.boxes_in_pattern = packing.pattern_count;
    r.layers = packing.layers;
    r.shift_events = shift_events;
    return r;
  }

  void start_pallet()
  {
    pallet_boxes = 0;
    pallet_volume = 0.0;
    pallet_max_height = 0.0;
    pallet_start = now();
    unverified = false;
    verified = 0;
    shift_events = 0;
  }
};
using ContextPtr = std::shared_ptr<Context>;

// Generic "call a service asynchronously" stateful node.
template<class Srv>
class ServiceNode : public BT::StatefulActionNode
{
public:
  ServiceNode(const std::string & name, const BT::NodeConfig & cfg, ContextPtr ctx,
    typename rclcpp::Client<Srv>::SharedPtr client, double timeout_s = 30.0)
  : BT::StatefulActionNode(name, cfg), ctx_(ctx), client_(client), timeout_(timeout_s) {}

  BT::NodeStatus onStart() override
  {
    auto req = std::make_shared<typename Srv::Request>();
    if (!fill(*req)) {return BT::NodeStatus::FAILURE;}
    if (!client_->service_is_ready()) {
      RCLCPP_WARN(ctx_->log(), "%s: service %s not ready", name().c_str(),
        client_->get_service_name());
      return BT::NodeStatus::FAILURE;
    }
    future_ = client_->async_send_request(req).future.share();
    start_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (future_.wait_for(0s) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() - start_ > std::chrono::duration<double>(timeout_)) {
        RCLCPP_WARN(ctx_->log(), "%s: timeout", name().c_str());
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    return done(*future_.get());
  }

  void onHalted() override {}

protected:
  virtual bool fill(typename Srv::Request &) {return true;}
  virtual BT::NodeStatus done(const typename Srv::Response & res) = 0;

  ContextPtr ctx_;
  typename rclcpp::Client<Srv>::SharedPtr client_;
  std::shared_future<typename Srv::Response::SharedPtr> future_;
  std::chrono::steady_clock::time_point start_;
  double timeout_;
};

// --------------------------------------------------------------------------- tree nodes
class WaitBoxReady : public BT::StatefulActionNode
{
public:
  WaitBoxReady(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : BT::StatefulActionNode(n, c), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("timeout", 300.0, "s")};}
  BT::NodeStatus onStart() override
  {
    start_ = std::chrono::steady_clock::now();
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    if (ctx_->box_ready) {
      ctx_->cycle_start = ctx_->now();
      ctx_->detect_attempts = 0;
      ctx_->grasp_attempts = 0;
      return BT::NodeStatus::SUCCESS;
    }
    double t = getInput<double>("timeout").value();
    if (std::chrono::steady_clock::now() - start_ > std::chrono::duration<double>(t)) {
      RCLCPP_WARN(ctx_->log(), "no box arrived within %.0f s", t);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}

private:
  ContextPtr ctx_;
  std::chrono::steady_clock::time_point start_;
};

class DetectBoxNode : public ServiceNode<pi::srv::DetectBox>
{
public:
  DetectBoxNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : ServiceNode(n, c, ctx, ctx->detect, 15.0) {}
  static BT::PortsList providedPorts() {return {BT::OutputPort<DetectedBox>("box")};}
  bool fill(pi::srv::DetectBox::Request &) override
  {
    ctx_->detect_attempts++;
    return true;
  }
  BT::NodeStatus done(const pi::srv::DetectBox::Response & res) override
  {
    if (!res.success) {
      RCLCPP_WARN(ctx_->log(), "detect failed: %s", res.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("box", res.box);
    return BT::NodeStatus::SUCCESS;
  }
};

class PlanPlacementNode : public ServiceNode<pi::srv::PlanPlacement>
{
public:
  PlanPlacementNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : ServiceNode(n, c, ctx, ctx->plan, 60.0) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<DetectedBox>("box"), BT::InputPort<double>("mass", 0.0, "kg"),
      BT::InputPort<bool>("check_only", false, ""), BT::OutputPort<PlacementMsg>("placement"),
      BT::InputPort<PlacementMsg>("exclude", "a placement that failed in motion"),
      BT::InputPort<bool>("recheck", false, "re-check the last plan with the measured mass"),
      BT::OutputPort<std::string>("reason", "why no placement (PalletStatus reasons)")};
  }
  bool fill(pi::srv::PlanPlacement::Request & req) override
  {
    auto box = getInput<DetectedBox>("box");
    if (!box) {return false;}
    req.box = box.value();
    req.mass = getInput<double>("mass").value_or(0.0);
    req.check_only = getInput<bool>("check_only").value_or(false);
    req.recheck = getInput<bool>("recheck").value_or(false);
    if (auto ex = getInput<PlacementMsg>("exclude")) {req.exclude.push_back(ex.value());}
    return true;
  }
  BT::NodeStatus done(const pi::srv::PlanPlacement::Response & res) override
  {
    if (res.state.has_pallet) {ctx_->packing = res.state;}
    if (!res.success) {
      RCLCPP_INFO(ctx_->log(), "plan: %s (%s)", res.message.c_str(), res.reason.c_str());
      setOutput("reason", res.reason);
      return BT::NodeStatus::FAILURE;
    }
    setOutput("placement", res.placement);
    return BT::NodeStatus::SUCCESS;
  }
};

template<class ActionT>
class ActionNode : public BT::StatefulActionNode
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  ActionNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx,
    typename rclcpp_action::Client<ActionT>::SharedPtr client)
  : BT::StatefulActionNode(n, c), ctx_(ctx), client_(client) {}

  BT::NodeStatus onStart() override
  {
    typename ActionT::Goal goal;
    if (!fill(goal)) {return BT::NodeStatus::FAILURE;}
    if (!client_->action_server_is_ready()) {
      RCLCPP_WARN(ctx_->log(), "%s: action server not ready", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    goal_future_ = client_->async_send_goal(goal);
    result_future_ = {};
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!result_future_.valid()) {
      if (goal_future_.wait_for(0s) != std::future_status::ready) {return BT::NodeStatus::RUNNING;}
      auto gh = goal_future_.get();
      if (!gh) {
        RCLCPP_WARN(ctx_->log(), "%s: goal rejected", name().c_str());
        return BT::NodeStatus::FAILURE;
      }
      handle_ = gh;
      result_future_ = client_->async_get_result(gh);
    }
    if (result_future_.wait_for(0s) != std::future_status::ready) {return BT::NodeStatus::RUNNING;}
    auto wrapped = result_future_.get();
    return done(*wrapped.result);
  }

  void onHalted() override
  {
    if (handle_) {client_->async_cancel_goal(handle_);}
  }

protected:
  virtual bool fill(typename ActionT::Goal & goal) = 0;
  virtual BT::NodeStatus done(const typename ActionT::Result & result) = 0;
  ContextPtr ctx_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
  std::shared_future<typename GoalHandle::WrappedResult> result_future_;
  typename GoalHandle::SharedPtr handle_;
};

class PickBoxNode : public ActionNode<pi::action::PickBox>
{
public:
  PickBoxNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : ActionNode(n, c, ctx, ctx->pick) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<DetectedBox>("box"), BT::OutputPort<double>("mass"),
      BT::OutputPort<std::string>("sim_name"), BT::OutputPort<std::string>("error")};
  }
  bool fill(pi::action::PickBox::Goal & goal) override
  {
    auto box = getInput<DetectedBox>("box");
    if (!box) {return false;}
    goal.box = box.value();
    ctx_->grasp_attempts++;
    return true;
  }
  BT::NodeStatus done(const pi::action::PickBox::Result & r) override
  {
    setOutput("error", r.error_code);
    if (!r.success) {
      RCLCPP_WARN(ctx_->log(), "pick failed: %s %s", r.error_code.c_str(), r.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("mass", r.measured_mass);
    setOutput("sim_name", r.sim_model_name);
    return BT::NodeStatus::SUCCESS;
  }
};

class PlaceBoxNode : public ActionNode<pi::action::PlaceBox>
{
public:
  PlaceBoxNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : ActionNode(n, c, ctx, ctx->place) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<DetectedBox>("box"), BT::InputPort<PlacementMsg>("placement")};
  }
  bool fill(pi::action::PlaceBox::Goal & goal) override
  {
    auto box = getInput<DetectedBox>("box");
    auto pl = getInput<PlacementMsg>("placement");
    if (!box || !pl) {return false;}
    goal.box = box.value();
    goal.placement = pl.value();
    return true;
  }
  BT::NodeStatus done(const pi::action::PlaceBox::Result & r) override
  {
    if (!r.success) {
      RCLCPP_WARN(ctx_->log(), "place failed: %s %s", r.error_code.c_str(), r.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

// Calls one of the Trigger services by key (resume, park, reject_pick, reject_held,
// clear_pallet, reset_packing, clear_scene).
class TriggerNode : public ServiceNode<Trigger>
{
public:
  TriggerNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx,
    const std::string & key, double timeout = 60.0)
  : ServiceNode(n, c, ctx, ctx->triggers.at(key), timeout), key_(key) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus done(const Trigger::Response & r) override
  {
    RCLCPP_INFO(ctx_->log(), "%s: %s", key_.c_str(), r.message.c_str());
    return r.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  std::string key_;
};

// Verify the release: the F/T sensor must read (close to) the tare again.
class VerifyRelease : public ServiceNode<pi::srv::Weigh>
{
public:
  VerifyRelease(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : ServiceNode(n, c, ctx, ctx->weigh, 10.0) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("max_mass", 0.3, "kg")};}
  bool fill(pi::srv::Weigh::Request & req) override
  {
    req.tare = false;
    req.duration = 0.2;
    return true;
  }
  BT::NodeStatus done(const pi::srv::Weigh::Response & r) override
  {
    double lim = getInput<double>("max_mass").value();
    if (!r.success || std::abs(r.mass) > lim) {
      RCLCPP_WARN(ctx_->log(), "release not verified: gripper still weighs %.2f kg", r.mass);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class CommitNode : public BT::StatefulActionNode
{
public:
  CommitNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : BT::StatefulActionNode(n, c), ctx_(ctx) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<DetectedBox>("box"), BT::InputPort<PlacementMsg>("placement"),
      BT::InputPort<double>("mass"), BT::InputPort<std::string>("sim_name")};
  }
  BT::NodeStatus onStart() override
  {
    auto req = std::make_shared<pi::srv::CommitPlacement::Request>();
    req->box = getInput<DetectedBox>("box").value();
    req->placement = getInput<PlacementMsg>("placement").value();
    req->mass = getInput<double>("mass").value_or(0.0);
    req->sim_model_name = getInput<std::string>("sim_name").value_or("");
    f1_ = ctx_->commit_packing->async_send_request(req).future.share();
    f2_ = ctx_->commit_scene->async_send_request(req).future.share();
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (f1_.wait_for(0s) != std::future_status::ready ||
      f2_.wait_for(0s) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    auto r1 = f1_.get();
    auto r2 = f2_.get();
    RCLCPP_INFO(ctx_->log(), "commit: %s", r1->message.c_str());
    if (!(r1->success && r2->success)) {return BT::NodeStatus::FAILURE;}
    ctx_->packing = r1->state;
    ctx_->unverified = true;
    ctx_->last_placement = getInput<PlacementMsg>("placement").value();
    ctx_->last_box = getInput<DetectedBox>("box").value();
    ctx_->last_sim_name = getInput<std::string>("sim_name").value_or("");
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {}

private:
  ContextPtr ctx_;
  std::shared_future<pi::srv::CommitPlacement::Response::SharedPtr> f1_, f2_;
};

// Publishes a CycleReport. outcome = placed | rejected_* ...
class ReportCycle : public BT::SyncActionNode
{
public:
  ReportCycle(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : BT::SyncActionNode(n, c), ctx_(ctx) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("outcome"), BT::InputPort<DetectedBox>("box"),
      BT::InputPort<PlacementMsg>("placement"), BT::InputPort<double>("mass"),
      BT::InputPort<std::string>("sim_name")};
  }
  BT::NodeStatus tick() override
  {
    pi::msg::CycleReport r;
    r.stamp = ctx_->now();
    r.pallet_index = ctx_->pallet_index;
    r.box_index = ctx_->box_index++;
    r.outcome = getInput<std::string>("outcome").value_or("unknown");
    r.success = r.outcome == "placed";
    if (auto b = getInput<DetectedBox>("box")) {r.detected = b.value();}
    if (auto p = getInput<PlacementMsg>("placement")) {r.placement = p.value();}
    r.measured_mass = getInput<double>("mass").value_or(0.0);
    r.sim_model_name = getInput<std::string>("sim_name").value_or("");
    r.detect_attempts = ctx_->detect_attempts;
    r.grasp_attempts = ctx_->grasp_attempts;
    r.cycle_time = (ctx_->now() - ctx_->cycle_start).seconds();
    if (r.success) {
      ctx_->pallet_boxes++;
      ctx_->pallet_volume += r.detected.length * r.detected.width * r.detected.height;
      ctx_->pallet_max_height = std::max(
        ctx_->pallet_max_height, r.placement.rest_z + r.placement.height - ctx_->pallet.deck_z);
      ctx_->publish_status(PalletStatus::IN_PROGRESS);
    }
    ctx_->cycle_pub->publish(r);
    RCLCPP_INFO(ctx_->log(), "cycle %u (pallet %u): %s in %.1f s", r.box_index, r.pallet_index,
      r.outcome.c_str(), r.cycle_time);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

// Pallet swap bookkeeping: publish the PalletReport of the finished pallet, start a new one.
class FinishPallet : public BT::SyncActionNode
{
public:
  FinishPallet(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : BT::SyncActionNode(n, c), ctx_(ctx) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("reason", "NO_FEASIBLE_PLACEMENT", "")};
  }
  BT::NodeStatus tick() override
  {
    auto why = getInput<std::string>("reason").value_or("NO_FEASIBLE_PLACEMENT");
    if (why.empty()) {why = "NO_FEASIBLE_PLACEMENT";}
    auto r = ctx_->pallet_report(why);
    ctx_->pallet_pub->publish(r);
    ctx_->publish_status(PalletStatus::FULL, why);
    RCLCPP_INFO(ctx_->log(), "pallet %u FULL (%s): %u/%u boxes, fill %.1f %%, max height %.2f m, "
      "%.0f s, %u stack events", r.pallet_index, why.c_str(), r.boxes, r.boxes_in_pattern,
      r.fill_percent, r.max_height, r.duration, r.shift_events);
    ctx_->completed_pallets++;
    ctx_->pallet_index++;
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

// FAILURE when the requested number of pallets is done (ends the main loop).
class MorePallets : public BT::ConditionNode
{
public:
  MorePallets(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : BT::ConditionNode(n, c), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    if (ctx_->max_pallets <= 0 || ctx_->completed_pallets < ctx_->max_pallets) {
      return BT::NodeStatus::SUCCESS;
    }
    // optionally place the first box of the next pallet (shows it started)
    return (ctx_->finish_with_next_box && ctx_->completed_pallets == ctx_->max_pallets &&
           ctx_->pallet_boxes == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};


// "Pallet full?" (the orchestrator decides FULL): operator request, or the single-SKU
// pattern is complete. (Mixed mode: a pallet is full when the next box does not fit,
// decided by the pre-check.)
class PalletFull : public BT::ConditionNode
{
public:
  PalletFull(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : BT::ConditionNode(n, c), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {BT::OutputPort<std::string>("reason")};}
  BT::NodeStatus tick() override
  {
    if (ctx_->operator_close.exchange(false)) {
      setOutput("reason", std::string("OPERATOR"));
      return BT::NodeStatus::SUCCESS;
    }
    const auto & p = ctx_->packing;
    if (ctx_->mode == "single_sku" && p.pattern_count > 0 && p.placed >= p.pattern_count) {
      setOutput("reason", std::string("PATTERN_COMPLETE"));
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

class IsPalletStarted : public BT::ConditionNode
{
public:
  IsPalletStarted(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : BT::ConditionNode(n, c), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return ctx_->pallet_boxes > 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

// Pallet exchange handshake: EXCHANGING -> action -> READY.
class PalletExchangeNode : public ActionNode<pi::action::PalletExchange>
{
public:
  PalletExchangeNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : ActionNode(n, c, ctx, ctx->exchange) {}
  static BT::PortsList providedPorts() {return {};}
  bool fill(pi::action::PalletExchange::Goal &) override
  {
    ctx_->pallet = pi::msg::DetectedPallet();
    ctx_->packing = pi::msg::PackingState();
    ctx_->start_pallet();
    ctx_->publish_status(PalletStatus::EXCHANGING);
    RCLCPP_INFO(ctx_->log(), "pallet exchange started");
    return true;
  }
  BT::NodeStatus done(const pi::action::PalletExchange::Result & r) override
  {
    if (!r.success) {
      RCLCPP_ERROR(ctx_->log(), "pallet exchange failed: %s", r.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ctx_->log(), "pallet exchange done: %s", r.message.c_str());
    ctx_->publish_status(PalletStatus::READY);
    return BT::NodeStatus::SUCCESS;
  }
};

// Runs several service calls in parallel and waits for all (helper for the nodes below).
class MultiCallNode : public BT::StatefulActionNode
{
public:
  MultiCallNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx, double timeout)
  : BT::StatefulActionNode(n, c), ctx_(ctx), timeout_(timeout) {}
  BT::NodeStatus onRunning() override
  {
    for (auto & ready : pending_) {
      if (!ready()) {
        if (std::chrono::steady_clock::now() - start_ > std::chrono::duration<double>(timeout_)) {
          RCLCPP_WARN(ctx_->log(), "%s: timeout", name().c_str());
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
    }
    return finish();
  }
  void onHalted() override {pending_.clear();}

protected:
  template<class Srv>
  std::shared_future<typename Srv::Response::SharedPtr> call(
    typename rclcpp::Client<Srv>::SharedPtr cli, typename Srv::Request::SharedPtr req)
  {
    auto f = cli->async_send_request(req).future.share();
    pending_.push_back([f]() {return f.wait_for(0s) == std::future_status::ready;});
    return f;
  }
  void begin()
  {
    pending_.clear();
    start_ = std::chrono::steady_clock::now();
  }
  virtual BT::NodeStatus finish() = 0;
  ContextPtr ctx_;
  double timeout_;
  std::vector<std::function<bool()>> pending_;
  std::chrono::steady_clock::time_point start_;
};

// Forget the old pallet everywhere (perception model, packing model, planning scene).
class ResetPalletModels : public MultiCallNode
{
public:
  ResetPalletModels(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : MultiCallNode(n, c, ctx, 30.0) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override
  {
    begin();
    auto none = std::make_shared<pi::srv::SetPallet::Request>();
    none->pallet.valid = false;
    call<Trigger>(ctx_->triggers.at("reset_pallet_perception"),
      std::make_shared<Trigger::Request>());
    call<Trigger>(ctx_->triggers.at("clear_scene"), std::make_shared<Trigger::Request>());
    call<pi::srv::SetPallet>(ctx_->set_pallet_packing, none);
    call<pi::srv::SetPallet>(ctx_->set_pallet_scene, none);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus finish() override {return BT::NodeStatus::SUCCESS;}
};

// Measure the new pallet (pallet camera), hand it to packing and the planning scene: EMPTY.
class MeasurePalletNode : public MultiCallNode
{
public:
  MeasurePalletNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : MultiCallNode(n, c, ctx, 90.0) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override
  {
    begin();
    stage_ = 0;
    auto req = std::make_shared<pi::srv::MeasurePallet::Request>();
    fm_ = call<pi::srv::MeasurePallet>(ctx_->measure, req);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus finish() override
  {
    if (stage_ == 0) {
      auto r = fm_.get();
      if (!r->success) {
        RCLCPP_WARN(ctx_->log(), "pallet measurement failed: %s", r->message.c_str());
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(ctx_->log(), "measured %s", r->message.c_str());
      ctx_->pallet = r->pallet;
      begin();
      stage_ = 1;
      auto req = std::make_shared<pi::srv::SetPallet::Request>();
      req->pallet = r->pallet;
      fp_ = call<pi::srv::SetPallet>(ctx_->set_pallet_packing, req);
      fs_ = call<pi::srv::SetPallet>(ctx_->set_pallet_scene, req);
      return BT::NodeStatus::RUNNING;
    }
    auto rp = fp_.get();
    auto rs = fs_.get();
    RCLCPP_INFO(ctx_->log(), "packing: %s; scene: %s", rp->message.c_str(), rs->message.c_str());
    if (!rp->success || !rs->success) {return BT::NodeStatus::FAILURE;}
    ctx_->packing = rp->state;
    ctx_->start_pallet();
    ctx_->publish_status(PalletStatus::EMPTY);
    return BT::NodeStatus::SUCCESS;
  }

private:
  int stage_{0};
  std::shared_future<pi::srv::MeasurePallet::Response::SharedPtr> fm_;
  std::shared_future<pi::srv::SetPallet::Response::SharedPtr> fp_, fs_;
};

// Measure the last committed placement with the pallet camera (waits for a clear view: runs
// while the arm picks the next box). Never fails the cycle: an unmeasured placement is logged.
class VerifyPlacementNode : public MultiCallNode
{
public:
  VerifyPlacementNode(const std::string & n, const BT::NodeConfig & c, ContextPtr ctx)
  : MultiCallNode(n, c, ctx, 60.0) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override
  {
    if (!ctx_->unverified) {return BT::NodeStatus::SUCCESS;}
    begin();
    auto req = std::make_shared<pi::srv::VerifyPlacement::Request>();
    req->placement = ctx_->last_placement;
    req->box = ctx_->last_box;
    req->index = ctx_->verified;
    req->sim_model_name = ctx_->last_sim_name;
    f_ = call<pi::srv::VerifyPlacement>(ctx_->verify, req);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    auto st = MultiCallNode::onRunning();
    if (st == BT::NodeStatus::FAILURE) {           // timeout: continue on the record
      ctx_->unverified = false;
      ctx_->verified++;
      return BT::NodeStatus::SUCCESS;
    }
    return st;
  }
  BT::NodeStatus finish() override
  {
    auto r = f_.get();
    ctx_->unverified = false;
    ctx_->verified++;
    if (!r->success || !r->check.detected) {
      RCLCPP_WARN(ctx_->log(), "placement not verified: %s", r->message.c_str());
    } else {
      RCLCPP_INFO(ctx_->log(), "verified: %s", r->message.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_future<pi::srv::VerifyPlacement::Response::SharedPtr> f_;
};

// --------------------------------------------------------------------------- main
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("orchestrator");
  auto ctx = std::make_shared<Context>();
  ctx->node = node;
  auto tree_file = node->declare_parameter<std::string>("tree_file", "");
  auto groot_port = node->declare_parameter<int>("groot_port", 1667);
  auto bt_log = node->declare_parameter<std::string>("bt_log_file", "");
  ctx->planner = node->declare_parameter<std::string>("planner", "heuristic");
  ctx->mode = node->declare_parameter<std::string>("mode", "mixed");
  ctx->max_pallets = node->declare_parameter<int>("max_pallets", 0);
  ctx->finish_with_next_box = node->declare_parameter<bool>("finish_with_next_box", false);
  double tick_hz = node->declare_parameter<double>("tick_rate", 50.0);

  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  ctx->detect = node->create_client<pi::srv::DetectBox>("/perception/detect_box",
      rclcpp::ServicesQoS(), cbg);
  ctx->plan = node->create_client<pi::srv::PlanPlacement>("/packing/plan_placement",
      rclcpp::ServicesQoS(), cbg);
  ctx->commit_packing = node->create_client<pi::srv::CommitPlacement>("/packing/commit",
      rclcpp::ServicesQoS(), cbg);
  ctx->commit_scene = node->create_client<pi::srv::CommitPlacement>(
    "/motion/scene/add_placed", rclcpp::ServicesQoS(), cbg);
  ctx->weigh = node->create_client<pi::srv::Weigh>("/suction/weigh", rclcpp::ServicesQoS(), cbg);
  ctx->measure = node->create_client<pi::srv::MeasurePallet>(
    "/pallet_perception/measure_pallet", rclcpp::ServicesQoS(), cbg);
  ctx->verify = node->create_client<pi::srv::VerifyPlacement>(
    "/pallet_perception/verify_placement", rclcpp::ServicesQoS(), cbg);
  ctx->set_pallet_packing = node->create_client<pi::srv::SetPallet>(
    "/packing/set_pallet", rclcpp::ServicesQoS(), cbg);
  ctx->set_pallet_scene = node->create_client<pi::srv::SetPallet>(
    "/motion/scene/set_pallet", rclcpp::ServicesQoS(), cbg);
  const std::map<std::string, std::string> trig = {
    {"resume", "/conveyor_controller/resume"}, {"park", "/motion/park"},
    {"reject_pick", "/sim/reject_pick_zone_box"}, {"reject_held", "/sim/reject_held_box"},
    {"reset_packing", "/packing/reset"}, {"reset_pallet_perception", "/pallet_perception/reset"},
    {"clear_scene", "/motion/scene/clear_placed"}, {"drop_held", "/motion/drop_held"}};
  for (auto & [k, srv] : trig) {
    ctx->triggers[k] = node->create_client<Trigger>(srv, rclcpp::ServicesQoS(), cbg);
  }
  ctx->pick = rclcpp_action::create_client<pi::action::PickBox>(node, "/motion/pick", cbg);
  ctx->place = rclcpp_action::create_client<pi::action::PlaceBox>(node, "/motion/place", cbg);
  ctx->exchange = rclcpp_action::create_client<pi::action::PalletExchange>(
    node, "/pallet_exchange", cbg);
  ctx->status_pub = node->create_publisher<PalletStatus>(
    "/pallet/status", rclcpp::QoS(1).reliable().transient_local());
  auto event_sub = node->create_subscription<pi::msg::StackEvent>(
    "/pallet_perception/stack_events", 50, [ctx](pi::msg::StackEvent::SharedPtr e) {
      ctx->shift_events++;
      RCLCPP_WARN(ctx->log(), "stack event on pallet %u: %s %s", ctx->pallet_index,
      e->kind.c_str(), e->message.c_str());
      if (ctx->pallet.valid) {ctx->publish_status(ctx->state, ctx->reason);}
    });
  auto close_srv = node->create_service<Trigger>(
    "/pallet/close", [ctx](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      ctx->operator_close = true;
      res->success = true;
      res->message = "pallet will be closed after the current box (OPERATOR)";
    }, rclcpp::ServicesQoS(), cbg);
  ctx->cycle_pub = node->create_publisher<pi::msg::CycleReport>("/orchestrator/cycle_report", 50);
  ctx->pallet_pub = node->create_publisher<pi::msg::PalletReport>(
    "/orchestrator/pallet_report", rclcpp::QoS(50).reliable().transient_local());
  auto ready_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/conveyor_controller/box_ready", 10,
    [ctx](std_msgs::msg::Bool::SharedPtr m) {ctx->box_ready = m->data;});

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin([&exec]() {exec.spin();});

  // Wait for the rest of the stack.
  RCLCPP_INFO(node->get_logger(), "waiting for perception, packing and motion servers...");
  ctx->detect->wait_for_service(120s);
  ctx->plan->wait_for_service(120s);
  ctx->pick->wait_for_action_server(180s);
  ctx->place->wait_for_action_server(180s);
  ctx->measure->wait_for_service(120s);
  ctx->verify->wait_for_service(120s);
  ctx->set_pallet_packing->wait_for_service(120s);
  ctx->set_pallet_scene->wait_for_service(120s);
  ctx->exchange->wait_for_action_server(120s);
  for (auto & [k, c] : ctx->triggers) {c->wait_for_service(60s);}

  BT::BehaviorTreeFactory factory;
  auto reg = [&](const std::string & id, auto builder_tag) {
      using T = typename decltype(builder_tag)::type;
      factory.registerBuilder<T>(id, [ctx](const std::string & n, const BT::NodeConfig & c) {
          return std::make_unique<T>(n, c, ctx);
        });
    };
  reg("WaitBoxReady", std::type_identity<WaitBoxReady>{});
  reg("DetectBox", std::type_identity<DetectBoxNode>{});
  reg("PlanPlacement", std::type_identity<PlanPlacementNode>{});
  reg("PickBox", std::type_identity<PickBoxNode>{});
  reg("PlaceBox", std::type_identity<PlaceBoxNode>{});
  reg("VerifyRelease", std::type_identity<VerifyRelease>{});
  reg("CommitPlacement", std::type_identity<CommitNode>{});
  reg("ReportCycle", std::type_identity<ReportCycle>{});
  reg("FinishPallet", std::type_identity<FinishPallet>{});
  reg("MorePallets", std::type_identity<MorePallets>{});
  reg("PalletFull", std::type_identity<PalletFull>{});
  reg("IsPalletStarted", std::type_identity<IsPalletStarted>{});
  reg("PalletExchange", std::type_identity<PalletExchangeNode>{});
  reg("ResetPalletModels", std::type_identity<ResetPalletModels>{});
  reg("MeasurePallet", std::type_identity<MeasurePalletNode>{});
  reg("VerifyPlacement", std::type_identity<VerifyPlacementNode>{});
  const std::map<std::string, std::string> trigger_nodes = {
    {"ResumeConveyor", "resume"}, {"Park", "park"}, {"RejectPickZoneBox", "reject_pick"},
    {"RejectHeldBox", "reject_held"}, {"ResetPackingModel", "reset_packing"},
    {"ClearPlanningScene", "clear_scene"}, {"DropHeld", "drop_held"}};
  for (auto & [id, key] : trigger_nodes) {
    factory.registerBuilder<TriggerNode>(id,
      [ctx, key = key](const std::string & n, const BT::NodeConfig & c) {
        return std::make_unique<TriggerNode>(n, c, ctx, key);
      });
  }

  auto tree = factory.createTreeFromFile(tree_file);
  BT::Groot2Publisher groot(tree, static_cast<unsigned>(groot_port));
  std::unique_ptr<BT::FileLogger2> file_logger;
  if (!bt_log.empty()) {file_logger = std::make_unique<BT::FileLogger2>(tree, bt_log);}
  RCLCPP_INFO(node->get_logger(), "tree loaded (%s); Groot2 on port %ld; mode %s, planner %s",
    tree_file.c_str(), groot_port, ctx->mode.c_str(), ctx->planner.c_str());
  (void)event_sub;
  (void)close_srv;

  ctx->start_pallet();
  auto period = std::chrono::duration<double>(1.0 / tick_hz);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickOnce();
    std::this_thread::sleep_for(period);
  }
  RCLCPP_INFO(node->get_logger(), "tree finished: %s (%d pallets)", BT::toStr(status).c_str(),
    ctx->completed_pallets);
  // give the last reports time to go out
  std::this_thread::sleep_for(1s);
  exec.cancel();
  spin.join();
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
