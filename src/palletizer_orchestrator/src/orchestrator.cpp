// Palletizing orchestrator: BehaviorTree.CPP v4 tree driving perception, packing and motion.
//
// The tree (trees/palletize.xml) runs: wait for a box at the pick eye -> detect -> pre-check
// fit (pallet full -> swap) -> pick (+ weigh) -> resume conveyor -> plan with measured mass
// -> place -> verify release -> commit to pallet model + planning scene -> report -> repeat.
// Recovery: re-detect, retry grasp once then reject, replan/retry place once then reject,
// no feasible placement -> pallet swap.
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

#include "palletizer_interfaces/action/pick_box.hpp"
#include "palletizer_interfaces/action/place_box.hpp"
#include "palletizer_interfaces/msg/cycle_report.hpp"
#include "palletizer_interfaces/msg/detected_box.hpp"
#include "palletizer_interfaces/msg/pallet_report.hpp"
#include "palletizer_interfaces/msg/placement.hpp"
#include "palletizer_interfaces/srv/commit_placement.hpp"
#include "palletizer_interfaces/srv/detect_box.hpp"
#include "palletizer_interfaces/srv/plan_placement.hpp"
#include "palletizer_interfaces/srv/weigh.hpp"

using namespace std::chrono_literals;
namespace pi = palletizer_interfaces;
using DetectedBox = pi::msg::DetectedBox;
using PlacementMsg = pi::msg::Placement;
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
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> triggers;
  rclcpp_action::Client<pi::action::PickBox>::SharedPtr pick;
  rclcpp_action::Client<pi::action::PlaceBox>::SharedPtr place;
  rclcpp::Publisher<pi::msg::CycleReport>::SharedPtr cycle_pub;
  rclcpp::Publisher<pi::msg::PalletReport>::SharedPtr pallet_pub;

  std::atomic<bool> box_ready{false};

  // bookkeeping
  std::string planner;
  uint32_t pallet_index{0};
  uint32_t box_index{0};
  uint32_t pallet_boxes{0};
  double pallet_volume{0.0};
  double pallet_max_height{0.0};
  double usable_height{1.05};
  double pallet_area{0.96};
  double pallet_top{0.144};
  rclcpp::Time pallet_start;
  rclcpp::Time cycle_start;
  uint32_t detect_attempts{0};
  uint32_t grasp_attempts{0};
  int max_pallets{0};          // 0 = unlimited
  int completed_pallets{0};

  rclcpp::Time now() {return node->now();}
  rclcpp::Logger log() {return node->get_logger();}

  pi::msg::PalletReport pallet_report(const std::string & reason)
  {
    pi::msg::PalletReport r;
    r.stamp = now();
    r.pallet_index = pallet_index;
    r.planner = planner;
    r.boxes = pallet_boxes;
    r.box_volume = pallet_volume;
    r.fill_percent = 100.0 * pallet_volume / (pallet_area * usable_height);
    r.max_height = pallet_max_height;
    r.duration = (now() - pallet_start).seconds();
    r.end_reason = reason;
    return r;
  }

  void start_pallet()
  {
    pallet_boxes = 0;
    pallet_volume = 0.0;
    pallet_max_height = 0.0;
    pallet_start = now();
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
      BT::InputPort<bool>("check_only", false, ""), BT::OutputPort<PlacementMsg>("placement")};
  }
  bool fill(pi::srv::PlanPlacement::Request & req) override
  {
    auto box = getInput<DetectedBox>("box");
    if (!box) {return false;}
    req.box = box.value();
    req.mass = getInput<double>("mass").value_or(0.0);
    req.check_only = getInput<bool>("check_only").value_or(false);
    return true;
  }
  BT::NodeStatus done(const pi::srv::PlanPlacement::Response & res) override
  {
    if (!res.success) {
      RCLCPP_INFO(ctx_->log(), "plan: %s", res.message.c_str());
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
    return r1->success && r2->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
        ctx_->pallet_max_height, r.placement.rest_z + r.placement.height - ctx_->pallet_top);
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
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("reason", "full", "")};}
  BT::NodeStatus tick() override
  {
    auto r = ctx_->pallet_report(getInput<std::string>("reason").value());
    ctx_->pallet_pub->publish(r);
    RCLCPP_INFO(ctx_->log(), "pallet %u done: %u boxes, fill %.1f %%, max height %.2f m, %.0f s",
      r.pallet_index, r.boxes, r.fill_percent, r.max_height, r.duration);
    ctx_->completed_pallets++;
    ctx_->pallet_index++;
    ctx_->start_pallet();
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
    return (ctx_->max_pallets <= 0 || ctx_->completed_pallets < ctx_->max_pallets) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
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
  ctx->max_pallets = node->declare_parameter<int>("max_pallets", 0);
  ctx->usable_height = node->declare_parameter<double>("usable_height", 1.05);
  ctx->pallet_area = node->declare_parameter<double>("pallet_area", 0.96);
  ctx->pallet_top = node->declare_parameter<double>("pallet_top", 0.144);
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
  const std::map<std::string, std::string> trig = {
    {"resume", "/conveyor_controller/resume"}, {"park", "/motion/park"},
    {"reject_pick", "/sim/reject_pick_zone_box"}, {"reject_held", "/sim/reject_held_box"},
    {"clear_pallet", "/sim/clear_pallet"}, {"reset_packing", "/packing/reset"},
    {"clear_scene", "/motion/scene/clear_placed"}, {"drop_held", "/motion/drop_held"}};
  for (auto & [k, srv] : trig) {
    ctx->triggers[k] = node->create_client<Trigger>(srv, rclcpp::ServicesQoS(), cbg);
  }
  ctx->pick = rclcpp_action::create_client<pi::action::PickBox>(node, "/motion/pick", cbg);
  ctx->place = rclcpp_action::create_client<pi::action::PlaceBox>(node, "/motion/place", cbg);
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
  const std::map<std::string, std::string> trigger_nodes = {
    {"ResumeConveyor", "resume"}, {"Park", "park"}, {"RejectPickZoneBox", "reject_pick"},
    {"RejectHeldBox", "reject_held"}, {"ClearPallet", "clear_pallet"},
    {"ResetPackingModel", "reset_packing"}, {"ClearPlanningScene", "clear_scene"},
    {"DropHeld", "drop_held"}};
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
  RCLCPP_INFO(node->get_logger(), "tree loaded (%s); Groot2 on port %ld; planner %s",
    tree_file.c_str(), groot_port, ctx->planner.c_str());

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
