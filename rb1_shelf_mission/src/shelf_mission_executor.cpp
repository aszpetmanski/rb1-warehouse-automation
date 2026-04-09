#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ShelfMissionExecutor : public rclcpp::Node {
public:
  ShelfMissionExecutor() : rclcpp::Node("shelf_mission_executor") {
    declare_parameter<std::string>("tree_xml_file", "");
    declare_parameter<int>("tick_period_ms", 100);
    declare_parameter<int>("server_timeout_ms", 2000);
    declare_parameter<int>("wait_for_service_timeout_ms", 2000);
    declare_parameter<bool>("autostart", false);
    declare_parameter<std::vector<std::string>>("plugin_libs",
                                                std::vector<std::string>{});

    readParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "start_shelf_mission",
        std::bind(&ShelfMissionExecutor::handleStart, this,
                  std::placeholders::_1, std::placeholders::_2));

    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
        "cancel_shelf_mission",
        std::bind(&ShelfMissionExecutor::handleCancel, this,
                  std::placeholders::_1, std::placeholders::_2));

    timer_ =
        create_wall_timer(std::chrono::milliseconds(tick_period_ms_),
                          std::bind(&ShelfMissionExecutor::tickTree, this));

    RCLCPP_INFO(get_logger(), "ShelfMissionExecutor started");
  }

  void initialize(const rclcpp::Node::SharedPtr &self_node) {
    self_node_ = self_node;

    if (autostart_) {
      std::string msg;
      const bool ok = startMission(msg);
      if (ok) {
        RCLCPP_INFO(get_logger(), "Autostart: %s", msg.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Autostart failed: %s", msg.c_str());
      }
    }
  }

private:
  void readParameters() {
    get_parameter("tree_xml_file", tree_xml_file_);
    get_parameter("tick_period_ms", tick_period_ms_);
    get_parameter("server_timeout_ms", server_timeout_ms_);
    get_parameter("wait_for_service_timeout_ms", wait_for_service_timeout_ms_);
    get_parameter("autostart", autostart_);
    get_parameter("plugin_libs", plugin_libs_);
  }

  bool loadPluginsIfNeeded(std::string &error_msg) {
    if (plugins_loaded_) {
      return true;
    }

    try {
      for (const auto &lib : plugin_libs_) {
        RCLCPP_INFO(get_logger(), "Loading BT plugin: %s", lib.c_str());
        factory_.registerFromPlugin(lib);
      }
      plugins_loaded_ = true;
      return true;
    } catch (const std::exception &e) {
      error_msg = std::string("Failed to load BT plugins: ") + e.what();
      return false;
    }
  }

  bool createTree(std::string &error_msg) {
    if (!self_node_) {
      error_msg = "Executor self node pointer not initialized";
      return false;
    }

    if (tree_xml_file_.empty()) {
      error_msg = "Parameter [tree_xml_file] is empty";
      return false;
    }

    if (!loadPluginsIfNeeded(error_msg)) {
      return false;
    }

    try {
      blackboard_ = BT::Blackboard::create();

      blackboard_->set<rclcpp::Node::SharedPtr>("node", self_node_);
      blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer",
                                                         tf_buffer_);
      blackboard_->set<std::chrono::milliseconds>(
          "server_timeout", std::chrono::milliseconds(server_timeout_ms_));
      blackboard_->set<std::chrono::milliseconds>(
          "bt_loop_duration", std::chrono::milliseconds(tick_period_ms_));
      blackboard_->set<std::chrono::milliseconds>(
          "wait_for_service_timeout",
          std::chrono::milliseconds(wait_for_service_timeout_ms_));
      blackboard_->set<int>("number_recoveries", 0);

      auto tree = factory_.createTreeFromFile(tree_xml_file_, blackboard_);
      tree_ = std::make_shared<BT::Tree>(std::move(tree));

      return true;
    } catch (const std::exception &e) {
      error_msg = std::string("Failed to create BT tree: ") + e.what();
      tree_.reset();
      blackboard_.reset();
      return false;
    }
  }

  bool startMission(std::string &message) {
    std::scoped_lock<std::mutex> lock(mutex_);

    if (running_) {
      message = "Mission already running";
      return false;
    }

    readParameters();

    std::string error_msg;
    if (!createTree(error_msg)) {
      message = error_msg;
      return false;
    }

    running_ = true;
    last_status_ = BT::NodeStatus::IDLE;

    message = "Shelf mission started";
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  bool cancelMission(std::string &message) {
    std::scoped_lock<std::mutex> lock(mutex_);

    if (!running_) {
      message = "Mission is not running";
      return false;
    }

    try {
      if (tree_) {
        tree_->haltTree();
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Exception while halting tree: %s", e.what());
    }

    running_ = false;
    last_status_ = BT::NodeStatus::IDLE;
    tree_.reset();
    blackboard_.reset();

    message = "Shelf mission canceled";
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return true;
  }

  void tickTree() {
    std::scoped_lock<std::mutex> lock(mutex_);

    if (!running_ || !tree_) {
      return;
    }

    BT::NodeStatus status = BT::NodeStatus::IDLE;

    try {
      status = tree_->tickRoot();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "BT tick exception: %s", e.what());
      running_ = false;
      tree_.reset();
      blackboard_.reset();
      return;
    }

    last_status_ = status;

    if (status == BT::NodeStatus::RUNNING) {
      return;
    }

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Shelf mission finished with SUCCESS");
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(get_logger(), "Shelf mission finished with FAILURE");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Shelf mission finished with unexpected status");
    }

    try {
      tree_->haltTree();
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Exception while halting finished tree: %s",
                  e.what());
    }

    running_ = false;
    tree_.reset();
    blackboard_.reset();
  }

  void handleStart(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::string msg;
    response->success = startMission(msg);
    response->message = msg;
  }

  void handleCancel(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::string msg;
    response->success = cancelMission(msg);
    response->message = msg;
  }

private:
  std::mutex mutex_;

  std::string tree_xml_file_;
  int tick_period_ms_{100};
  int server_timeout_ms_{2000};
  int wait_for_service_timeout_ms_{2000};
  bool autostart_{false};
  std::vector<std::string> plugin_libs_;

  bool plugins_loaded_{false};
  bool running_{false};

  BT::BehaviorTreeFactory factory_;
  std::shared_ptr<BT::Blackboard> blackboard_;
  std::shared_ptr<BT::Tree> tree_;
  BT::NodeStatus last_status_{BT::NodeStatus::IDLE};

  rclcpp::Node::SharedPtr self_node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ShelfMissionExecutor>();
  node->initialize(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}