#include <memory>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_ros_control_interface/ControllerHandle.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace moveit_ros_control_interface {
class ForwardPositionControllerHandle
    : public moveit_controller_manager::MoveItControllerHandle {
public:
  ForwardPositionControllerHandle(const rclcpp::Node::SharedPtr &node,
                                  const std::string &name,
                                  const std::string &ns,
                                  const std::vector<std::string> &resources)
      : moveit_controller_manager::MoveItControllerHandle(name), node_(node),
        resources_(resources) {

    multi_array_publisher_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            ns.empty() ? name_ : name_ + "/" + ns, rclcpp::SystemDefaultsQoS());
  }

  bool
  sendTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override {
    RCLCPP_INFO_STREAM(node_->get_logger(), "New trajectory to " << name_);
    if (multi_array_publisher_->get_subscription_count() == 0) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "No subscribers to "
                              << multi_array_publisher_->get_topic_name());
      return false;
    }

    cancelExecution(); // Ensure any previous execution is stopped

    should_stop_.store(false, std::memory_order_release);
    is_complete_.store(false, std::memory_order_release);

    execution_future_ = std::async(
        std::launch::async, [&, trajectory = trajectory.joint_trajectory] {
          execute_trajectory(trajectory);
        });
    return true;
  }

  bool cancelExecution() override {
    {
      std::lock_guard<std::mutex> lock(cv_mutex_);
      should_stop_.store(true, std::memory_order_release);
    }
    cv_.notify_all();

    if (execution_future_.valid()) {
      execution_future_.wait();
    }
    return true;
  }

  bool waitForExecution(const rclcpp::Duration &timeout =
                            rclcpp::Duration::from_seconds(-1.0)) override {
    if (!execution_future_.valid()) {
      return true;
    }

    if (timeout.seconds() < 0) {
      execution_future_.wait();
    } else {
      const auto status = execution_future_.wait_for(
          timeout.to_chrono<std::chrono::duration<double>>());
      if (status == std::future_status::timeout) {
        return false;
      }
    }
    return true;
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override {
    if (execution_future_.valid() &&
        !should_stop_.load(std::memory_order_acquire) &&
        !is_complete_.load(std::memory_order_acquire)) {
      return moveit_controller_manager::ExecutionStatus::RUNNING;
    }
    return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      multi_array_publisher_;
  std::future<void> execution_future_;
  std::atomic<bool> should_stop_{false};
  std::atomic<bool> is_complete_{false};
  mutable std::condition_variable cv_;
  mutable std::mutex cv_mutex_;
  std::vector<std::string> resources_;

  void
  execute_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory) {

    std::vector<std::size_t> joint_indices;
    joint_indices.reserve(resources_.size());
    for (const auto &resource : resources_) {
      const auto it = std::find(trajectory.joint_names.begin(),
                                trajectory.joint_names.end(), resource);
      if (it == trajectory.joint_names.end()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "Resource '" << resource
                                         << "' not found in trajectory");
        return;
      }
      joint_indices.push_back(
          std::distance(trajectory.joint_names.begin(), it));
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(resources_.size());
    auto previous_duration = rclcpp::Duration::from_seconds(0.0);
    for (const auto &current_waypoint : trajectory.points) {
      for (std::size_t index = 0; index < joint_indices.size(); ++index) {
        msg.data[index] = current_waypoint.positions[joint_indices[index]];
      }
      multi_array_publisher_->publish(msg);

      // Wait until it's time to execute this waypoint or until stopped
      const auto current_duration =
          rclcpp::Duration(current_waypoint.time_from_start);
      {
        std::unique_lock<std::mutex> lock(cv_mutex_);
        cv_.wait_for(
            lock,
            (current_duration - previous_duration)
                .to_chrono<std::chrono::duration<double>>(),
            [this]() { return should_stop_.load(std::memory_order_acquire); });

        if (should_stop_.load(std::memory_order_acquire)) {
          return;
        }
      }
      previous_duration = current_duration;
    }
    is_complete_.store(true, std::memory_order_release);
  }
};

class ForwardPositionControllerAllocator : public ControllerHandleAllocator {
public:
  moveit_controller_manager::MoveItControllerHandlePtr
  alloc(const rclcpp::Node::SharedPtr &node, const std::string &name,
        const std::vector<std::string> &resources) override {
    return std::make_shared<ForwardPositionControllerHandle>(
        node, name, "commands", resources);
  }
};

} // namespace moveit_ros_control_interface

PLUGINLIB_EXPORT_CLASS(
    moveit_ros_control_interface::ForwardPositionControllerAllocator,
    moveit_ros_control_interface::ControllerHandleAllocator);
