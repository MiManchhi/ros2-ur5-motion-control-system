#include "motion_api_pkg/motion_api_node.hpp"
#include "robot_common_pkg/constants.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <thread>
#include <utility>

using namespace std::chrono_literals;
using namespace robot_common_pkg::constants;

MotionApiNode::MotionApiNode(const rclcpp::NodeOptions &options)
    : Node("motion_api_node", options)
    , task_counter_(0)
{
	// 声明参数：告诉 ROS2 这个节点支持哪些配置项，并设置默认值
	this->declare_parameter<std::string>("action_name", "/move_joints");
	this->declare_parameter<std::string>("system_state_topic", "/system_state");
	this->declare_parameter<double>("default_timeout_sec", 10.0);
	this->declare_parameter<double>("feedback_period_sec", 0.5);
	this->declare_parameter<double>("min_speed_scale", 0.1);
	this->declare_parameter<double>("max_speed_scale", 1.0);
	this->declare_parameter<int>("expected_joint_count", 6);

	// 读取参数：保存到成员变量，方便后续使用
	action_name_          = this->get_parameter("action_name").as_string();
	system_state_topic_   = this->get_parameter("system_state_topic").as_string();
	default_timeout_sec_  = this->get_parameter("default_timeout_sec").as_double();
	feedback_period_sec_  = this->get_parameter("feedback_period_sec").as_double();
	min_speed_scale_      = this->get_parameter("min_speed_scale").as_double();
	max_speed_scale_      = this->get_parameter("max_speed_scale").as_double();
	expected_joint_count_ = this->get_parameter("expected_joint_count").as_int();

    // 创建 publisher：后续通过这个对象向 /system_state 发布状态
	system_state_pub_ = this->create_publisher<robot_motion_msgs::msg::SystemState>(system_state_topic_, 10);

    // 创建 Action Server：注册三个核心回调
	action_server_ = rclcpp_action::create_server<MoveJoints>(
		this,
		action_name_,
		std::bind(&MotionApiNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&MotionApiNode::handle_cancel, this, std::placeholders::_1),
		std::bind(&MotionApiNode::handle_accepted, this, std::placeholders::_1));

	RCLCPP_INFO(this->get_logger(), "MotionApiNode started.");
	RCLCPP_INFO(this->get_logger(), "Action server: %s", action_name_.c_str());
	RCLCPP_INFO(this->get_logger(), "System state topic: %s", system_state_topic_.c_str());
}

rclcpp_action::GoalResponse MotionApiNode::handle_goal(
	const rclcpp_action::GoalUUID & uuid,
  	std::shared_ptr<const MoveJoints::Goal> goal)
{
	(void)uuid;

	RCLCPP_INFO(
		this->get_logger(),
		"Received MoveJoints goal: task_name=%s",
		goal->task_name.c_str());

	std::string reason;
	if (!validate_goal(goal, reason))
	{
		RCLCPP_WARN(this->get_logger(), "Goal rejected: %s", reason.c_str());
		publish_system_state("", state::kRejected, reason, true);
		return rclcpp_action::GoalResponse::REJECT;
	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionApiNode::handle_cancel(
  	const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
	(void)goal_handle;
	RCLCPP_WARN(this->get_logger(), "Received request to cancel goal.");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionApiNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  	std::thread(&MotionApiNode::execute, this, goal_handle).detach();
}

void MotionApiNode::execute(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
	const auto goal = goal_handle->get_goal();
	const std::string task_id = generate_task_id();

	RCLCPP_INFO(
		this->get_logger(),
		"Accepted task: task_id=%s, task_name=%s",
		task_id.c_str(),
		goal->task_name.c_str());

	publish_system_state(task_id, state::kAccepted, "task accepted by motion_api_node", false);

	auto feedback = std::make_shared<MoveJoints::Feedback>();
	auto result = std::make_shared<MoveJoints::Result>();

	constexpr int total_steps = 5;
	for (int step = 1; step <= total_steps; ++step)
	{
		if (goal_handle->is_canceling())
		{
			result->success = false;
			result->task_id = task_id;
			result->message = "task canceled";
			result->final_error = 0.0;

			publish_system_state(task_id, state::kCanceled, "task canceled by client", true);
			goal_handle->canceled(result);

			RCLCPP_WARN(this->get_logger(), "Task canceled: %s", task_id.c_str());
			return;
		}

		if (step == 1)
		{
			publish_system_state(task_id, state::kPlanning, "motion api validating and forwarding task", false);
		}
		else if (step < total_steps)
		{
			publish_system_state(task_id, state::kExecuting, "stage2 mock execution in progress", false);
		}

		feedback->task_id = task_id;
		feedback->current_state = (step == 1) ? state::kPlanning : state::kExecuting;
		feedback->progress = static_cast<double>(step) / static_cast<double>(total_steps);
		feedback->current_error = static_cast<double>(total_steps - step) * 0.01;

		goal_handle->publish_feedback(feedback);

		RCLCPP_INFO(
		this->get_logger(),
		"Feedback: task_id=%s state=%s progress=%.2f error=%.4f",
		feedback->task_id.c_str(),
		feedback->current_state.c_str(),
		feedback->progress,
		feedback->current_error);

		std::this_thread::sleep_for(std::chrono::duration<double>(feedback_period_sec_));
	}

	result->success = true;
	result->task_id = task_id;
	result->message = "stage2 motion api mock execution completed";
	result->final_error = 0.0;

	publish_system_state(task_id, state::kCompleted, "task completed in motion_api stage2 mock flow", false);
	goal_handle->succeed(result);

	RCLCPP_INFO(this->get_logger(), "Task completed: task_id=%s", task_id.c_str());
}

bool MotionApiNode::validate_goal(
  	const std::shared_ptr<const MoveJoints::Goal> & goal,
  	std::string & reason) const
{
    if (goal->joint_names.empty())
    {
		reason = "joint_names must not be empty";
		return false;
	}

	if (goal->target_positions.empty())
	{
		reason = "target_positions must not be empty";
		return false;
	}

	if (goal->joint_names.size() != goal->target_positions.size())
	{
		reason = "joint_names size must equal target_positions size";
		return false;
	}

	if (expected_joint_count_ > 0 && static_cast<int>(goal->joint_names.size()) != expected_joint_count_)
	{
		std::ostringstream oss;
		oss << "joint count mismatch, expected " << expected_joint_count_
			<< ", got " << goal->joint_names.size();
		reason = oss.str();
		return false;
	}

	if (goal->speed_scale < min_speed_scale_ ||
		goal->speed_scale > max_speed_scale_)
	{
		std::ostringstream oss;
		oss << "speed_scale out of range [" << min_speed_scale_
			<< ", " << max_speed_scale_ << "]";
		reason = oss.str();
		return false;
	}

	const double timeout_sec = (goal->timeout_sec > 0.0) ? goal->timeout_sec : default_timeout_sec_;

	if (timeout_sec <= 0.0)
	{
		reason = "timeout_sec must be > 0";
		return false;
	}

	reason = "ok";
	return true;
}

std::string MotionApiNode::generate_task_id()
{
	const auto now_ns = this->now().nanoseconds();
	const auto id = ++task_counter_;

	std::ostringstream oss;
	oss << "task_" << now_ns << "_" << id;
	return oss.str();
}

void MotionApiNode::publish_system_state(
  	const std::string & task_id,
  	const std::string & state,
  	const std::string & message,
  	bool is_error)
{
	robot_motion_msgs::msg::SystemState msg;
	msg.task_id  = task_id;
	msg.state    = state;
	msg.message  = message;
	msg.is_error = is_error;
	msg.stamp    = this->now();

	system_state_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotionApiNode>());
	rclcpp::shutdown();
	return 0;
}