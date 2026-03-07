#ifndef MOTION_API_PKG__MOTION_API_NODE_HPP_
#define MOTION_API_PKG__MOTION_API_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_motion_msgs/action/move_joints.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"

// MotionApiNode 继承自 rclcpp::Node，表示它是一个 ROS2 节点
class MotionApiNode : public rclcpp::Node
{
public:
    using MoveJoints = robot_motion_msgs::action::MoveJoints;
    using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;

    // NodeOptions ROS2 节点常见写法，方便以后扩展 命名空间 参数重映射 组件化加载
    explicit MotionApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
    // 收到 Goal 请求时调用：决定接受还是拒绝
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveJoints::Goal> goal);

    // 收到取消请求时调用：决定是否允许取消
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

    // Goal 被接受后调用：通常在这里启动异步执行
    void handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

    // 真正执行动作逻辑的函数
    void execute(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

    // 校验客户端发来的 goal 是否合法
    bool validate_goal(const std::shared_ptr<const MoveJoints::Goal> & goal, std::string & reason) const;

    // 生成任务 ID
    std::string generate_task_id();

    // 发布系统状态到 topic
    void publish_system_state(const std::string & task_id, const std::string & state, const std::string & message, bool is_error);

private:
    // ===== 配置参数 =====
    std::string action_name_;              // action 名称
    std::string system_state_topic_;       // 状态 topic 名称
    double default_timeout_sec_;           // 默认超时时间
    double feedback_period_sec_;           // 反馈周期
    double min_speed_scale_;               // 最小速度比例
    double max_speed_scale_;               // 最大速度比例
    int expected_joint_count_;             // 期望的关节数量

  std::atomic<uint64_t> task_counter_;     // 原子计数器：用于生成 task_id，保证多线程安全

  // 状态消息发布器
  rclcpp::Publisher<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_pub_;
  // Action 服务器
  rclcpp_action::Server<MoveJoints>::SharedPtr action_server_;
};

#endif  // MOTION_API_PKG__MOTION_API_NODE_HPP_