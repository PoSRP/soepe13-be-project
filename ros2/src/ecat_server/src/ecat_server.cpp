#include "ecat_server/ecat_server.hpp"

#include "ecat_server/soem_impl.hpp"

using namespace EtherCAT;

Server::Server(const rclcpp::NodeOptions & options)
: Node("ecat_server_" + std::to_string(getpid())),
  _network_interface_param(""),
  _network_cycle_time_us(1000),
  _node_name("ecat_server_" + std::to_string(getpid())),
  _execution_active(false),
  _network_active(false)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Message topics
  _log_publisher = create_publisher<std_msgs::msg::String>(_node_name + "/log", 10);

  // Execute move action
  _execute_move_action_server = rclcpp_action::create_server<ExecuteMove>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), "execute_move",
    std::bind(&Server::_handle_execute_move_goal, this, _1, _2),
    std::bind(&Server::_handle_execute_move_cancel, this, _1),
    std::bind(&Server::_handle_execute_move_accepted, this, _1));
}

rclcpp_action::GoalResponse Server::_handle_execute_move_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMove::Goal> goal)
{
  using namespace EtherCAT;
  auto obj = _getOdObj(OD_KEY(0, 0));
  if (obj) {
    RCLCPP_INFO(get_logger(), "OD read: %s", obj->description);
  }

  if (_execution_active.load()) {
    RCLCPP_INFO(get_logger(), "Move request rejected, another move is active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::string data{};
  for (std::size_t i = 0; i < goal->time_data_ns.size(); i++) {
    data += "(" + std::to_string(goal->time_data_ns.at(i)) + ", " +
      std::to_string(goal->position_data_um.at(i)) + ")";
    if (i < goal->time_data_ns.size() - 1) {
      data += "; ";
    }
  }

  if (data.empty()) {
    RCLCPP_INFO(get_logger(), "Move request rejected, no data provided");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received move request with data: %s", data.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Server::_handle_execute_move_cancel(
  const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle)
{
  auto result = std::make_shared<ExecuteMove::Result>();
  result->msg = "Move aborted by cancel request";
  RCLCPP_INFO(get_logger(), result->msg.c_str());
  goal_handle->canceled(result);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Server::_handle_execute_move_accepted(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle)
{
  using std::placeholders::_1;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Server::_execute_move, this, _1), goal_handle}.detach();
}

void Server::_execute_move(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle)
{
  _execution_active.exchange(true);
  RCLCPP_INFO(get_logger(), "Executing move");

  // Note: This is not the execution cycle time, but the buffer update cycle time
  // TODO: Read next wake time set by OP thread, use that for sleep (_next_buffer_read)
  double rate_hz = 1 / (_network_cycle_time_us / 1000000.0);
  rclcpp::Rate loop_rate(rate_hz);

  const auto goal = goal_handle->get_goal();
  const auto time_vec = goal->time_data_ns;
  const auto value_vec = goal->position_data_um;

  auto feedback = std::make_shared<ExecuteMove::Feedback>();
  auto & time_fb = feedback->time_data_ns;
  auto & value_fb = feedback->position_data_um;

  auto result = std::make_shared<ExecuteMove::Result>();
  int counter{0};
  /* TODO: This
   *       Prepare OP thread buffers
   *       Start network OP thread
   *       Fill/read buffers in a timely manner
   *       Feedback read buffer
   */
  for (std::size_t i = 0; (i < time_vec.size()) && rclcpp::ok(); i++) {
    if (goal_handle->is_canceling()) {
      result->msg = "Move was cancelled";
      RCLCPP_INFO(get_logger(), result->msg.c_str());
      _execution_active.exchange(false);
      return;
    }

    // TODO: This is where we do stuff

    time_fb.push_back(time_vec[i]);
    value_fb.push_back(value_vec[i]);
    goal_handle->publish_feedback(feedback);

    std::string data{};
    for (std::size_t i = 0; i < time_fb.size(); i++) {
      data += std::to_string(counter) + ": (" + std::to_string(time_fb.at(i)) + ", " +
        std::to_string(value_fb.at(i)) + ")";
      if (i < goal->time_data_ns.size() - 1) {
        data += "; ";
      }
    }
    RCLCPP_INFO(get_logger(), "Sent feedback: %s", data.c_str());

    time_fb.clear();
    value_fb.clear();
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->msg = "Move completed";
    RCLCPP_INFO(get_logger(), result->msg.c_str());
  }
  _execution_active.exchange(false);

  // TODO: Missing return?
}
