#include "ecat_server/ecat_server.hpp"

#include "ecat_server/soem_impl.hpp"

using namespace soem_impl;

Server::Server(const rclcpp::NodeOptions & options)
: Node("ecat_server_" + std::to_string(getpid())),
  _node_name("ecat_server_" + std::to_string(getpid())),
  ecat_master(_network_interface_param)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Message topics
  _log_publisher = create_publisher<std_msgs::msg::String>(_node_name + "/log", 10);

  _network_status_publisher =
    create_publisher<std_msgs::msg::String>(_node_name + "/network_status", 10);
  _execute_move_publisher =
    create_publisher<std_msgs::msg::String>(_node_name + "/execute_move_fb", 10);

  // Move action
  _execute_move_action_server = rclcpp_action::create_server<ExecuteMove>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), "execute_move",
    std::bind(&Server::_handle_execute_move_goal, this, _1, _2),
    std::bind(&Server::_handle_execute_move_cancel, this, _1),
    std::bind(&Server::_handle_execute_move_accepted, this, _1));

  // EtherCAT backend
  ecat_master.async_spin();
}

/**
 * Service NetworkCtrl
 **/
void Server::_handle_srv_network_ctrl(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<ecat_interfaces::srv::NetworkCtrl::Request> request,
  std::shared_ptr<ecat_interfaces::srv::NetworkCtrl::Response> response)
{
  using ecat_interfaces::srv::NetworkCtrl;

  switch (request->action) {
    case NetworkCtrl::Request::START:
      if (_network_active.load()) {
        const char * err_str = "Network control request rejected: Network already active";
        RCLCPP_ERROR(get_logger(), err_str);
        response->result = NetworkCtrl::Response::ERROR;
        response->msg = err_str;
      } else {
      }
      break;

    case NetworkCtrl::Request::STOP:
      if (!_network_active.load()) {
        const char * err_str = "Network control request rejected: Network already inactive";
        RCLCPP_ERROR(get_logger(), err_str);
        response->result = NetworkCtrl::Response::ERROR;
        response->msg = err_str;
      } else {
      }
      break;
  }
}

/**
 * Action ExecuteMove - Handle Goal
 **/
rclcpp_action::GoalResponse Server::_handle_execute_move_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMove::Goal> goal)
{
  if (!_network_active.load(std::memory_order_relaxed)) {
    RCLCPP_WARN(get_logger(), "Move request rejected, network is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto is_running = false;
  if (!_move_active.compare_exchange_strong(is_running, true, std::memory_order_relaxed)) {
    RCLCPP_WARN(get_logger(), "Move request rejected, another operation is active");
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
    RCLCPP_WARN(get_logger(), "Move request rejected, no data provided");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received move request with data: %s", data.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * Action ExecuteMove - Handle Cancel
 **/
rclcpp_action::CancelResponse Server::_handle_execute_move_cancel(
  const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle)
{
  auto result = std::make_shared<ExecuteMove::Result>();
  result->msg = "Move aborted by cancel request";
  RCLCPP_WARN(get_logger(), result->msg.c_str());
  goal_handle->canceled(result);
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * Action ExecuteMove - Handle Accept
 **/
void Server::_handle_execute_move_accepted(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle)
{
  using std::placeholders::_1;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Server::_execute_move, this, _1), goal_handle}.detach();
}

/**
 * Action ExecuteMove - Handle Execution
 **/
void Server::_execute_move(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle)
{
  constexpr int fb_array_max = 500;
  RCLCPP_INFO(get_logger(), "Executing move");

  // Note: This is not the execution cycle time, but the buffer update cycle time
  // TODO: Read next wake time set by OP thread, use that for sleep (_next_buffer_read)
  double rate_hz = 1 / (_network_cycle_period_us_param / 1000000.0);
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
      RCLCPP_WARN(get_logger(), result->msg.c_str());
      _move_active.exchange(false);
      return;
    }

    // TODO: This is where we do stuff

    time_fb.push_back(time_vec[i]);
    value_fb.push_back(value_vec[i]);

    if (time_fb.size() >= fb_array_max) {
      goal_handle->publish_feedback(feedback);
      std::string data{};
      for (std::size_t i = 0; i < time_fb.size(); i++) {
        data += std::to_string(counter) + ": (" + std::to_string(time_fb.at(i)) + ", " +
          std::to_string(value_fb.at(i)) + ")";
        if (i < goal->time_data_ns.size() - 1) {
          data += "; ";
        }
      }
      time_fb.clear();
      value_fb.clear();
      RCLCPP_INFO(get_logger(), "Sent feedback: %s", data.c_str());
    }

    loop_rate.sleep();
  }

  if (time_fb.size()) {
    goal_handle->publish_feedback(feedback);
    std::string data{};
    for (std::size_t i = 0; i < time_fb.size(); i++) {
      data += std::to_string(counter) + ": (" + std::to_string(time_fb.at(i)) + ", " +
        std::to_string(value_fb.at(i)) + ")";
      if (i < goal->time_data_ns.size() - 1) {
        data += "; ";
      }
    }
    time_fb.clear();
    value_fb.clear();
    RCLCPP_INFO(get_logger(), "Sent feedback: %s", data.c_str());
  }

  if (rclcpp::ok()) {
    result->msg = "Move completed";
    RCLCPP_INFO(get_logger(), result->msg.c_str());
  }
  _move_active.exchange(false);
}
