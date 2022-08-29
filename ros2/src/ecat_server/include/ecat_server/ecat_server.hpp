/*
 * ecat_server.hpp
 *
 *  Created on: Jul 14, 2022
 *      Author: sr
 */

#ifndef INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_
#define INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_

#include <iostream>
#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <unistd.h>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "ecat_interfaces/action/execute_move.hpp"


namespace EtherCAT {

class Server : public rclcpp::Node
{
public:
  using ExecuteMove = ecat_interfaces::action::ExecuteMove;
  using ExecuteMoveGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMove>;

  explicit Server(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Message publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _log_publisher;

  // Actions
  rclcpp_action::Server<ExecuteMove>::SharedPtr _execute_move_action_server;
  rclcpp_action::GoalResponse _handle_execute_move_goal(
		    const rclcpp_action::GoalUUID & uuid,
		    std::shared_ptr<const ExecuteMove::Goal> goal);
  rclcpp_action::CancelResponse _handle_execute_move_cancel(
    const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle);
  void _handle_execute_move_accepted(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle);
  void _execute_move(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle);

  // Parameters
  std::string _network_interface_param;
  uint64_t _network_cycle_time_us;

  // Member variables
  std::string _node_name;
  std::atomic_bool _execution_active;
};

} /* ns EtherCAT */

#endif /* INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_ */
