/*
 * ecat_server.hpp
 *
 *  Created on: Jul 14, 2022
 *      Author: sr
 */

#ifndef INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_
#define INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ecat_interfaces/action/execute_move.hpp"
#include "ecat_interfaces/srv/network_ctrl.hpp"
#include "ecat_interfaces/srv/start_network.hpp"
#include "ecat_interfaces/srv/stop_network.hpp"
#include "ethercat.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "soem_impl.hpp"
#include "std_msgs/msg/string.hpp"

namespace soem_impl
{

class Server : public rclcpp::Node
{
public:
  using ExecuteMove = ecat_interfaces::action::ExecuteMove;
  using ExecuteMoveGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMove>;

  explicit Server(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  Server(const Server &) = delete;
  Server & operator=(const Server &) = delete;
  Server & operator=(Server &) = delete;

  // Topics
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _log_publisher;
  // TODO: Custom message type
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _network_status_publisher;
  // TODO: Custom message type
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _execute_move_publisher;

  // Service - Network Control
  rclcpp::Service<ecat_interfaces::srv::NetworkCtrl>::SharedPtr _srv_network_ctrl;
  void _handle_srv_network_ctrl(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ecat_interfaces::srv::NetworkCtrl::Request> request,
    std::shared_ptr<ecat_interfaces::srv::NetworkCtrl::Response> response);

  // Actions - Execute Move
  rclcpp_action::Server<ExecuteMove>::SharedPtr _execute_move_action_server;
  rclcpp_action::GoalResponse _handle_execute_move_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMove::Goal> goal);
  rclcpp_action::CancelResponse _handle_execute_move_cancel(
    const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle);
  void _handle_execute_move_accepted(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle);
  void _execute_move(const std::shared_ptr<ExecuteMoveGoalHandle> goal_handle);

  // Parameters
  std::string _network_interface_param{};
  uint64_t _network_cycle_period_us_param{};
  std::vector<std::pair<std::string, std::string>> slave_params{};

  // Member variables
  std::string _node_name;
  char io_map[4096];
  std::atomic_bool _is_initialized{false};
  std::atomic_bool _is_terminating{false};
  std::chrono::time_point<std::chrono::high_resolution_clock> _next_buffer_read{};

  inline static std::atomic<bool> _network_active{false};
  inline static std::atomic<bool> _move_active{false};

  // SOEM backend
  master_node ecat_master;
  //  OSAL_THREAD_HANDLE _ecat_thread{nullptr};
  //  OSAL_THREAD_HANDLE _osal_thread{nullptr};
  //  inline static int _expected_wkc{0};
  //  inline static volatile int _wkc{0};
  //  static OSAL_THREAD_FUNC _ecat_thread_function();
  //  static OSAL_THREAD_FUNC _osal_thread_function();
  //  static void _ecat_init();
};

}  // namespace soem_impl

#endif /* INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_ */
