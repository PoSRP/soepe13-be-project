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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ecat_interfaces/msg/network_error.hpp"
#include "ecat_interfaces/msg/master_configuration.hpp"
#include "ecat_interfaces/msg/master_status.hpp"
#include "ecat_interfaces/msg/slave_configuration.hpp"
#include "ecat_interfaces/msg/slave_status.hpp"

#include "ecat_interfaces/srv/get_master_status.hpp"
#include "ecat_interfaces/srv/get_master_configuration.hpp"
#include "ecat_interfaces/srv/set_master_configuration.hpp"
#include "ecat_interfaces/srv/get_connected_slaves.hpp"
#include "ecat_interfaces/srv/get_slave_status.hpp"
#include "ecat_interfaces/srv/get_slave_configuration.hpp"
#include "ecat_interfaces/srv/get_slave_configurations.hpp"
#include "ecat_interfaces/srv/set_slave_configuration.hpp"


class EcatServer : public rclcpp::Node
{
public:
  EcatServer();

private:
  // Message publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _log_publisher;
  rclcpp::Publisher<ecat_interfaces::msg::NetworkError>::SharedPtr _network_error_publisher;
  rclcpp::Publisher<ecat_interfaces::msg::MasterStatus>::SharedPtr _master_status_publisher;
  rclcpp::Publisher<ecat_interfaces::msg::MasterConfiguration>::SharedPtr
    _master_configuration_publisher;
  rclcpp::Publisher<ecat_interfaces::msg::SlaveStatus>::SharedPtr _slave_status_publisher;
  rclcpp::Publisher<ecat_interfaces::msg::SlaveConfiguration>::SharedPtr
    _slave_configuration_publisher;

  // Service pointers - Slaves
  rclcpp::Service<ecat_interfaces::srv::GetConnectedSlaves>::SharedPtr _get_connected_slaves_service;
  rclcpp::Service<ecat_interfaces::srv::GetSlaveStatus>::SharedPtr _get_slave_status_service;
  rclcpp::Service<ecat_interfaces::srv::GetSlaveConfiguration>::SharedPtr
    _get_slave_configuration_service;
  rclcpp::Service<ecat_interfaces::srv::GetSlaveConfigurations>::SharedPtr
    _get_slave_configurations_service;
  rclcpp::Service<ecat_interfaces::srv::SetSlaveConfiguration>::SharedPtr
    _set_slave_configuration_service;

  // Service functions - Slaves
  void get_connected_slaves(
    const std::shared_ptr<ecat_interfaces::srv::GetConnectedSlaves::Request> request,
    std::shared_ptr<ecat_interfaces::srv::GetConnectedSlaves::Response> response);
  void get_slave_status(
    const std::shared_ptr<ecat_interfaces::srv::GetSlaveStatus::Request> request,
    std::shared_ptr<ecat_interfaces::srv::GetSlaveStatus::Response> response);
  void get_slave_configuration(
    const std::shared_ptr<ecat_interfaces::srv::GetSlaveConfiguration::Request> request,
    std::shared_ptr<ecat_interfaces::srv::GetSlaveConfiguration::Response> response);
  void get_slave_configurations(
    const std::shared_ptr<ecat_interfaces::srv::GetSlaveConfigurations::Request> request,
    std::shared_ptr<ecat_interfaces::srv::GetSlaveConfigurations::Response> response);
  void set_slave_configuration(
    const std::shared_ptr<ecat_interfaces::srv::SetSlaveConfiguration::Request> request,
    std::shared_ptr<ecat_interfaces::srv::SetSlaveConfiguration::Response> response);

  // Services pointers - Master
  rclcpp::Service<ecat_interfaces::srv::GetMasterStatus>::SharedPtr _get_master_status_service;
  rclcpp::Service<ecat_interfaces::srv::GetMasterConfiguration>::SharedPtr
    _get_master_configuration_service;
  rclcpp::Service<ecat_interfaces::srv::SetMasterConfiguration>::SharedPtr
    _set_master_configuration_service;

  // Service functions - Master
  void get_master_status(
    const std::shared_ptr<ecat_interfaces::srv::GetMasterStatus::Request> request,
    std::shared_ptr<ecat_interfaces::srv::GetMasterStatus::Response> response);
  void get_master_configuration(
    const std::shared_ptr<ecat_interfaces::srv::GetMasterConfiguration::Request> request,
    std::shared_ptr<ecat_interfaces::srv::GetMasterConfiguration::Response> response);
  void set_master_configuration(
    const std::shared_ptr<ecat_interfaces::srv::SetMasterConfiguration::Request> request,
    std::shared_ptr<ecat_interfaces::srv::SetMasterConfiguration::Response> response);

  // Actions
  void start();
  void stop();
  void reload();
  void move();

  // Parameters
  std::string _network_interface_param;

  // Member variables
  size_t _count;
};


#endif /* INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_ */
