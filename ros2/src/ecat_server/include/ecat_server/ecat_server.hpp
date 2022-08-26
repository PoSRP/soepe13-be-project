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


class EcatServer : public rclcpp::Node
{
public:
  EcatServer(uint64_t pid);

private:
  // Message publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _log_publisher;

  // Actions
  void execute_move();

  // Parameters
  std::string _network_interface_param;

  // Member variables
  std::string _node_name;
};


#endif /* INCLUDE_ECAT_SERVER_ECAT_SERVER_HPP_ */
