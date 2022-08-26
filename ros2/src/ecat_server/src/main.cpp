/*
 * main.cpp
 *
 *  Created on: Jul 14, 2022
 *      Author: sr
 */

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "ecat_server/ecat_server.hpp"

int main(int argc, char ** argv)
{
  std::cout << "Hello world from ecat_server package" << std::endl;

  rclcpp::init(argc, argv);
  auto srv = std::make_shared<EcatServer>(getpid());
  rclcpp::spin(srv);
  rclcpp::shutdown();

  return 0;
}
