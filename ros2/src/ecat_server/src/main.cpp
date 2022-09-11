/*
 * main.cpp
 *
 *  Created on: Jul 14, 2022
 *      Author: sr
 */

#include "ecat_server/ecat_server.hpp"
#include "ecat_server/rtelligent_ect60.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  std::cout << "Hello world from ecat_server package" << std::endl;

  // Call before anything else
  soem_impl::ect60::init_od();

  // Call during PRE_OP as a setup callback function
  soem_impl::ect60::setup(1, {{0x1000, 0}, {0x1001, 0}});

  rclcpp::init(argc, argv);
  auto srv = std::make_shared<soem_impl::Server>();
  rclcpp::spin(srv);
  rclcpp::shutdown();

  return 0;
}
