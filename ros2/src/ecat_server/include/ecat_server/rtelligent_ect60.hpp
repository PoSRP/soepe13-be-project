/*
 * rtelligent_ect60.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_

#include <cstdint>

namespace soem_impl
{

namespace ECT60
{

static bool setup(uint16_t slave);

}  // namespace ECT60

}  // namespace EtherCAT

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_ */
