/*
 * rtelligent-ect60-od.hpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "ecat_server/generic_od.hpp"
#include "ecat_server/soem_impl.hpp"

namespace soem_impl::rtelligent::ect60
{

inline static const std::vector<const od_obj_t> od = {
  soem_impl::od_obj<uint32_t>(0x1000, "", soem_impl::OD_OBJ_CAT::REQUIRED, {})};

}  // namespace soem_impl::rtelligent::ect60

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_ */
