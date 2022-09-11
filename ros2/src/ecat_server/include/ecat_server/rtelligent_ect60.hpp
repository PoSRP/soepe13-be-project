/*
 * rtelligent_ect60.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "ecat_server/generic_od.hpp"
#include "ecat_server/soem_impl.hpp"

namespace soem_impl
{

class ect60
{
public:
  static std::vector<od_obj_t> od;
  static void init_od();
  static std::optional<od_obj_ent_t> find_obj(const std::pair<uint16_t, uint8_t> idx);
  static bool setup(
    const uint16_t slave, const std::vector<std::pair<uint16_t, uint8_t>> idx_list = {});
};

}  // namespace soem_impl

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_ */
