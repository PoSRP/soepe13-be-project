/*
 * rtelligent_ect60.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_

#include <inttypes.h>

#include <variant>
#include <vector>

#include "ethercat.h"
#include "soem_impl.hpp"

namespace soem_impl::rtelligent_ect60
{
enum class cmd_addr : uint16_t {
  CMD_FILTER_SET_S8 = 0x4129,
  CMD_FILTER_TIME_CONSTANT_S16 = 0x412a,
  SMOOTH_DRIVE_U8 = 0x412c,
  ELEC_DAMPER_FUNC_S8 = 0x4136,
  RESONANCE_SUPP_CTRL_FREQ_S16 = 0x4137,
  RESONANCE_SUPP_CTRL_GAIN_S16 = 0x4138,
  STARTING_SPEED_S32 = 0x4142
};

static std::vector<variant_types> setup_values;

bool ECT60_setup(uint16_t slave);

}  // namespace soem_impl::rtelligent_ect60

//#include <cstdint>
//#include <optional>
//#include <vector>
//
//#include "ecat_server/generic_od.hpp"
//#include "ecat_server/soem_impl.hpp"
//
//namespace soem_impl
//{
//
//class ect60
//{
//public:
//  static std::vector<od_obj_t> od;
//  static void init_od();
//  static std::optional<od_obj_ent_t> find_obj(const std::pair<uint16_t, uint8_t> idx);
//  static bool setup(
//    const uint16_t slave, const std::vector<std::pair<uint16_t, uint8_t>> idx_list = {});
//};
//
//}  // namespace soem_impl

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_HPP_ */
