/*
 * rtelligent-ect60-od.hpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_

#include <inttypes.h>

#include <map>

#include "ecat_server/soem_impl.hpp"

namespace EtherCAT
{
namespace ECT60
{

inline constexpr std::map<OD_KEY, DictionaryObjectType> manufacturer_od = {};

}  // namespace ECT60
}  // namespace EtherCAT

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_ */
