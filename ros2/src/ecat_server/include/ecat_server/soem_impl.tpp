/*
 * soem_impl.tpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_TPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_TPP_

#include <optional>

#include "ecat_server/soem_impl.hpp"

namespace EtherCAT
{

enum class DICT_OBJ_PERMISSION { READ_ONLY, READ_WRITE };

template <class T>
struct DictionaryObject
{
  using type = T;

  DictionaryObject(
    uint16_t && index, uint16_t && sub_index, T && range_min, T && range_max,
    DICT_OBJ_PERMISSION && permission, const char * description)
  : index(index),
    sub_index(sub_index),
    range_min(range_min),
    range_max(range_max),
    permission(permission),
    description(description){};

  const uint16_t index;
  const uint16_t sub_index;
  const T range_min;
  const T range_max;
  const DICT_OBJ_PERMISSION permission;
  const char * description;
};

template <class... Ts>
using one_type_of = std::variant<DictionaryObject<Ts>...>;
using DictionaryObjectType =
  one_type_of<const char *, int8_t, int16_t, int32_t, uint8_t, uint16_t, uint32_t>;

struct OD_KEY
{
  OD_KEY(uint16_t && index, uint16_t && sub_index) : index(index), sub_index(sub_index) {}
  const uint16_t index;
  const uint16_t sub_index;
  bool operator<(const OD_KEY & rhs) const
  {
    if (index < rhs.index || (index == rhs.index && sub_index < rhs.sub_index))
      return true;
    else
      return false;
  }
};

std::optional<const DictionaryObjectType> _getOdObj(OD_KEY key) const
{
  auto it = generic_od.find(key);
  if (it != generic_od.end()) {
    auto obj = generic_od.at(key);
    return std::optional<DictionaryObjectType>(obj);
  } else {
    return std::nullopt;
  }
}

}  // namespace EtherCAT

#endif  // ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_HPP_
