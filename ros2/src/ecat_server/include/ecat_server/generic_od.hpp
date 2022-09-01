/*
 * generic_od.hpp
 *
 *  Created on: Sep 1, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_GENERIC_OD_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_GENERIC_OD_HPP_

#include <map>
#include <memory>
#include <optional>
#include <type_traits>
#include <variant>
#include <vector>

namespace EtherCAT
{

enum class OD_OBJ_PERM { RO, WO, RW };
enum class OD_OBJ_PDO { OPTIONAL, REQUIRED, NO };
enum class OD_OBJ_CAT { OPTIONAL, REQUIRED };

template <class T>
struct OD_OBJ
{
  using type = T;

  OD_OBJ(uint16_t && index, OD_OBJ_CAT && cat, const char * desc, std::vector<T> && entries)
  : index(index), category(cat), description(desc), entries(entries)
  {
  }

  const uint16_t index;
  const OD_OBJ_CAT category;
  const char * description;
  const std::vector<T> entries;
};

template <class T>
struct OD_OBJ_ENT
{
  using type = T;

  OD_OBJ_ENT(
    uint8_t sub_index, std::optional<T> range_min, std::optional<T> range_max,
    std::optional<T> default_value, OD_OBJ_PERM perm, OD_OBJ_PDO pdo, OD_OBJ_CAT cat,
    const char * desc)
  : sub_index(sub_index),
    range_min(range_min),
    range_max(range_max),
    default_value(default_value),
    permission(perm),
    pdo(pdo),
    category(cat),
    description(desc)
  {
  }

  const uint8_t sub_index;
  const std::optional<T> range_min;
  const std::optional<T> range_max;
  const std::optional<T> default_value;
  const OD_OBJ_PERM permission;
  const OD_OBJ_PDO pdo;
  const OD_OBJ_CAT category;
  const char * description;
};

template <class... Ts>
using _var_t_ = std::variant<OD_OBJ<Ts>...>;
using OB_OBJ_TYPES = _var_t_<int8_t, int16_t, int32_t, uint8_t, uint16_t, uint32_t, const char *>;
template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

static const std::map<uint64_t, OB_OBJ_TYPES> generic_od = {
  {0x1000,
   OD_OBJ<uint32_t>(
     0x1000, OD_OBJ_CAT::REQUIRED, "Device type",
     {OD_OBJ_ENT<uint32_t>(
       0, std::nullopt, std::nullopt, std::nullopt, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO,
       OD_OBJ_CAT::REQUIRED, "Device type")})},
  {0x1001,
   OD_OBJ<uint8_t>(
     0x1001, OD_OBJ_CAT::REQUIRED, "Error register",
     {OD_OBJ_ENT<uint8_t>(
       0, std::nullopt, std::nullopt, std::nullopt, OD_OBJ_PERM::RO, OD_OBJ_PDO::OPTIONAL,
       OD_OBJ_CAT::REQUIRED, "Error register")})},
  {0x1002, OD_OBJ<uint32_t>(0x1002, )}};

}  //namespace EtherCAT

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_GENERIC_OD_HPP_ */
