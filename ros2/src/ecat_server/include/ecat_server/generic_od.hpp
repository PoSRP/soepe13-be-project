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

namespace soem_impl
{

enum class OD_OBJ_PERM { RO, WO, RW };
enum class OD_OBJ_PDO { OPTIONAL, REQUIRED, NO };
enum class OD_OBJ_CAT { OPTIONAL, REQUIRED };

template <class T>
class od_obj_ent
{
public:
  using type = T;

  od_obj_ent(
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
  ~od_obj_ent() {}

  const uint8_t sub_index;
  const std::optional<T> range_min;
  const std::optional<T> range_max;
  const std::optional<T> default_value;
  const OD_OBJ_PERM permission;
  const OD_OBJ_PDO pdo;
  const OD_OBJ_CAT category;
  const char * description;
};

template <class T>
class od_obj
{
public:
  using type = T;

  od_obj(uint16_t && index, const char * desc, OD_OBJ_CAT && cat)
  : index(index), description(desc), category(cat)
  {
  }
  ~od_obj() {}
  void add_entry(const od_obj_ent<T> & entry) { entries.push_back(entry); }
  const std::vector<od_obj_ent<T>> & get_entries() const { return entries; }

  const uint16_t index;
  const char * description;
  const OD_OBJ_CAT category;

private:
  std::vector<od_obj_ent<T>> entries{};
};

template <class... Ts>
using _var_t_ = std::variant<od_obj<Ts>...>;
template <class... Ts>
using _ent_var_t_ = std::variant<od_obj_ent<Ts>...>;
using od_obj_t = _var_t_<int8_t, int16_t, int32_t, uint8_t, uint16_t, uint32_t, const char *>;
using od_obj_ent_t =
  _ent_var_t_<int8_t, int16_t, int32_t, uint8_t, uint16_t, uint32_t, const char *>;
template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// TODO: Write static od_obj_visitor?

// TODO: This will be painful - and probably incomplete without
// 		 access to the latest draft standard proposal :(
//static const std::vector<od_obj_t> generic_od;

enum SM_DIR { IN, OUT };

class sm_conf_t
{
public:
  sm_conf_t(uint8_t && id, uint16_t && addr, SM_DIR && dir, std::vector<od_obj_t> objects)
  : id(id), addr(addr), dir(dir), objects(objects)
  {
  }
  ~sm_conf_t() {}
  const uint8_t id;
  const uint16_t addr;
  const SM_DIR dir;
  const std::vector<od_obj_t> objects;
};

}  //namespace soem_impl

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_GENERIC_OD_HPP_ */
