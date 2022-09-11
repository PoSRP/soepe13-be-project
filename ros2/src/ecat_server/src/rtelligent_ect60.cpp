/*
 * rtelligent_ect60.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: sr
 */

#include "ecat_server/rtelligent_ect60.hpp"

#include <algorithm>
#include <iostream>
#include <variant>

#include "ecat_server/generic_od.hpp"
#include "ecat_server/soem_impl.hpp"

using soem_impl::od_obj;
using soem_impl::OD_OBJ_CAT;
using soem_impl::od_obj_ent;
using soem_impl::od_obj_ent_t;
using soem_impl::OD_OBJ_PDO;
using soem_impl::OD_OBJ_PERM;
using soem_impl::od_obj_t;

std::vector<od_obj_t> soem_impl::ect60::od = {};

void soem_impl::ect60::init_od()
{
  od.clear();

  // 0x1000 - Device Type
  od_obj<uint32_t> idx_0x1000 = od_obj<uint32_t>(0x1000, "Device Type", OD_OBJ_CAT::REQUIRED);
  od_obj_ent<uint32_t> idx_0x1000_00 = od_obj_ent<uint32_t>(
    0, std::nullopt, std::nullopt, std::optional<uint32_t>(0), OD_OBJ_PERM::RO, OD_OBJ_PDO::NO,
    OD_OBJ_CAT::REQUIRED, "Device Type");
  idx_0x1000.add_entry(idx_0x1000_00);
  od.push_back(idx_0x1000);

  // 0x1001 - Error Register
  od_obj<const char *> idx_0x1001 =
    od_obj<const char *>(0x1001, "Error register", OD_OBJ_CAT::REQUIRED);
  od_obj_ent<const char *> idx_0x1001_00 = od_obj_ent<const char *>(
    0, std::nullopt, std::nullopt, std::optional<const char *>(""), OD_OBJ_PERM::RO, OD_OBJ_PDO::NO,
    OD_OBJ_CAT::REQUIRED, "Error Register");
  idx_0x1001.add_entry(idx_0x1001_00);
  od.push_back(idx_0x1001);
}

std::optional<od_obj_ent_t> soem_impl::ect60::find_obj(const std::pair<uint16_t, uint8_t> idx)
{
  for (auto & obj : od) {
    if (std::visit(
          [&](auto && arg) -> bool {
            if (arg.index == idx.first) {
              for (auto & ent : arg.get_entries()) {
                if (ent.sub_index == idx.second) return true;
              }
            }
            return false;
          },
          obj)) {
      return std::visit(
        [&](auto && arg) -> od_obj_ent_t { return arg.get_entries().at(idx.second); }, obj);
    }
  }
  return std::nullopt;
}

bool soem_impl::ect60::setup(
  const uint16_t slave, const std::vector<std::pair<uint16_t, uint8_t>> idx_list)
{
  (void)slave;
  std::vector<od_obj_ent_t> tmp;
  for (const auto & idx : idx_list) {
    if (auto obj = find_obj(idx); obj != std::nullopt)
      tmp.push_back(*obj);
    else
      return false;
  }
  return true;
}
