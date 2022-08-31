/*
 * soem_impl.hpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_HPP_

#include <assert.h>
#include <inttypes.h>

#include <map>
#include <memory>
#include <variant>
#include <vector>

#include "ecat_server/soem_impl.tpp"

namespace EtherCAT
{

struct controlword_t
{
  bool switch_on : 1;         /* Bit 0 */
  bool enable_voltage : 1;    /* Bit 1 */
  bool quick_stop : 1;        /* Bit 2 */
  bool enable_operation : 1;  /* Bit 3 */
  bool op_mode_specific0 : 1; /* Bit 4 */
  bool op_mode_specific1 : 1; /* Bit 5 */
  bool op_mode_specific2 : 1; /* Bit 6 */
  bool fault_reset : 1;       /* Bit 7 */
  bool halt : 1;              /* Bit 8 */
  bool op_mode_specific3 : 1; /* Bit 9 */
  bool __reserved__ : 1;      /* Bit 10 - Reserved and never used */
  bool mfr_specific0 : 1;     /* Bit 11 */
  bool mfr_specific1 : 1;     /* Bit 12 */
  bool mfr_specific2 : 1;     /* Bit 13 */
  bool mfr_specific3 : 1;     /* Bit 14 */
  bool mfr_specific4 : 1;     /* Bit 15 */

  uint16_t uint16()
  {
    uint16_t ret{0};
    ret += switch_on << 0;
    ret += enable_voltage << 1;
    ret += quick_stop << 2;
    ret += enable_operation << 3;
    ret += op_mode_specific0 << 4;
    ret += op_mode_specific1 << 5;
    ret += op_mode_specific2 << 6;
    ret += fault_reset << 7;
    ret += halt << 8;
    ret += op_mode_specific3 << 9;
    ret += mfr_specific0 << 11;
    ret += mfr_specific1 << 12;
    ret += mfr_specific2 << 13;
    ret += mfr_specific3 << 14;
    ret += mfr_specific4 << 15;
    return ret;
  }
};
static_assert(sizeof(controlword_t) == sizeof(uint16_t));

struct statusword_t
{
  bool ready_to_switch_on : 1;    /* Bit 0 */
  bool switched_on : 1;           /* Bit 1 */
  bool operation_enabled : 1;     /* Bit 2 */
  bool fault : 1;                 /* Bit 3 */
  bool voltage_enabled : 1;       /* Bit 4 */
  bool quick_stop : 1;            /* Bit 5 */
  bool switch_on_disabled : 1;    /* Bit 6 */
  bool warning : 1;               /* Bit 7 */
  bool mfr_specific0 : 1;         /* Bit 8 */
  bool remote : 1;                /* Bit 9 */
  bool target_reached : 1;        /* Bit 10 */
  bool internal_limit_active : 1; /* Bit 11 */
  bool op_mode_specific0 : 1;     /* Bit 12 */
  bool op_mode_specific1 : 1;     /* Bit 13 */
  bool mfr_specific1 : 1;         /* Bit 14 */
  bool mfr_specific2 : 1;         /* Bit 15 */

  void set(uint16_t new_value)
  {
    ready_to_switch_on = (0b0000000000000001 & new_value);
    switched_on = (0b0000000000000010 & new_value) >> 1;
    operation_enabled = (0b0000000000000100 & new_value) >> 2;
    fault = (0b0000000000001000 & new_value) >> 3;
    voltage_enabled = (0b0000000000010000 & new_value) >> 4;
    quick_stop = (0b0000000000100000 & new_value) >> 5;
    switch_on_disabled = (0b0000000001000000 & new_value) >> 6;
    warning = (0b0000000010000000 & new_value) >> 7;
    mfr_specific0 = (0b0000000100000000 & new_value) >> 8;
    remote = (0b0000001000000000 & new_value) >> 9;
    target_reached = (0b0000010000000000 & new_value) >> 10;
    internal_limit_active = (0b0000100000000000 & new_value) >> 11;
    op_mode_specific0 = (0b0001000000000000 & new_value) >> 12;
    op_mode_specific1 = (0b0010000000000000 & new_value) >> 13;
    mfr_specific1 = (0b0100000000000000 & new_value) >> 14;
    mfr_specific2 = (0b1000000000000000 & new_value) >> 15;
  }
  uint16_t uint16()
  {
    uint16_t ret{0};
    ret += ready_to_switch_on << 0;
    ret += switched_on << 1;
    ret += operation_enabled << 2;
    ret += fault << 3;
    ret += voltage_enabled << 4;
    ret += quick_stop << 5;
    ret += switch_on_disabled << 6;
    ret += warning << 7;
    ret += mfr_specific0 << 8;
    ret += remote << 9;
    ret += target_reached << 10;
    ret += internal_limit_active << 11;
    ret += op_mode_specific0 << 12;
    ret += op_mode_specific1 << 13;
    ret += mfr_specific1 << 14;
    ret += mfr_specific2 << 15;
    return ret;
  }
};
static_assert(sizeof(statusword_t) == sizeof(uint16_t));

struct __attribute__((__packed__)) cyclic_position_output_t
{
  controlword_t controlword;  // 0x6040h
  int32_t target_position;    // 0x607Ah
  int8_t operationmode;       // 0x6060h
};
struct __attribute__((__packed__)) cyclic_position_input_t
{
  statusword_t statusword;       // 0x6041h
  int32_t current_position;      // 0x6064h
  int8_t operationmode_display;  // 0x6061h
};

static const std::map<EtherCAT::OD_KEY, EtherCAT::DictionaryObjectType> generic_od;

}  // namespace EtherCAT

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_HPP_ */
