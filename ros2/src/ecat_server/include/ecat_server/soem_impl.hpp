/*
 * soem_impl.hpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

//#include "ecat_server/generic_od.hpp"
#include "ethercat.h"

namespace soem_impl
{

#define EC_TIMEOUTMON 500

enum class slave_op_mode : int8_t { CYCLIC_POSITION = 8 };

enum class slave_op_state : uint16_t {
  NOT_READY_TO_SWITCH_ON = 0b0000000000000000,
  FAULT = 0b0000000000101000,
  FAULT_REACTION_ACTIVE = 0b0000000000101111,
  SWITCH_ON_DISABLED = 0b0000000001100000,
  READY_TO_SWITCH_ON = 0b0000000000100001,
  SWITCHED_ON = 0b0000000000100011,
  OPERATION_ENABLED = 0b0000000000100111,
  QUICK_STOP_ACTIVE = 0b0000000000000111,
  INVALID,
  MASK = 0b0000000001101111
};

const std::map<slave_op_state, const char *> slave_op_status_str{
  {slave_op_state::NOT_READY_TO_SWITCH_ON, "NOT_READY_TO_SWITCH_ON"},
  {slave_op_state::FAULT, "FAULT"},
  {slave_op_state::FAULT_REACTION_ACTIVE, "FAULT_REACTION_ACTIVE"},
  {slave_op_state::SWITCH_ON_DISABLED, "SWITCH_ON_DISABLED"},
  {slave_op_state::READY_TO_SWITCH_ON, "READY_TO_SWITCH_ON"},
  {slave_op_state::SWITCHED_ON, "SWITCHED_ON"},
  {slave_op_state::OPERATION_ENABLED, "OPERATION_ENABLED"},
  {slave_op_state::QUICK_STOP_ACTIVE, "QUICK_STOP_ACTIVE"},
  {slave_op_state::INVALID, "INVALID"},
  {slave_op_state::MASK, "MASK"}};

enum class slave_op_command : uint16_t {
  DISABLE_OPERATION,
  DISABLE_VOLTAGE,
  ENABLE_OPERATION,
  FAULT_RESET,
  QUICK_STOP,
  SHUTDOWN,
  SWITCH_ON,
  SWITCH_ON_AND_ENABLE_OPERATION,
};

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

inline const std::map<uint16_t, const char *> ec_state2string{
  {EC_STATE_ACK, "ACK"},       {EC_STATE_BOOT, "BOOT"},       {EC_STATE_ERROR, "ERROR"},
  {EC_STATE_INIT, "INIT"},     {EC_STATE_NONE, "NONE"},       {EC_STATE_OPERATIONAL, "OPERATIONAL"},
  {EC_STATE_PRE_OP, "PRE_OP"}, {EC_STATE_SAFE_OP, "SAFE_OP"},
};

struct slave_configuration_t
{
  const char * name;
  slave_op_mode mode;
  const uint16_t aliasaddr;
  uint16_t io_idx;

  // TODO: Some container for configuration stuff
};

template <class T>
struct setup_value
{
  uint16_t index;
  uint16_t sub_index;
  T value;
};

template <class... Ts>
using var_t = std::variant<setup_value<Ts>...>;
using variant_types = var_t<uint8_t, uint16_t, uint32_t, int8_t, int16_t, int32_t>;
template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

class master_node
{
public:
  explicit master_node(const std::string & ifname);

  bool setup_slave();
  bool async_spin();
  bool is_terminating() const;
  void stop();

  std::map<uint16_t, int32_t> read();
  void write(const std::map<uint16_t, int32_t> & vars);

private:
  master_node() = delete;
  master_node(const master_node &) = delete;
  master_node & operator=(const master_node &) = delete;
  master_node & operator=(master_node &) = delete;

  // Main ecat function
  OSAL_THREAD_FUNC _ecat_thread_function();
  /* Background thread checker function */
  OSAL_THREAD_FUNC _error_thread_function();

  /* Initialization main thread */
  void _initialize();

  /* Move a slave to specific operation mode */
  bool _set_slave_op_mode(uint16_t slave, slave_op_mode mode);
  /* Move a slave to specific operation state */
  bool _set_slave_op_state(uint16_t slave, slave_op_state state);
  /* Invoke a specific command on the slave controlword */
  bool _write_slave_control(uint16_t slave, slave_op_command);
  /* Return the current status / OP state for a slave */
  slave_op_state _get_slave_status(uint16_t slave);

private:
  std::vector<slave_configuration_t> _slave_configurations;

  inline static const int _io_map_size = 4096;
  char _io_map[_io_map_size];

  int _cycle_period_us{2000};
  std::string _interface_name{""};

  std::atomic_bool _is_terminating{false};
  std::atomic_bool _is_initialized{false};
  OSAL_THREAD_HANDLE _ecat_thread{nullptr};
  OSAL_THREAD_HANDLE _error_thread{nullptr};

  inline static std::atomic_bool _in_operation{false};
  inline static int _expected_working_counter{0};
  inline static volatile int _working_counter{0};
};

}  // namespace soem_impl

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_SOEM_IMPL_HPP_ */
