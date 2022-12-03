/*
 * soem_impl.cpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#include "ecat_server/soem_impl.hpp"

#include <string.h>

#include <bitset>
#include <chrono>
#include <map>
#include <thread>
#include <variant>

#include "rclcpp/rclcpp.hpp"

namespace soem_impl
{
master_node::master_node(const std::string & ifname) : _interface_name(ifname)
{
  // TODO: Check that the interface given is valid
}

bool master_node::setup_slave() { return false; }

bool master_node::async_spin()
{
  /* Thread to handle errors while in OP mode */
  osal_thread_create(
    &_error_thread, 128000, (void *)&master_node::_error_thread_function, (void *)ctime);
  /* Main EC thread */
  osal_thread_create(
    &_ecat_thread, 128000, (void *)&master_node::_ecat_thread_function, (void *)ctime);

  /* Waiting for EtherCAT to be fully operational */
  auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (timeout > std::chrono::steady_clock::now() && !_is_initialized) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return _is_initialized;
}

bool master_node::is_terminating() const { return _is_terminating; }
void master_node::stop() { _is_terminating = true; }

std::map<uint16_t, int32_t> master_node::read()
{
  std::map<uint16_t, int32_t> ret;

  /* Lock and copy the IO map buffer and read values from that */
  //  char local_buf[_io_map_size];
  //  {
  //    std::lock_guard lock{_buf_mtx};
  //    memcpy(local_buf, _io_map_buf, _io_map_size);
  //  }

  for (const slave_configuration_t & conf : _slave_configurations) {
    if (conf.io_idx == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SOEM"), "Unable to read from slave %s! Saved IO index is zero!",
        conf.name);
      continue;
    }

    switch (conf.mode) {
      case slave_op_mode::CYCLIC_POSITION: {
        auto inptr = (cyclic_position_input_t *)ec_slave[conf.io_idx].inputs;
        ret[conf.io_idx] = inptr->current_position;
        //      RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Read position %d from slave %s",
        //          ret[slave.first],
        //          slave_node_name_str.at(slave.first));
      } break;

      default:
        RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Unhandled case in master_node::read()!");
    }
  }

  return ret;
}

void master_node::write(const std::map<uint16_t, int32_t> & vars)
{
  for (const slave_configuration_t & conf : _slave_configurations) {
    if (conf.io_idx == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SOEM"), "Unable to write to slave %s! Saved IO index is zero!",
        conf.name);
      continue;
    }

    switch (conf.mode) {
      case slave_op_mode::CYCLIC_POSITION: {
        // TODO: Not sure if this access is thread safe, probably isn't so maybe lock the mutex?
        auto outptr = (cyclic_position_output_t *)ec_slave[conf.io_idx].outputs;
        outptr->target_position = vars.at(conf.io_idx);
        //      RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Wrote position %d to slave %s",
        //          vars.at(slave.first),
        //          slave_node_name_str.at(slave.first));
      } break;

      default:
        RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Unhandled case in master_node::write(...)!");
    }
  }
}

void master_node::_ecat_thread_function()
{
  _initialize();
  if (!_is_initialized) return;

  while (!_is_terminating) {
    ec_send_processdata();
    _working_counter = ec_receive_processdata(EC_TIMEOUTRET);

    if (_working_counter >= _expected_working_counter) {
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Working counter does not match expectation!");
    }

    osal_usleep(_cycle_period_us);
  }
  _in_operation = false;
  _is_terminating = true;

  for (int i{1}; i <= ec_slavecount; i++) {
    if (!_set_slave_op_state(i, slave_op_state::READY_TO_SWITCH_ON)) {
      RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Failed bringing all slaves to READY_TO_SWITCH_ON!");
    }
  }

  /* Request SAFE_OP state for all slaves */
  ec_slave[0].state = EC_STATE_SAFE_OP;
  /* Send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Attempting to bring all slaves to EC_STATE_SAFE_OP");
  int chk{200};
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_SAFE_OP, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_SAFE_OP));
  if (ec_slave[0].state != EC_STATE_SAFE_OP) {
    RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Failed bringing all slaves to EC_STATE_SAFE_OP!");
  }

  return;
}

void master_node::_initialize()
{
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Initializing the EtherCAT network ..");
  /* Initialize the interface */
  if (!ec_init(_interface_name.c_str())) {
    RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Failed initialization");
    stop();
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Initialized the interface");

  /* Initialize the configuration */
  if (ec_config_init(FALSE) <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Failed configuration initialization!");
    stop();
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Initialized the configuration");

  /* Checking alias addresses and running PDO setup */
  for (uint16_t i{1}; i <= ec_slavecount; i++) {
    RCLCPP_INFO(
      rclcpp::get_logger("SOEM"), "Slave %d has name '%s', address '0x%02x'", i, ec_slave[i].name,
      ec_slave[i].aliasadr);

    /* Check for alias */
    auto alias_predicate = [&](const slave_configuration_t & conf) {
      return conf.aliasaddr == ec_slave[i].aliasadr;
    };
    auto it = std::find_if(
      std::begin(_slave_configurations), std::end(_slave_configurations), alias_predicate);

    if (it == std::end(_slave_configurations)) {
      RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "A slave with unknown alias ID connected!");
      stop();
      return;
    } else { /* Else save slave index */
      it->io_idx = i;
    }

    /* Check for a setup function */
    if (strcmp(ec_slave[i].name, "AZD-KED") == 0) {
      RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Running AZD-KED setup on slave %d", i);
      //      bool setup_success = azd_ked::AZD_KED_setup(i);
      bool setup_success = false;
      if (!setup_success) {
        RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Failed setting up PDO mailboxes for slave %d!", i);
        stop();
        return;
      }
    } else /* Slave with unknown name connected */
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SOEM"), "Slave with unknown/unhandled name connected: %s",
        ec_slave[i].name);
      stop();
      return;
    }
  }

  /* Configure the IO map */
  int map_size = ec_config_map(&_io_map);
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Allocated %d bytes for IO map", map_size);

  /* Configure the distributed clock */
  bool dc_config = ec_configdc();
  RCLCPP_INFO(
    rclcpp::get_logger("SOEM"), "Configured distributed clock, slaves %s accept DC",
    dc_config ? "do" : "don't");

  /* Request SAFE_OP state for all slaves */
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  _expected_working_counter = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  RCLCPP_INFO(
    rclcpp::get_logger("SOEM"), "Expected working counter calculated as: %d",
    _expected_working_counter);

  /* Request OP state for all slaves */
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  /* Send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Attempting to bring all slaves to EC_STATE_OPERATIONAL");

  int chk{200};
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
  if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Failed bringing all slaves to EC_STATE_OPERATIONAL!");
    stop();
    return;
  }

  //  ec_recover_slave(1, EC_TIMEOUTSAFE);
  //  RCLCPP_WARN(
  //    rclcpp::get_logger("SOEM"), "Chk: %d, Slave 1 is in state %s", chk,
  //    ec_state2string.at(ec_slave[1].state));

  _in_operation = true;  // Starting the EC background thread
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "All slaves reached EC_STATE_OPERATIONAL");

  /* Zero out slaves */
  for (int i{1}; i <= ec_slavecount; i++) {
    auto alias_predicate = [&](const slave_configuration_t & conf) {
      return conf.aliasaddr == ec_slave[i].aliasadr;
    };
    auto it = std::find_if(
      std::begin(_slave_configurations), std::end(_slave_configurations), alias_predicate);
    if (it->mode == slave_op_mode::CYCLIC_POSITION) {
      auto inptr = (cyclic_position_input_t *)ec_slave[i].inputs;
      auto outptr = (cyclic_position_output_t *)ec_slave[i].outputs;
      outptr->target_position = inptr->current_position;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("SOEM"),
        "Slave configuration error! No matching slave operational mode!");
      stop();
      return;
    }
  }

  /* Move slaves to drive OP mode */
  for (uint16_t i{1}; i <= ec_slavecount; i++) {
    auto alias_predicate = [&](const slave_configuration_t & conf) {
      return conf.aliasaddr == ec_slave[i].aliasadr;
    };
    auto it = std::find_if(
      std::begin(_slave_configurations), std::end(_slave_configurations), alias_predicate);
    if (!_set_slave_op_mode(i, it->mode)) {
      /* Failed setting slave op mode */
      RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Failed setting slave %d to its OP mode", i);
      stop();
      return;
    }
  }

  /* Move slaves to drive OP state */
  for (uint16_t i{1}; i <= ec_slavecount; i++) {
    if (!_set_slave_op_state(i, slave_op_state::OPERATION_ENABLED)) {
      /* Failed setting slave op state */
      RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Failed setting slave %d to OPERATION_ENABLED", i);
      stop();
      return;
    }
  }

  _is_initialized = true;  // Settings the flag for successful initialization
  RCLCPP_INFO(rclcpp::get_logger("SOEM"), "EtherCAT network initialization complete");
  return;  // End of EtherCAT network initialization
}

OSAL_THREAD_FUNC master_node::_error_thread_function()
{
  int slave;

  while (1) {
    RCLCPP_WARN(
      rclcpp::get_logger("SOEM CHECKER"), "In OP: %d WKC: %d, eWKC: %d, Master Check: %d",
      _in_operation.load(), _working_counter, _expected_working_counter, ec_group[0].docheckstate);
    if (
      _in_operation &&
      ((_working_counter < _expected_working_counter) || ec_group[0].docheckstate)) {
      /* one ore more slaves are not responding */
      ec_group[0].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[0].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            RCLCPP_WARN(
              rclcpp::get_logger("SOEM CHECKER"), "Slave %d is in SAFE_OP + ERROR, attempting ACK",
              slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            RCLCPP_WARN(
              rclcpp::get_logger("SOEM CHECKER"), "Slave %d is in SAFE_OP, change to OPERATIONAL",
              slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              RCLCPP_WARN(rclcpp::get_logger("SOEM CHECKER"), "Slave %d reconfigured", slave);
            }
          } else if (!ec_slave[slave].islost) {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = TRUE;
              RCLCPP_WARN(rclcpp::get_logger("SOEM CHECKER"), "Slave %d lost", slave);
            }
          }
        }
        if (ec_slave[slave].islost) {
          if (ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              RCLCPP_WARN(rclcpp::get_logger("SOEM CHECKER"), "Slave %d recovered", slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            RCLCPP_WARN(rclcpp::get_logger("SOEM CHECKER"), "Slave %d found", slave);
          }
        }
      }
      if (!ec_group[0].docheckstate)
        RCLCPP_INFO(rclcpp::get_logger("SOEM CHECKER"), "All slaves resumed OPERATIONAL");
    }
    osal_usleep(10000);
  }
}

bool master_node::_set_slave_op_mode(uint16_t slave, slave_op_mode mode)
{
  auto io_idx_predicate = [&](const slave_configuration_t & conf) { return conf.io_idx == slave; };
  auto it = std::find_if(
    std::begin(_slave_configurations), std::end(_slave_configurations), io_idx_predicate);
  int timeout{20};

  do {
    if (it->mode == slave_op_mode::CYCLIC_POSITION) {
      auto outptr = (cyclic_position_output_t *)ec_slave[slave].outputs;
      if (outptr->operationmode == static_cast<int8_t>(mode))
        return true;
      else
        outptr->operationmode = static_cast<int8_t>(mode);
    } else {
      return false;  // TODO: Implement other modes
    }

    ec_send_processdata();
    _working_counter = ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);

  } while (timeout--);
  return false;  // Should return elsewhere
}

bool master_node::_set_slave_op_state(uint16_t slave, slave_op_state state)
{
  /* Get latest status */
  slave_op_state current_status = _get_slave_status(slave);

  RCLCPP_INFO(
    rclcpp::get_logger("SOEM"), "Transitioning slave %d from state %s to state %s", slave,
    slave_op_status_str.at(current_status), slave_op_status_str.at(state));

  /* Maximum times round the loop */
  int cnt{100};

  while (cnt--) {
    current_status = _get_slave_status(slave);

    if (current_status == state) return true;

    switch (current_status) {
      /* Cases we cannot handle */
      case slave_op_state::FAULT_REACTION_ACTIVE:
      case slave_op_state::NOT_READY_TO_SWITCH_ON:
      case slave_op_state::INVALID:
      case slave_op_state::MASK:
        RCLCPP_ERROR(
          rclcpp::get_logger("SOEM"),
          "Error while changing slave %d state: Current status '%s' is either invalid or "
          "un-handleable at this moment",
          slave, slave_op_status_str.at(current_status));
        return false;
      /* Fault reset is always this */
      case slave_op_state::FAULT:
        RCLCPP_WARN(
          rclcpp::get_logger("SOEM"), "Slave %d is in %s state, writing a FAULT_RESET!", slave,
          slave_op_status_str.at(current_status));
        _write_slave_control(slave, slave_op_command::FAULT_RESET);
        break;
    }

    switch (state) {
      case slave_op_state::SWITCH_ON_DISABLED:
        switch (current_status) {
          case slave_op_state::READY_TO_SWITCH_ON:  // Transition 7
          case slave_op_state::SWITCHED_ON:         // Transition 10
          case slave_op_state::QUICK_STOP_ACTIVE:   // Transition 12
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command DISABLE_VOLTAGE",
              slave, slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::DISABLE_VOLTAGE);
            break;
          case slave_op_state::OPERATION_ENABLED:  // Transition 11 + 12 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command QUICKSTOP", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::QUICK_STOP);
            break;
          default:  // Not meant to happen
            RCLCPP_ERROR(
              rclcpp::get_logger("SOEM"),
              "Error while changing slave state: Current status unhandled during "
              "SWITCH_ON_DISABLED transition!");
            return false;
        }
        break;

      case slave_op_state::READY_TO_SWITCH_ON:
        switch (current_status) {
          case slave_op_state::SWITCH_ON_DISABLED:  // Transition 2
          case slave_op_state::SWITCHED_ON:         // Transition 6
          case slave_op_state::OPERATION_ENABLED:   // Transition 8
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command SHUTDOWN", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::SHUTDOWN);
            break;
          case slave_op_state::QUICK_STOP_ACTIVE:  // Transition 12 + 2 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command DISABLE_VOLTAGE",
              slave, slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::DISABLE_VOLTAGE);
            break;
          default:  // Not meant to happen
            RCLCPP_ERROR(
              rclcpp::get_logger("SOEM"),
              "Error while changing slave state: Current status unhandled during "
              "READY_TO_SWITCH_ON transition!");
            return false;
        }
        break;

      case slave_op_state::SWITCHED_ON:
        switch (current_status) {
          case slave_op_state::SWITCH_ON_DISABLED:  // Transition 2 + 3 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command SHUTDOWN", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::SHUTDOWN);
            break;
          case slave_op_state::READY_TO_SWITCH_ON:  // Transition 3
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command SWITCH_ON", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::SWITCH_ON);
            break;
          case slave_op_state::OPERATION_ENABLED:  // Transition 5
            _write_slave_control(slave, slave_op_command::DISABLE_OPERATION);
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command DISABLE_OPERATION",
              slave, slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            break;
          case slave_op_state::QUICK_STOP_ACTIVE:  // Transition 12 + 2 + 3 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command DISABLE_VOLTAGE",
              slave, slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::DISABLE_VOLTAGE);
            break;
          default:  // Not meant to happen
            RCLCPP_ERROR(
              rclcpp::get_logger("SOEM"),
              "Error while changing slave state: Current status unhandled during SWITCHED_ON "
              "transition!");
            return false;
        }
        break;

      case slave_op_state::OPERATION_ENABLED:
        switch (current_status) {
          case slave_op_state::SWITCH_ON_DISABLED:  // Transition 2 + 3 + 4 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command SHUTDOWN", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::SHUTDOWN);
            break;
          case slave_op_state::READY_TO_SWITCH_ON:  // Transition 3 + 4
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"),
              "Slave %d: From %s to %s via command SWITCH_ON_AND_ENABLE_OPERATION", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::SWITCH_ON_AND_ENABLE_OPERATION);
            break;
          case slave_op_state::SWITCHED_ON:        // Transition 4
          case slave_op_state::QUICK_STOP_ACTIVE:  // Transition 16
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command ENABLE_OPERATION",
              slave, slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::ENABLE_OPERATION);
            break;
          default:  // Not meant to happen
            RCLCPP_ERROR(
              rclcpp::get_logger("SOEM"),
              "Error while changing slave state: Current status unhandled during OPERATION_ENABLED "
              "transition!");
            return false;
        }
        break;

      case slave_op_state::QUICK_STOP_ACTIVE:
        switch (current_status) {
          case slave_op_state::OPERATION_ENABLED:  // Transition 11
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command QUICK_STOP", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::QUICK_STOP);
            break;
          case slave_op_state::READY_TO_SWITCH_ON:  // Transition 3 + 4 + 11 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"),
              "Slave %d: From %s to %s via command SWITCH_ON_AND_ENABLE_OPERATION", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::SWITCH_ON_AND_ENABLE_OPERATION);
            break;
          case slave_op_state::SWITCHED_ON:  // Transition 4 + 11 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command ENABLE_OPERATION",
              slave, slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::ENABLE_OPERATION);
            break;
          case slave_op_state::SWITCH_ON_DISABLED:  // Transition 2 + 3 + 4 + 11 later
            RCLCPP_INFO(
              rclcpp::get_logger("SOEM"), "Slave %d: From %s to %s via command SHUTDOWN", slave,
              slave_op_status_str.at(current_status), slave_op_status_str.at(state));
            _write_slave_control(slave, slave_op_command::SHUTDOWN);
            break;
          default:  // Not meant to happen
            RCLCPP_ERROR(
              rclcpp::get_logger("SOEM"),
              "Error while changing slave state: Current status unhandled during QUICK_STOP "
              "transition!");
            return false;
        }
        break;

      case slave_op_state::INVALID:
      case slave_op_state::MASK:
      case slave_op_state::FAULT_REACTION_ACTIVE:
      case slave_op_state::FAULT:
        RCLCPP_ERROR(
          rclcpp::get_logger("SOEM"),
          "Error while changing slave state: Requested state is not a valid target state");
        return false;  // Who asked for this :(
    }

    // Update network and current status
    ec_send_processdata();
    _working_counter = ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);
  }

  // Reaching here means success if the counter is not zero
  if (cnt <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SOEM"), "Exceed timeout counter for switching slave state!");
    return false;
  } else {
    return true;
  }
}

bool master_node::_write_slave_control(uint16_t slave, slave_op_command ctrl)
{
  auto io_idx_predicate = [&](const slave_configuration_t & conf) { return conf.io_idx == slave; };
  auto it = std::find_if(
    std::begin(_slave_configurations), std::end(_slave_configurations), io_idx_predicate);

  controlword_t current_control;
  if (it->mode == slave_op_mode::CYCLIC_POSITION) {
    auto outptr = (cyclic_position_output_t *)ec_slave[slave].outputs;
    current_control = outptr->controlword;
  } else {
    // TODO: Other modes
  }

  /* Get latest status */
  slave_op_state current_status = _get_slave_status(slave);

  /* Do some work on the controlword */
  switch (ctrl) {
    case slave_op_command::DISABLE_OPERATION:
      if (current_status != slave_op_state::OPERATION_ENABLED)
        return false;  // Invalid starting status
      current_control.switch_on = true;
      current_control.enable_voltage = true;
      current_control.quick_stop = true;
      current_control.enable_operation = false;
      RCLCPP_INFO(
        rclcpp::get_logger("SOEM"), "Writing to slave %d: Command DISABLE_OPERATION", slave);
      break;

    case slave_op_command::DISABLE_VOLTAGE:
      if (
        current_status != slave_op_state::READY_TO_SWITCH_ON &&
        current_status != slave_op_state::OPERATION_ENABLED &&
        current_status != slave_op_state::SWITCHED_ON &&
        current_status != slave_op_state::QUICK_STOP_ACTIVE)
        return false;  // Invalid starting status
      current_control.enable_voltage = false;
      RCLCPP_INFO(
        rclcpp::get_logger("SOEM"), "Writing to slave %d: Command DISABLE_VOLTAGE", slave);
      break;

    case slave_op_command::ENABLE_OPERATION:
      if (
        current_status != slave_op_state::SWITCHED_ON &&
        current_status != slave_op_state::QUICK_STOP_ACTIVE)
        return false;  // Invalid starting status
      current_control.switch_on = true;
      current_control.enable_voltage = true;
      current_control.quick_stop = true;
      current_control.enable_operation = true;
      RCLCPP_INFO(
        rclcpp::get_logger("SOEM"), "Writing to slave %d: Command ENABLE_OPERATION", slave);
      break;

    case slave_op_command::FAULT_RESET:
      /* Brute force clear this for now */
      // TODO: Log any information available about this error
      current_control.fault_reset = false;
      if (it->mode == slave_op_mode::CYCLIC_POSITION) {
        auto outptr = (cyclic_position_output_t *)ec_slave[slave].outputs;
        outptr->controlword = current_control;
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
      }
      current_control.fault_reset = true;
      if (it->mode == slave_op_mode::CYCLIC_POSITION) {
        auto outptr = (cyclic_position_output_t *)ec_slave[slave].outputs;
        outptr->controlword = current_control;
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
      }

      break;

    case slave_op_command::QUICK_STOP:
      if (
        current_status != slave_op_state::READY_TO_SWITCH_ON &&
        current_status != slave_op_state::SWITCHED_ON &&
        current_status != slave_op_state::OPERATION_ENABLED)
        return false;  // Invalid starting status
      current_control.enable_voltage = true;
      current_control.quick_stop = false;  // It is active low
      RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Writing to slave %d: Command QUICK_STOP", slave);
      break;

    case slave_op_command::SHUTDOWN:
      if (
        current_status != slave_op_state::SWITCH_ON_DISABLED &&
        current_status != slave_op_state::SWITCHED_ON &&
        current_status != slave_op_state::OPERATION_ENABLED)
        return false;  // Invalid starting status
      current_control.switch_on = false;
      current_control.enable_voltage = true;
      current_control.quick_stop = true;
      RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Writing to slave %d: Command SHUTDOWN", slave);
      break;

    case slave_op_command::SWITCH_ON:
      if (current_status != slave_op_state::READY_TO_SWITCH_ON)
        return false;  // Invalid starting status
      current_control.switch_on = true;
      current_control.enable_voltage = true;
      current_control.quick_stop = true;
      current_control.enable_operation = false;
      RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Writing to slave %d: Command SWITCH_ON", slave);
      break;

    case slave_op_command::SWITCH_ON_AND_ENABLE_OPERATION:
      if (current_status != slave_op_state::READY_TO_SWITCH_ON)
        return false;  // Invalid starting status
      current_control.switch_on = true;
      current_control.enable_voltage = true;
      current_control.quick_stop = true;
      current_control.enable_operation = true;
      RCLCPP_INFO(
        rclcpp::get_logger("SOEM"), "Writing to slave %d: Command SWITCH_ON_AND_ENABLE_OPERATION",
        slave);
      break;

    default:
      return false;  // Unthinkable error
  }

  /* Write the new value to the IO map */
  if (it->mode == slave_op_mode::CYCLIC_POSITION) {
    auto outptr = (cyclic_position_output_t *)ec_slave[slave].outputs;
    outptr->controlword = current_control;
  } else {
    // TODO: Add other modes
  }

  return true;
}

slave_op_state master_node::_get_slave_status(uint16_t slave)
{
  auto io_idx_predicate = [&](const slave_configuration_t & conf) { return conf.io_idx == slave; };
  auto it = std::find_if(
    std::begin(_slave_configurations), std::end(_slave_configurations), io_idx_predicate);
  statusword_t current_status;

  if (it->mode == slave_op_mode::CYCLIC_POSITION) {
    auto inptr = (cyclic_position_input_t *)ec_slave[slave].inputs;
    current_status = inptr->statusword;
  } else {
    // TODO: Add other operation modes
    RCLCPP_ERROR(
      rclcpp::get_logger("SOEM"), "Slave %d in unsupported mode for getting state!", slave);
    return slave_op_state::INVALID;
  }

  // Mask and return
  current_status.set(current_status.uint16() & static_cast<uint16_t>(slave_op_state::MASK));

  try {
    RCLCPP_INFO(
      rclcpp::get_logger("SOEM"), "Slave %d is in %s state", slave,
      slave_op_status_str.at(static_cast<slave_op_state>(current_status.uint16())));
  } catch (...) {
    RCLCPP_INFO(rclcpp::get_logger("SOEM"), "Slave %d is in an un-stringifyable state!", slave);
  }

  switch (static_cast<slave_op_state>(current_status.uint16())) {
    case slave_op_state::NOT_READY_TO_SWITCH_ON:
      return slave_op_state::NOT_READY_TO_SWITCH_ON;

    case slave_op_state::FAULT:
      return slave_op_state::FAULT;

    case slave_op_state::FAULT_REACTION_ACTIVE:
      return slave_op_state::FAULT_REACTION_ACTIVE;

    case slave_op_state::SWITCH_ON_DISABLED:
      return slave_op_state::SWITCH_ON_DISABLED;

    case slave_op_state::READY_TO_SWITCH_ON:
      return slave_op_state::READY_TO_SWITCH_ON;

    case slave_op_state::SWITCHED_ON:
      return slave_op_state::SWITCHED_ON;

    case slave_op_state::OPERATION_ENABLED:
      return slave_op_state::OPERATION_ENABLED;

    case slave_op_state::QUICK_STOP_ACTIVE:
      return slave_op_state::QUICK_STOP_ACTIVE;

    case slave_op_state::MASK:
    case slave_op_state::INVALID:
    default:
      return slave_op_state::INVALID;  // No matching state
  }
}

}  // namespace soem_impl
