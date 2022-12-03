/*
 * rtelligent_ect60.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: sr
 */

#include "ecat_server/rtelligent_ect60.hpp"

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace soem_impl::rtelligent_ect60
{
bool ECT60_setup(uint16_t slave)
{
  int retval{0};
  int cnt_default{20}, cnt{cnt_default};
  uint8_t temp_u8;

  uint16_t SM2_ADDR = 0x1c12;
  uint16_t RxPDO1_ADDR = 0x1600;
  uint16_t SM3_ADDR = 0x1c13;
  uint16_t TxPDO1_ADDR = 0x1a00;

  /*
     * Controlword
     * Target Position
     * Modes of Operation
     */
  uint8_t RxPDO_ENTRIES_NUM = 0x03;
  uint32_t RxPDO_ENTRIES[RxPDO_ENTRIES_NUM] = {0x60400010, 0x607a0020, 0x60600008};

  /*
     * Statusword
     * Actual Position
     * Display Mode of Operation
     */
  uint8_t TxPDO_ENTRIES_NUM = 0x03;
  uint32_t TxPDO_ENTRIES[TxPDO_ENTRIES_NUM] = {0x60410010, 0x60640020, 0x60610008};

  /* RxPDO setup */
  // Set number of entries for sync manager 2 to zero
  temp_u8 = 0x00;
  do {
    retval = ec_SDOwrite(slave, SM2_ADDR, 0x00, false, sizeof(temp_u8), &temp_u8, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for Sync Manager 2", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote 0 to Number of Entries for Sync Manager 2",
      slave);
  }
  retval = 0;
  cnt = cnt_default;

  // Set number of entries for RxPDO objects to zero
  temp_u8 = 0x00;
  do {
    retval =
      ec_SDOwrite(slave, RxPDO1_ADDR, 0x00, false, sizeof(temp_u8), &temp_u8, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for RxPDO1", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote 0 to Number of Entries for RxPDO1", slave);
  }
  retval = 0;
  cnt = cnt_default;

  // Change RxPDO mapping objects
  for (int i{0}; i < RxPDO_ENTRIES_NUM; i++) {
    do {
      retval = ec_SDOwrite(
        slave, RxPDO1_ADDR, i + 1, false, sizeof(RxPDO_ENTRIES[i]), &RxPDO_ENTRIES[i],
        EC_TIMEOUTSAFE);
    } while (cnt-- && !retval);
    if (cnt <= 0 || retval <= 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ECT60"), "Slave %d timeout: Failed SDO write to RxPDO1 mapping %d",
        slave, i);
      return false;
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("ECT60"), "Slave %d: Wrote object entry %d to RxPDO1", slave, i);
    }
    retval = 0;
    cnt = cnt_default;
  }

  // Change RxPDO number of entries
  do {
    retval = ec_SDOwrite(
      slave, RxPDO1_ADDR, 0x00, false, sizeof(RxPDO_ENTRIES_NUM), &RxPDO_ENTRIES_NUM,
      EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for RxPDO1", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote 4 to Number of Entries for RxPDO1", slave);
  }
  retval = 0;
  cnt = cnt_default;

  // Change SM2 object
  do {
    retval =
      ec_SDOwrite(slave, SM2_ADDR, 0x01, false, sizeof(RxPDO1_ADDR), &RxPDO1_ADDR, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to RxPDO entry for Sync Manager 2", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote RxPDO entry to Sync Manager 2", slave);
  }
  retval = 0;
  cnt = cnt_default;

  // Change SM2 number of entries
  temp_u8 = 0x01;
  do {
    retval = ec_SDOwrite(slave, SM2_ADDR, 0x00, false, sizeof(temp_u8), &temp_u8, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for Sync Manager 2", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote 1 to Number of Entries for Sync Manager 2",
      slave);
  }
  retval = 0;
  cnt = cnt_default;

  /* TxPDO setup */
  // Set number of entries for sync manager 3 to zero
  temp_u8 = 0x00;
  do {
    retval = ec_SDOwrite(slave, SM3_ADDR, 0x00, false, sizeof(temp_u8), &temp_u8, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for Sync Manager 3", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote 0 to Number of Entries for Sync Manager 3",
      slave);
  }
  retval = 0;
  cnt = cnt_default;

  // Set number of entries for TxPDO objects to zero
  temp_u8 = 0x00;
  do {
    retval =
      ec_SDOwrite(slave, TxPDO1_ADDR, 0x00, false, sizeof(temp_u8), &temp_u8, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for TxPDO1", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote 0 to Number of Entries for TxPDO1", slave);
  }
  retval = 0;
  cnt = cnt_default;

  // Change TxPDO mapping objects
  for (int i{0}; i < TxPDO_ENTRIES_NUM; i++) {
    do {
      retval = ec_SDOwrite(
        slave, TxPDO1_ADDR, i + 1, false, sizeof(TxPDO_ENTRIES[i]), &TxPDO_ENTRIES[i],
        EC_TIMEOUTSAFE);
    } while (cnt-- && !retval);
    if (cnt <= 0 || retval <= 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ECT60"), "Slave %d timeout: Failed SDO write to TxPDO1 mapping %d",
        slave, i);
      return false;
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("ECT60"), "Slave %d: Wrote object entry %d to TxPDO1", slave, i);
    }
    retval = 0;
    cnt = cnt_default;
  }

  // Change TxPDO mapping number of entries
  do {
    retval = ec_SDOwrite(
      slave, TxPDO1_ADDR, 0x00, false, sizeof(TxPDO_ENTRIES_NUM), &TxPDO_ENTRIES_NUM,
      EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for TxPDO1", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote %d to Number of Entries for TxPDO1", slave,
      TxPDO_ENTRIES_NUM);
  }
  retval = 0;
  cnt = cnt_default;

  // Change SM3 object
  do {
    retval =
      ec_SDOwrite(slave, SM3_ADDR, 0x01, false, sizeof(TxPDO1_ADDR), &TxPDO1_ADDR, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to TxPDO entry for Sync Manager 3", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote TxPDO entry to Sync Manager 3", slave);
  }
  retval = 0;
  cnt = cnt_default;

  // Change SM3 number of entries
  temp_u8 = 0x01;
  do {
    retval = ec_SDOwrite(slave, SM3_ADDR, 0x00, false, sizeof(temp_u8), &temp_u8, EC_TIMEOUTSAFE);
  } while (cnt-- && !retval);
  if (cnt <= 0 || retval <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ECT60"),
      "Slave %d timeout: Failed SDO write to Number of Entries for Sync Manager 3", slave);
    return false;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ECT60"), "Slave %d: Wrote 1 to Number of Entries for Sync Manager 3",
      slave);
  }
  retval = 0;
  cnt = cnt_default;

  /* AFTER HERE IS REAL SETUP */

  for (const auto & val : setup_values) {
    do {
      std::visit(
        soem_impl::overloaded{
          [&](setup_value<uint8_t> & arg) {
            retval = ec_SDOwrite(
              slave, arg.index, arg.sub_index, false, sizeof(arg.value), &arg.value,
              EC_TIMEOUTSAFE);
          },
          [&](setup_value<uint16_t> & arg) {
            retval = ec_SDOwrite(
              slave, arg.index, arg.sub_index, false, sizeof(arg.value), &arg.value,
              EC_TIMEOUTSAFE);
          },
          [&](setup_value<uint32_t> & arg) {
            retval = ec_SDOwrite(
              slave, arg.index, arg.sub_index, false, sizeof(arg.value), &arg.value,
              EC_TIMEOUTSAFE);
          },
          [](auto & arg) {
            (void)arg; /*Nothing*/
          }},
        val);
    } while (cnt-- && !retval);
  }

  RCLCPP_INFO(rclcpp::get_logger("ECT60"), "Slave %d setup completed", slave);
  return true;
}

}  // namespace soem_impl::rtelligent_ect60

//#include <algorithm>
//#include <iostream>
//#include <variant>
//
//#include "ecat_server/generic_od.hpp"
//#include "ecat_server/soem_impl.hpp"
//
//using soem_impl::od_obj;
//using soem_impl::OD_OBJ_CAT;
//using soem_impl::od_obj_ent;
//using soem_impl::od_obj_ent_t;
//using soem_impl::OD_OBJ_PDO;
//using soem_impl::OD_OBJ_PERM;
//using soem_impl::od_obj_t;
//
//std::vector<od_obj_t> soem_impl::ect60::od = {};
//
//void soem_impl::ect60::init_od()
//{
//  using std::nullopt;
//  od.clear();
//
//  ///////////////////////////
//  /* STANDARD CIA-301 AREA */
//  ///////////////////////////
//  // 0x1000 - Device Type
//  auto idx_0x1000 = od_obj<uint32_t>(0x1000, "Device Type", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x1000_00 = od_obj_ent<uint32_t>(
//    0, nullopt, nullopt, 0, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Device Type");
//  idx_0x1000.add_entry(idx_0x1000_00);
//  od.push_back(idx_0x1000);
//
//  // 0x1001 - Error Register
//  auto idx_0x1001 = od_obj<const char *>(0x1001, "Error register", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x1001_00 = od_obj_ent<const char *>(
//    0, nullopt, nullopt, "", OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Error Register");
//  idx_0x1001.add_entry(idx_0x1001_00);
//  od.push_back(idx_0x1001);
//
//  // 0x1009 - Hardware Version
//  auto idx_0x1009 = od_obj<const char *>(0x1009, "Hardware Version", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x1009_00 = od_obj_ent<const char *>(
//    0, nullopt, nullopt, "", OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Hardware Version");
//  idx_0x1009.add_entry(idx_0x1009_00);
//  od.push_back(idx_0x1009);
//
//  // 0x100A - Software Version
//  auto idx_0x100A = od_obj<const char *>(0x100A, "Software Version", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x100A_00 = od_obj_ent<const char *>(
//    0, nullopt, nullopt, "", OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Software Version");
//  idx_0x100A.add_entry(idx_0x100A_00);
//  od.push_back(idx_0x100A);
//
//  // 0x1010 - Save Parameters
//  //	00	- Maximum number of sub-indexes
//  //	01	- Save Parameters
//  auto idx_0x1010 = od_obj<uint8_t>(0x1010, "Save Parameters", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x1010_00 = od_obj_ent<uint8_t>(
//    0, nullopt, nullopt, 1, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Maximum Number of Sub-Indexes");
//  auto idx_0x1010_01 = od_obj_ent<uint8_t>(
//    1, 0, 1, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Save Parameters");
//  idx_0x1010.add_entry(idx_0x1010_00);
//  idx_0x1010.add_entry(idx_0x1010_01);
//  od.push_back(idx_0x1010);
//
//  // 0x1011 - Restore Factory Settings
//  //	00	- Maximum number of sub-indexes
//  //	01	- Restore Parameters
//  auto idx_0x1011 = od_obj<uint8_t>(0x1011, "Restore Parameters", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x1011_00 = od_obj_ent<uint8_t>(
//    0, nullopt, nullopt, 1, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Maximum Number of Sub-Indexes");
//  auto idx_0x1011_01 = od_obj_ent<uint8_t>(
//    1, 0, 1, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Restore Parameters");
//  idx_0x1011.add_entry(idx_0x1011_00);
//  idx_0x1011.add_entry(idx_0x1011_01);
//  od.push_back(idx_0x1011);
//
//  ////////////////////////////////
//  /* MANUFACTURER SPECIFIC AREA */
//  ////////////////////////////////
//  // 0x2000 - Operating Current [mA]
//  auto idx_0x2000 = od_obj<uint16_t>(0x2000, "Operating Current", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2000_00 = od_obj_ent<uint16_t>(
//    0, 100, 6000, 3000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Peak Current [mA]");
//  idx_0x2000.add_entry(idx_0x2000_00);
//  od.push_back(idx_0x2000);
//
//  // 0x2001	- Open Loop Pulses Per Rotation
//  auto idx_0x2001 = od_obj<uint16_t>(0x2001, "Subdivision / Resolution", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2001_00 = od_obj_ent<uint16_t>(
//    0, 200, 65535, 10000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Motor Resolution [PPR]");
//  idx_0x2001.add_entry(idx_0x2001_00);
//  od.push_back(idx_0x2001);
//
//  // 0x2002 - Standby Time
//  auto idx_0x2002 = od_obj<uint16_t>(0x2002, "Standby Time", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2002_00 = od_obj_ent<uint16_t>(
//    0, 200, 65535, 500, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Idle Time [ms]");
//  idx_0x2002.add_entry(idx_0x2002_00);
//  od.push_back(idx_0x2002);
//
//  // 0x2003 - Percentage of Standby Current
//  auto idx_0x2003 = od_obj<uint16_t>(0x2003, "Percentage of Standby Current", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2003_00 = od_obj_ent<uint16_t>(
//    0, 0, 100, 50, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Idle Current [%]");
//  idx_0x2003.add_entry(idx_0x2003_00);
//  od.push_back(idx_0x2003);
//
//  // 0x2005 - Output Port Function
//  // 	0 - Custom output
//  //	1 - Alarm output
//  //	2 - Brake output
//  //	3 - In place output
//  auto idx_0x2005 = od_obj<uint16_t>(0x2005, "Output Port Function", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2005_01 = od_obj_ent<uint16_t>(
//    1, 0, 3, 1, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Output 1 Function");
//  auto idx_0x2005_02 = od_obj_ent<uint16_t>(
//    2, 0, 3, 2, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Output 2 Function");
//  idx_0x2005.add_entry(idx_0x2005_01);
//  idx_0x2005.add_entry(idx_0x2005_02);
//  od.push_back(idx_0x2005);
//
//  // 0x2006 - Output Port Polarity
//  // 	Bit 0 - OUT1
//  //	Bit 1 - OUT2
//  //	0 = NC, 1 = NO
//  auto idx_0x2006 = od_obj<uint16_t>(0x2006, "Output Port Polarity", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2006_00 = od_obj_ent<uint16_t>(
//    0, 0, 3, 3, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Outputs Polarity");
//  idx_0x2006.add_entry(idx_0x2006_00);
//  od.push_back(idx_0x2006);
//
//  // 0x2007 - Input Port Function
//  // 	0 - General Input Port
//  //	1 - CW Limit Input
//  //	2 - CCW Limit Input
//  //	3 - HOME Input
//  // 	4 - Fault Clearance
//  //	5 - Emergency Stop Signal
//  //	6 - Motor Offline
//  //	7 - Probe 1
//  //	8 - Probe 2
//  auto idx_0x2007 = od_obj<uint16_t>(0x2007, "Input Port Function", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2007_04 = od_obj_ent<uint16_t>(
//    4, 0, 8, 1, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Input 3 Function");
//  auto idx_0x2007_05 = od_obj_ent<uint16_t>(
//    5, 0, 8, 2, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Input 4 Function");
//  auto idx_0x2007_06 = od_obj_ent<uint16_t>(
//    6, 0, 8, 3, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Input 5 Function");
//  auto idx_0x2007_07 = od_obj_ent<uint16_t>(
//    7, 0, 8, 6, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Input 6 Function");
//  idx_0x2007.add_entry(idx_0x2007_04);
//  idx_0x2007.add_entry(idx_0x2007_05);
//  idx_0x2007.add_entry(idx_0x2007_06);
//  idx_0x2007.add_entry(idx_0x2007_07);
//  od.push_back(idx_0x2007);
//
//  // 0x2008 - Input Port Polarity
//  // 	Bit 0 - IN1
//  //	Bit 1 - IN2
//  //	Bit 2 - IN3
//  //	Bit 3 - IN4
//  //	Bit 4 - IN5
//  //	Bit 5 - IN6
//  //	0 = NC, 1 = NO
//  auto idx_0x2008 = od_obj<uint16_t>(0x2008, "Input Port Polarity", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2008_00 = od_obj_ent<uint16_t>(
//    0, 0, 0x3F, 0x3F, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Inputs Polarity");
//  idx_0x2008.add_entry(idx_0x2008_00);
//  od.push_back(idx_0x2008);
//
//  // 0x2009 - Filter Time
//  auto idx_0x2009 = od_obj<uint16_t>(0x2009, "Filter Time", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2009_00 = od_obj_ent<uint16_t>(
//    0, 0, 25600, 6400, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Filter Time [us]");
//  idx_0x2009.add_entry(idx_0x2009_00);
//  od.push_back(idx_0x2009);
//
//  // 0x200A - Shaft Lock Time
//  auto idx_0x200A = od_obj<uint16_t>(0x200A, "Shaft Lock Time", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x200A_00 = od_obj_ent<uint16_t>(
//    0, 0, 65535, 1000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Shaft Lock Time [50us]");
//  idx_0x200A.add_entry(idx_0x200A_00);
//  od.push_back(idx_0x200A);
//
//  // 0x200B - Current Loop Parameters
//  auto idx_0x200B = od_obj<uint16_t>(0x200B, "Current Loop Parameters", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x200B_01 = od_obj_ent<uint16_t>(
//    1, 0, 1, 1, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Auto PI Enable");
//  auto idx_0x200B_02 = od_obj_ent<uint16_t>(
//    2, 100, 65535, 1000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Current Loop Kp");
//  auto idx_0x200B_03 = od_obj_ent<uint16_t>(
//    3, 0, 10000, 200, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Current Loop Ki");
//  auto idx_0x200B_04 = od_obj_ent<uint16_t>(
//    4, 0, 1024, 256, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Current Loop Kc");
//  idx_0x200B.add_entry(idx_0x200B_01);
//  idx_0x200B.add_entry(idx_0x200B_02);
//  idx_0x200B.add_entry(idx_0x200B_03);
//  idx_0x200B.add_entry(idx_0x200B_04);
//  od.push_back(idx_0x200B);
//
//  // 0x200C - Motor Parameters
//  auto idx_0x200C = od_obj<uint16_t>(0x200C, "Motor Parameters", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x200C_01 = od_obj_ent<uint16_t>(
//    1, 0, 1, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Motor Type");
//  auto idx_0x200C_02 = od_obj_ent<uint16_t>(
//    2, 100, 65535, 1000, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Resistance Auto [mOhm]");
//  auto idx_0x200C_03 = od_obj_ent<uint16_t>(
//    3, 0, 10, 1, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Inductance Auto [mH]");
//  auto idx_0x200C_04 = od_obj_ent<uint16_t>(
//    4, 0, 10000, 1000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Resistance Set [mOhm]");
//  auto idx_0x200C_05 = od_obj_ent<uint16_t>(
//    5, 1, 10, 1, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Inductance Set [mH]");
//  auto idx_0x200C_06 = od_obj_ent<uint16_t>(
//    6, 0, 1000, 256, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "BEMF Coefficient");
//  idx_0x200C.add_entry(idx_0x200C_01);
//  idx_0x200C.add_entry(idx_0x200C_02);
//  idx_0x200C.add_entry(idx_0x200C_03);
//  idx_0x200C.add_entry(idx_0x200C_04);
//  idx_0x200C.add_entry(idx_0x200C_05);
//  idx_0x200C.add_entry(idx_0x200C_06);
//  od.push_back(idx_0x200C);
//
//  // 0x200D - Run Reverse
//  auto idx_0x200D = od_obj<uint16_t>(0x200D, "Run Reverse", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x200D_00 = od_obj_ent<uint16_t>(
//    0, 0, 1, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Invert Motor Direction");
//  idx_0x200D.add_entry(idx_0x200D_00);
//  od.push_back(idx_0x200D);
//
//  // 0x200E - Internal Alarm Code
//  //	0x0001 - Internal Voltage Error
//  //	0x0002 - Overcurrent
//  //	0x0004 - Overvoltage
//  //	0x0008 - Reserved
//  //	0x0080 - Position Error is out of tolerance
//  //	Other  - Reserved
//  auto idx_0x200E = od_obj<uint16_t>(0x200E, "Alarm Code", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x200E_00 = od_obj_ent<uint16_t>(
//    0, nullopt, nullopt, 0, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Alarm Code");
//  idx_0x200E.add_entry(idx_0x200E_00);
//  od.push_back(idx_0x200E);
//
//  // 0x200F - Internal Status Code
//  //	0x0001 - Driver Enabled
//  //	0x0002 - Driver Failed
//  //	0x0004 - In Place Signal
//  //	0x0008 - Motor Running
//  //	0x0010 - Homing Completed
//  //	0x0020 - Driver Ready
//  //	Other  - Reserved
//  auto idx_0x200F = od_obj<uint16_t>(0x200F, "Status Code", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x200F_00 = od_obj_ent<uint16_t>(
//    0, nullopt, nullopt, 0, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Status Code");
//  idx_0x200F.add_entry(idx_0x200F_00);
//  od.push_back(idx_0x200F);
//
//  // 0x2010 - Position Cleared
//  auto idx_0x2010 = od_obj<uint16_t>(0x2010, "Position Cleared", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2010_00 = od_obj_ent<uint16_t>(
//    0, 0, 1, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Zero Position");
//  idx_0x2010.add_entry(idx_0x2010_00);
//  od.push_back(idx_0x2010);
//
//  // 0x2011 - Control Mode
//  //	0 - Open Loop
//  //	1 - Closed Loop
//  //	2 - Closed Loop / FOC Mode
//  auto idx_0x2011 = od_obj<uint16_t>(0x2011, "Control Mode", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2011_00 = od_obj_ent<uint16_t>(
//    0, 0, 2, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Control Mode");
//  idx_0x2011.add_entry(idx_0x2011_00);
//  od.push_back(idx_0x2011);
//
//  // 0x2020 - Encoder Resolution
//  auto idx_0x2020 = od_obj<uint16_t>(0x2020, "Encoder Resolution", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2020_00 = od_obj_ent<uint16_t>(
//    0, 1000, 65535, 4000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Encoder Resolution [PPR]");
//  idx_0x2020.add_entry(idx_0x2020_00);
//  od.push_back(idx_0x2020);
//
//  // 0x2021 - Encoder Position
//  auto idx_0x2021 = od_obj<uint16_t>(0x2021, "Encoder Position", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2021_00 = od_obj_ent<uint16_t>(
//    0, 0, 65535, 0, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Encoder Position");
//  idx_0x2021.add_entry(idx_0x2021_00);
//  od.push_back(idx_0x2021);
//
//  // 0x2022 - Position Excess Alarm Value
//  auto idx_0x2022 = od_obj<uint16_t>(0x2022, "Position Excess Alarm Value", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2022_00 = od_obj_ent<uint16_t>(
//    0, 0, 65535, 4000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Position Track Alarm Value");
//  idx_0x2022.add_entry(idx_0x2022_00);
//  od.push_back(idx_0x2022);
//
//  // 0x2023 - Servo Mode 1 Control Parameters
//  auto idx_0x2023 =
//    od_obj<uint16_t>(0x2023, "Servo Mode 1 Control Parameters", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2023_01 = od_obj_ent<uint16_t>(
//    1, 0, 10000, 2000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Position Loop Kp");
//  auto idx_0x2023_02 = od_obj_ent<uint16_t>(
//    2, 0, 1000, 100, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Position Loop Ki");
//  auto idx_0x2023_03 = od_obj_ent<uint16_t>(
//    3, 0, 10000, 200, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Position Loop Kd");
//  auto idx_0x2023_04 = od_obj_ent<uint16_t>(
//    4, 0, 100, 30, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Position Loop Kvff");
//  auto idx_0x2023_05 = od_obj_ent<uint16_t>(
//    5, 0, 500, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Position Loop Kdi");
//  idx_0x2023.add_entry(idx_0x2023_01);
//  idx_0x2023.add_entry(idx_0x2023_02);
//  idx_0x2023.add_entry(idx_0x2023_03);
//  idx_0x2023.add_entry(idx_0x2023_04);
//  idx_0x2023.add_entry(idx_0x2023_05);
//  od.push_back(idx_0x2023);
//
//  // 0x2024 - In Place Signal
//  auto idx_0x2024 = od_obj<uint16_t>(0x2024, "In Place Signal", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2024_01 = od_obj_ent<uint16_t>(
//    1, 0, 1, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "In Position Mode");
//  auto idx_0x2024_02 = od_obj_ent<uint16_t>(
//    2, 0, 1000, 100, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "In Position Count");
//  auto idx_0x2024_03 = od_obj_ent<uint16_t>(
//    3, 0, 10000, 200, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "In Position Time");
//  idx_0x2024.add_entry(idx_0x2024_01);
//  idx_0x2024.add_entry(idx_0x2024_02);
//  idx_0x2024.add_entry(idx_0x2024_03);
//  od.push_back(idx_0x2024);
//
//  // 0x2025 - Servo Speed Filter
//  //	Normally set Secondary = 3 * Primary
//  auto idx_0x2025 = od_obj<uint16_t>(0x2025, "Servo Speed Filter", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2025_01 = od_obj_ent<uint16_t>(
//    1, 0, 1000, 200, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Primary Low-Pass Speed Feedback Filter BW [Hz]");
//  auto idx_0x2025_02 = od_obj_ent<uint16_t>(
//    2, 0, 2000, 600, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Secondary Low-Pass Speed Feedback Filter BW [Hz]");
//  auto idx_0x2025_03 = od_obj_ent<uint16_t>(
//    3, 0, 5000, 5000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "FOC Speed Loop Output Variable BW [Hz]");
//  idx_0x2025.add_entry(idx_0x2025_01);
//  idx_0x2025.add_entry(idx_0x2025_02);
//  idx_0x2025.add_entry(idx_0x2025_03);
//  od.push_back(idx_0x2025);
//
//  // 0x2026 - Servo Mode 2 Control Parameters
//  auto idx_0x2026 =
//    od_obj<uint16_t>(0x2026, "Servo Mode 2 Control Parameters", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2026_01 = od_obj_ent<uint16_t>(
//    1, 0, 10000, 2000, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Proportional Gain");
//  auto idx_0x2026_02 = od_obj_ent<uint16_t>(
//    2, 0, 1000, 100, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Integral Gain");
//  auto idx_0x2026_03 = od_obj_ent<uint16_t>(
//    3, 0, 10000, 200, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Speed feedback gain 1");
//  auto idx_0x2026_04 = od_obj_ent<uint16_t>(
//    4, 0, 100, 30, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Speed feedback gain 2");
//  auto idx_0x2026_05 = od_obj_ent<uint16_t>(
//    5, 0, 500, 0, OD_OBJ_PERM::RW, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
//    "Speed feedforward gain 1");
//  idx_0x2026.add_entry(idx_0x2026_01);
//  idx_0x2026.add_entry(idx_0x2026_02);
//  idx_0x2026.add_entry(idx_0x2026_03);
//  idx_0x2026.add_entry(idx_0x2026_04);
//  idx_0x2026.add_entry(idx_0x2026_05);
//  od.push_back(idx_0x2026);
//
//  // 0x2043 - Speed Given
//  auto idx_0x2043 = od_obj<uint16_t>(0x2043, "Speed given", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2043_00 = od_obj_ent<uint16_t>(
//    0, -3000, 3000, 0, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Speed Reference");
//  idx_0x2043.add_entry(idx_0x2043_00);
//  od.push_back(idx_0x2043);
//
//  // 0x2044 - Speed Feedback
//  auto idx_0x2044 = od_obj<uint16_t>(0x2044, "Speed feedback", OD_OBJ_CAT::REQUIRED);
//  auto idx_0x2044_00 = od_obj_ent<uint16_t>(
//    0, -3000, 3000, 0, OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED, "Speed Feedback");
//  idx_0x2044.add_entry(idx_0x2044_00);
//  od.push_back(idx_0x2044);
//
//  // 0x2048 - Voltage
//  // 0x2049 - Input Level
//  // 0x204A - Output Level
//  // 0x2060 - Harmonic Amplitude of the First Resonance Point
//  // 0x2061 - First Resonance Point A Phase Harmonic Phase
//  // 0x2062 - First Resonance Point B Phase Harmonic Phase
//
//  // 0x603F - Fault Code
//  // 0x6040 - Control Word
//  // 0x6041 - Status Word
//  // 0x6060 - Operation Mode
//  // 0x6061 - Operation Mode Display
//  // 0x6064 - Actual Position
//  // 0x606C - Actual Speed
//  // 0x607A - Target Location
//  // 0x607C - Home Offset
//  // 0x6083 - Track Acceleration
//  // 0x6084 - Track Deceleration
//  // 0x6085 - Quickstop Declaration
//  // 0x6098 - Homing Method
//  // 0x6099 - Homing Speed
//  // 0x609A - Homing Acceleration
//  // 0x60B8 - Probe Function Setting
//  // 0x60B9 - Probe Status
//  // 0x60BA - Probe 1 Positive Latch Value
//  // 0x60BB - Probe 1 Negative Latch Value
//  // 0x60BC - Probe 2 Positive Latch Value
//  // 0x60BD - Probe 2 Negative Latch Value
//  // 0x60FD - Digital Inputs
//  // 0x60FF - PV Mode Speed Setting
//  // 0x6502 - Operation Mode Supported
//}
