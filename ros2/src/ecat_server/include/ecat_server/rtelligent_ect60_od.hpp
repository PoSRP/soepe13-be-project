/*
 * rtelligent-ect60-od.hpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#ifndef ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_
#define ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "ecat_server/generic_od.hpp"
#include "ecat_server/soem_impl.hpp"

namespace soem_impl::rtelligent::ect60
{

// clang-format off

// TODO: Fill this up
inline const std::vector<const od_obj_t> od = {
	/* Communication area */
	od_obj<uint32_t>(0x1000, "Type of device", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint32_t>(
			0, std::nullopt, std::nullopt, 0x00040192,
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Type of device")
	}),
	od_obj<const char *>(0x1001, "Device name", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<const char *>(
			0, std::nullopt, std::nullopt, "ECT60",
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Device name")
	}),
	od_obj<const char *>(0x1009, "Hardware version", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<const char *>(
			0, std::nullopt, std::nullopt, "0xA1",
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Hardware version")
	}),
	od_obj<const char *>(0x100A, "Software version", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<const char *>(
			0, std::nullopt, std::nullopt, "0x101B",
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Software version")
	}),
	od_obj<uint8_t>(0x1010, "Save parameters", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint8_t>(
			0, 1, 1, 1,
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Maximum number of sub-indexes"),
		od_obj_ent<uint8_t>(
			1, 0, 1, 0,
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Save parameters")
	}),
	od_obj<uint8_t>(0x1011, "Restore factory settings", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint8_t>(
			0, 1, 1, 1,
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Maximum number of sub-indexes"),
		od_obj_ent<uint8_t>(
			1, 0, 1, 0,
			OD_OBJ_PERM::RO, OD_OBJ_PDO::NO, OD_OBJ_CAT::REQUIRED,
			"Save parameters")
	}),
	/* Manufacturer area */
	od_obj<uint16_t>(0x2000, "Operating current", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2001, "Subdivision / resolution", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2002, "Standby time", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2003, "Percentage of standby current", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2005, "Output port function", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2006, "Output port polarity", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2007, "Input port function", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2008, "Input port polarity", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2009, "Filter time", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x200A, "Shaft lock time", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x200B, "Current loop parameters", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x200C, "Motor parameters", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x200D, "Run reverse", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x200E, "Internal alarm code", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x200F, "Internal status code", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2010, "Position cleared", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2011, "Control mode", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2020, "Encoder resolution", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2021, "Encoder position", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2022, "Position excess alarm value", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2023, "Servo mode 1 control parameters", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2024, "In place signal", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2025, "Servo speed filter", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2026, "Servo mode 2 control parameters", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2043, "Speed given", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2044, "Speed feedback", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2048, "Voltage", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2049, "Input level", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x204A, "Output level", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2060, "Harmonic amplitude of the first resonance point", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2061, "First resonance point A phase harmonic phase", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	od_obj<uint16_t>(0x2062, "First resonance point B phase harmonic phase", OD_OBJ_CAT::REQUIRED, {
		od_obj_ent<uint16_t>()
	}),
	/* Profile area */
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
	od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	}),
	od_obj<uint8_t>({
		od_obj_ent<const char *>()
	})
};

// clang-format on

std::optional<std::vector<sm_conf_t>> get_sm_confs(
  std::vector<od_obj_t> in, std::vector<od_obj_t> out)
{
  sm_conf_t sm_out(2, 0x1600, SM_DIR::OUT, out);
  sm_conf_t sm_in(3, 0x1A00, SM_DIR::IN, in);
  return std::nullopt;
}

auto vec = get_sm_confs({}, {});

}  // namespace soem_impl::rtelligent::ect60

#endif /* ROS2_SRC_ECAT_SERVER_INCLUDE_ECAT_SERVER_RTELLIGENT_ECT60_OD_HPP_ */
