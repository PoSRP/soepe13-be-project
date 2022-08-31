/*
 * soem_impl.cpp
 *
 *  Created on: Aug 30, 2022
 *      Author: sr
 */

#include "ecat_server/soem_impl.hpp"

#include "ecat_server/soem_impl.tpp"

using OD_KEY = EtherCAT::OD_KEY;
template <class T>
using OBJ = EtherCAT::DictionaryObject<T>;

EtherCAT::generic_od = {
  {OD_KEY(0, 0), OBJ<const char *>(0, 0, "", "", DICT_OBJ_PERMISSION::READ_ONLY, "Device Type")},
  {OD_KEY(1, 0), OBJ<uint16_t>(1, 0, 0, 65535, DICT_OBJ_PERMISSION::READ_ONLY, "Error Register")},
  {OD_KEY(2, 0), OBJ<uint16_t>(2, 0, 0, 65535, DICT_OBJ_PERMISSION::READ_ONLY, "Something Else")}};
