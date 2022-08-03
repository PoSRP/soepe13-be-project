cmake_minimum_required(VERSION 3.8)
project(ecat_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MasterConfiguration.msg
  msg/MasterStatus.msg
  msg/NetworkError.msg
  msg/SlaveConfiguration.msg
  msg/SlaveStatus.msg
  msg/gen/EcatCoEInitCmd.msg
  msg/gen/EcatCoeMailbox.msg
  msg/gen/EcatDC.msg
  msg/gen/EcatDataType.msg
  msg/gen/EcatDescription.msg
  msg/gen/EcatDevice.msg
  msg/gen/EcatDeviceEsc.msg
  msg/gen/EcatDeviceInfo.msg
  msg/gen/EcatDeviceName.msg
  msg/gen/EcatDeviceRegister.msg
  msg/gen/EcatDeviceSM.msg
  msg/gen/EcatDeviceType.msg
  msg/gen/EcatDictionary.msg
  msg/gen/EcatDictionaryObject.msg
  msg/gen/EcatEEPROM.msg
  msg/gen/EcatEEPROMCategory.msg
  msg/gen/EcatFlagType.msg
  msg/gen/EcatGroup.msg
  msg/gen/EcatGroupName.msg
  msg/gen/EcatInfoEndItem.msg
  msg/gen/EcatInfoSubItem.msg
  msg/gen/EcatMailbox.msg
  msg/gen/EcatOpMode.msg
  msg/gen/EcatPDO.msg
  msg/gen/EcatPdoEntry.msg
  msg/gen/EcatPdoExclude.msg
  msg/gen/EcatProfile.msg
  msg/gen/EcatSubDataType.msg
  msg/gen/EcatVendor.msg
  srv/GetConnectedSlaves.srv
  srv/GetMasterConfiguration.srv
  srv/GetMasterStatus.srv
  srv/GetSlaveConfiguration.srv
  srv/GetSlaveConfigurations.srv
  srv/GetSlaveStatus.srv
  srv/SetMasterConfiguration.srv
  srv/SetSlaveConfiguration.srv
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()