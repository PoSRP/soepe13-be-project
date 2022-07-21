cmake_minimum_required(VERSION 3.8)
project(ecat_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EcatCoEInitCmdMsg.msg"
  "msg/EcatCoeMailboxMsg.msg"
  "msg/EcatDCMsg.msg"
  "msg/EcatDataTypeMsg.msg"
  "msg/EcatDescriptionMsg.msg"
  "msg/EcatDeviceEscMsg.msg"
  "msg/EcatDeviceInfoMsg.msg"
  "msg/EcatDeviceMsg.msg"
  "msg/EcatDeviceNameMsg.msg"
  "msg/EcatDeviceRegisterMsg.msg"
  "msg/EcatDeviceSMMsg.msg"
  "msg/EcatDeviceTypeMsg.msg"
  "msg/EcatDictionaryMsg.msg"
  "msg/EcatDictionaryObjectMsg.msg"
  "msg/EcatEEPROMCategoryMsg.msg"
  "msg/EcatEEPROMMsg.msg"
  "msg/EcatFlagTypeMsg.msg"
  "msg/EcatGroupMsg.msg"
  "msg/EcatGroupNameMsg.msg"
  "msg/EcatInfoEndItemMsg.msg"
  "msg/EcatInfoSubItemMsg.msg"
  "msg/EcatMailboxMsg.msg"
  "msg/EcatOpModeMsg.msg"
  "msg/EcatPDOMsg.msg"
  "msg/EcatPdoEntryMsg.msg"
  "msg/EcatPdoExcludeMsg.msg"
  "msg/EcatProfileMsg.msg"
  "msg/EcatSubDataTypeMsg.msg"
  "msg/EcatVendorMsg.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()