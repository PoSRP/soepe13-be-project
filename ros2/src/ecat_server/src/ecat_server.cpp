#include "ecat_server/ecat_server.hpp"

EcatServer::EcatServer(uint64_t pid)
: _node_name("ecat_server_" + std::to_string(pid)), Node("ecat_server_" + std::to_string(pid))
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Registering message publishers
  _log_publisher = create_publisher<std_msgs::msg::String>("ecat_server/log", 10);
}
