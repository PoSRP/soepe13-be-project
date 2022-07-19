#include "ecat_server/ecat_server.hpp"

EcatServer::EcatServer() : Node("ecat_server"), _count(0) {
	using std::placeholders::_1;
	using std::placeholders::_2;

	// Declaring parameters
	declare_parameter<std::string>("network_interface", "not set");

	// Registering message publishers
	_log_publisher = create_publisher<std_msgs::msg::String>("ecat_server/log", 10);
	_network_error_publisher = create_publisher<ecat_interfaces::msg::NetworkError>("ecat_server/network_error", 10);
	_master_status_publisher = create_publisher<ecat_interfaces::msg::MasterStatus>("ecat_server/master_status", 10);
	_master_configuration_publisher = create_publisher<ecat_interfaces::msg::MasterConfiguration>("ecat_server/master_configuration", 10);
	_slave_status_publisher = create_publisher<ecat_interfaces::msg::SlaveStatus>("ecat_server/slave_status", 10);
	_slave_configuration_publisher = create_publisher<ecat_interfaces::msg::SlaveConfiguration>("ecat_server/slave_configuration", 10);

	// Registering services
	_get_connected_slaves_service = create_service<ecat_interfaces::srv::GetConnectedSlaves>("get_connected_slaves",
			std::bind(&EcatServer::get_connected_slaves, this, _1, _2));

	_get_slave_status_service = create_service<ecat_interfaces::srv::GetSlaveStatus>("get_slave_status",
			std::bind(&EcatServer::get_slave_status, this, _1, _2));

	_get_slave_configuration_service = create_service<ecat_interfaces::srv::GetSlaveConfiguration>("get_slave_configuration",
			std::bind(&EcatServer::get_slave_configuration, this, _1, _2));

	_get_slave_configurations_service = create_service<ecat_interfaces::srv::GetSlaveConfigurations>("get_slave_configurations",
			std::bind(&EcatServer::get_slave_configurations, this, _1, _2));

	_set_slave_configuration_service = create_service<ecat_interfaces::srv::SetSlaveConfiguration>("set_slave_configuration",
			std::bind(&EcatServer::set_slave_configuration, this, _1, _2));

	_get_master_status_service = create_service<ecat_interfaces::srv::GetMasterStatus>("get_master_status",
			std::bind(&EcatServer::get_master_status, this, _1, _2));

	_get_master_configuration_service = create_service<ecat_interfaces::srv::GetMasterConfiguration>("get_master_configuration",
			std::bind(&EcatServer::get_master_configuration, this, _1, _2));

	_set_master_configuration_service = create_service<ecat_interfaces::srv::SetMasterConfiguration>("set_master_configuration",
			std::bind(&EcatServer::set_master_configuration, this, _1, _2));
}

// Service functions - Slaves
void EcatServer::get_connected_slaves(
		const std::shared_ptr<ecat_interfaces::srv::GetConnectedSlaves::Request> request,
		std::shared_ptr<ecat_interfaces::srv::GetConnectedSlaves::Response> response) {

	(void) request;
	(void) response;
}
void EcatServer::get_slave_status(
		const std::shared_ptr<ecat_interfaces::srv::GetSlaveStatus::Request> request,
		std::shared_ptr<ecat_interfaces::srv::GetSlaveStatus::Response> response) {

	(void) request;
	(void) response;
}
void EcatServer::get_slave_configuration(
		const std::shared_ptr<ecat_interfaces::srv::GetSlaveConfiguration::Request> request,
		std::shared_ptr<ecat_interfaces::srv::GetSlaveConfiguration::Response> response) {

	(void) request;
	(void) response;
}
void EcatServer::get_slave_configurations(
		const std::shared_ptr<ecat_interfaces::srv::GetSlaveConfigurations::Request> request,
		std::shared_ptr<ecat_interfaces::srv::GetSlaveConfigurations::Response> response) {

	(void) request;
	(void) response;
}
void EcatServer::set_slave_configuration(
		const std::shared_ptr<ecat_interfaces::srv::SetSlaveConfiguration::Request> request,
		std::shared_ptr<ecat_interfaces::srv::SetSlaveConfiguration::Response> response) {

	(void) request;
	(void) response;
}
// Service functions - Master
void EcatServer::get_master_status(
		const std::shared_ptr<ecat_interfaces::srv::GetMasterStatus::Request> request,
		std::shared_ptr<ecat_interfaces::srv::GetMasterStatus::Response> response) {

	(void) request;
	(void) response;
}
void EcatServer::get_master_configuration(
		const std::shared_ptr<ecat_interfaces::srv::GetMasterConfiguration::Request> request,
		std::shared_ptr<ecat_interfaces::srv::GetMasterConfiguration::Response> response) {

	(void) request;
	(void) response;
}
void EcatServer::set_master_configuration(
		const std::shared_ptr<ecat_interfaces::srv::SetMasterConfiguration::Request> request,
		std::shared_ptr<ecat_interfaces::srv::SetMasterConfiguration::Response> response) {

	(void) request;
	(void) response;
}
