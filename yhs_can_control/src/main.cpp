#include "yhs_can_control/yhs_dgt_can_control_node.hpp"
#include "yhs_can_control/yhs_fw_can_control_node.hpp"
#include "yhs_can_control/yhs_fr_can_control_node.hpp"
#include "yhs_can_control/yhs_mk_can_control_node.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("yhs_can_control_node");

  std::string chassis_type;
  node->declare_parameter("chassis_type", "DGT");
  node->get_parameter("chassis_type", chassis_type);

  std::shared_ptr<yhs_chassis::CanControl> cc;

  if (chassis_type == "DGT") {
      cc = std::make_shared<can_control::DgtCanControl>(node);
  } else if (chassis_type == "FW") {
      cc = std::make_shared<can_control::FwCanControl>(node);
  } else if (chassis_type == "FR") {
      cc = std::make_shared<can_control::FrCanControl>(node);
  } else if (chassis_type == "MK") {
      cc = std::make_shared<can_control::MkCanControl>(node);
  } else {
      RCLCPP_ERROR(node->get_logger(), "Invalid chassis type. Please specify DGT, FW, FR, or MK.");
      return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Chassis type: %s", chassis_type.c_str());


	if (!cc->Run())
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to initialize yhs_can_control_node");
		return 1;
	}

	RCLCPP_INFO(node->get_logger(), "yhs_can_control_node initialized successfully");

	rclcpp::spin(node);

	cc->Stop();

	RCLCPP_INFO(node->get_logger(), "yhs_can_control_node stopped");

	rclcpp::shutdown();

	return 0;
}