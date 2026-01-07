// ==============================
// STEP3: Position publish and visualization in Rviz
// ==============================

// Standard headers
#include <stdlib.h>
#include <stdio.h>
#include <chrono>

// Dynamixel SDK header
#include <dynamixel_sdk/dynamixel_sdk.h>

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

// Control table adress (XM430-W210, XC330-T288)
#define ADDR_PRESENT_POSITION    116

// Data byte length
#define LEN_PRESENT_POSITION     4

// Protocal version
#define PROTOCAL_VERSION         2.0

// Settings
#define DXL1_ID                  1
#define DXL2_ID                  2
#define DXL3_ID                  3
#define DXL4_ID                  4
#define DXL5_ID                  5
#define DXL6_ID                  6
#define DXL7_ID                  7

#define DXL1_NAME                "joint_1"
#define DXL2_NAME                "joint_2"
#define DXL3_NAME                "joint_3"
#define DXL4_NAME                "joint_4"
#define DXL5_NAME                "joint_5"
#define DXL6_NAME                "joint_6"
#define DXL7_NAME                "joint_7"

#define NUMBER_OF_DXL            7                 // Do not include the last Dynamixel

#define BAUDRATE                 4000000
#define DEVICENAME               "/dev/ttyUSB0"

#define OFFSET                   2048              // Offset for Dynamixel motor positions -> radians
#define SCALE_FACTOR             3.141592/2048     // Scale factor for Dynamixel motor positions -> radians


// Publisher class
class JointStatePublisher : public rclcpp::Node
{
  public:
  JointStatePublisher()
  : Node("joint_state_publisher"),
    portHandler(dynamixel::PortHandler::getPortHandler(DEVICENAME)),
    packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCAL_VERSION)),
    groupSyncRead(
      portHandler,
      packetHandler,
      ADDR_PRESENT_POSITION,
      LEN_PRESENT_POSITION)
  {
    // ========== Dynamixel Initialization ==========
    // Open port
    if (portHandler->openPort())
    {
      RCLCPP_INFO(this->get_logger(), "Succeeded to open the port");
    }
    else
    {
      throw std::runtime_error("Failed to open the port!\n");
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      RCLCPP_INFO(this->get_logger(), "Succeeded to change the baudrate");
    }
    else
    {
      throw std::runtime_error("Failed to change the baudrate!\n");
    }

    // Add parameter storage for Dynamixel present position values
    for (int i = 0; i < NUMBER_OF_DXL; i++)
    {
      dxl_addparam_result = groupSyncRead.addParam(dxl_ids[i]);
      if (dxl_addparam_result == true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam succeeded!\n", dxl_ids[i]);
      }
      else
      {
        throw std::runtime_error("[ID:" + std::to_string(dxl_ids[i]) + "] groupSyncRead addparam failed");
      }
    }

    // ========== ROS2 Node Initialization ==========
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(5ms, std::bind(&JointStatePublisher::publish_joint_states, this));    // 200Hz publish of states

    joint_names_ = {DXL1_NAME, DXL2_NAME, DXL3_NAME, DXL4_NAME, DXL5_NAME, DXL6_NAME, DXL7_NAME};

    RCLCPP_INFO(this->get_logger(), "Joints state publisher started");
  }

  ~JointStatePublisher()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down Dynamixel port");
    portHandler->closePort();
  }

  private:
  void publish_joint_states()
  {
    sensor_msgs::msg::JointState joint_msg;

    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = joint_names_;
    
    joint_msg.position.resize(NUMBER_OF_DXL);
    joint_msg.velocity.resize(NUMBER_OF_DXL);
    joint_msg.effort.resize(NUMBER_OF_DXL);

    // Get current positions
    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      return;
    }

    for (int i = 0; i < NUMBER_OF_DXL; i++)
    {
      dxl_position_raw = groupSyncRead.getData(dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      joint_msg.position[i] = (dxl_position_raw-OFFSET)*SCALE_FACTOR;
    }

    // Get current velocities TODO
    for (int i = 0; i < NUMBER_OF_DXL; i++)
    {
      joint_msg.velocity[i] = 0.0;
    }

    // Efforts
    for (int i = 0; i < NUMBER_OF_DXL; i++)
    {
      joint_msg.effort[i] = 0.0;
    }

    publisher_->publish(joint_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  dynamixel::GroupSyncRead groupSyncRead;

  int dxl_ids[NUMBER_OF_DXL] = {DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID, DXL7_ID};
  int dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  int32_t dxl_position_raw = 0;
  std::vector<std::string> joint_names_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}