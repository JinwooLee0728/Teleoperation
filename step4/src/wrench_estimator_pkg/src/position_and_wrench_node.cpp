// ==================================================
// STEP 4: End effector wrench estimation and visualization
// ==================================================


// Standard headers
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

// Dynamixel SDK header
#include <dynamixel_sdk/dynamixel_sdk.h>

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

// Pinocchio headers
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"

// Custom headers (Wrench estimation)
#include "wrench_estimator_pkg/wrench_estimator.hpp"

using namespace std::chrono_literals;

// URDF Address (Absolute)
#define URDF_FILENAME            "/home/jinwoo/Desktop/teleop_prj/FACTR_materials/FACTR_Hardware/urdf/factr_teleop_franka.urdf"

// Control table address (XM430-W210, XC330-T288)
#define ADDR_PRESENT_VELOCITY    128
#define ADDR_PRESENT_POSITION    132

// Data byte length
#define LEN_PRESENT_POSITION     4
#define LEN_PRESENT_VELOCITY     4

// Protocal version
#define PROTOCAL_VERSION         2.0

// Miscellaneous
#define NUMBER_OF_DXL            7
#define DXL1_ID                  1
#define DXL2_ID                  2
#define DXL3_ID                  3
#define DXL4_ID                  4
#define DXL5_ID                  5
#define DXL6_ID                  6
#define DXL7_ID                  7
#define DXL1_NAME                "joint_1"    // Must match URDF joint names
#define DXL2_NAME                "joint_2"
#define DXL3_NAME                "joint_3"
#define DXL4_NAME                "joint_4"
#define DXL5_NAME                "joint_5"
#define DXL6_NAME                "joint_6"
#define DXL7_NAME                "joint_7"

#define BAUDRATE                 4000000
#define DEVICENAME               "/dev/ttyUSB0"

#define POS_OFFSET               2048                  // Pos. offset for Dynamixel readings conversion
#define POS_SCALE_FACTOR         3.141592/2048.0       // Pos. scale factor for Dynamixel readings conversion
#define VEL_SCALE_FACTOR         0.229*(2*3.141592/60) // Vel. scale fcator for Dynamixel readings conversion

#define QOS                      10

// Publisher class
class PositionAndWrenchPublisher : public rclcpp::Node
{
    public:
    PositionAndWrenchPublisher()
    : Node("position_and_wrench_publisher"),
      portHandler(dynamixel::PortHandler::getPortHandler(DEVICENAME)),
      packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCAL_VERSION)),
      groupSyncRead_pos(
        portHandler,
        packetHandler,
        ADDR_PRESENT_POSITION,
        LEN_PRESENT_POSITION
      ),
      groupSyncRead_vel(
        portHandler,
        packetHandler,
        ADDR_PRESENT_VELOCITY,
        LEN_PRESENT_VELOCITY
      )
    {
        // ========== Dynamixel Communication Initialization ==========
        // Open Port
        if (portHandler->openPort()) {
            RCLCPP_INFO(this->get_logger(), "Succeeded in opening the port.");
        }
        else {
            throw std::runtime_error("Failed to open the port.");
        }

        // Set port baudrate
        if (portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_INFO(this->get_logger(), "Succeeded in changing the baudrate.");
        }
        else {
            throw std::runtime_error("Failed to change the baudrate.");
        }

        // Add parameter storage for Dynamixel present position and velocity values
        for (int i = 0; i < NUMBER_OF_DXL; i++) {
            dxl_addparam_result_pos = groupSyncRead_pos.addParam(dxl_ids[i]);
            if (dxl_addparam_result_pos == true) {
                RCLCPP_INFO(this->get_logger(), "[ID: %03d] groupSyncRead_pos addparam succeeded.", dxl_ids[i]);
            }
            else {
                throw std::runtime_error("[ID: " + std::to_string(dxl_ids[i]) + "] groupSyncRead_pos addparam failed.");
            }

            dxl_addparam_result_vel = groupSyncRead_vel.addParam(dxl_ids[i]);
            if (dxl_addparam_result_vel == true) {
                RCLCPP_INFO(this->get_logger(), "[ID: %03d] groupSyncRead_vel addparam succeeded.", dxl_ids[i]);
            }
            else {
                throw std::runtime_error("[ID: " + std::to_string(dxl_ids[i]) + "] groupSyncRead_vel addparam failed.");
            }
        }

        // ========== ROS2 Node Initialization ==========
        joint_states_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", QOS);
        wrench_publisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>("end_effector_wrench", QOS);
        timer = this->create_wall_timer(10ms, std::bind(&PositionAndWrenchPublisher::publish_position_and_wrench, this));
        
        last_time = this->get_clock()->now();

        joint_names = {DXL1_NAME, DXL2_NAME, DXL3_NAME, DXL4_NAME, DXL5_NAME, DXL6_NAME, DXL7_NAME};

        RCLCPP_INFO(this->get_logger(), "Joint states publisher and end effector wrench publisher started.");

        // ========== Pinocchio Initialization ==========
        pinocchio::urdf::buildModel(URDF_FILENAME, model);
        data = pinocchio::Data(model);

        if (!model.existFrame("link_7")) {
            throw std::runtime_error("End-effector frame not found in URDF");
        }
        end_effector_frame_id = model.getFrameId("link_7");

        q.resize(model.nq);
        q_dot.resize(model.nv);
        wrench_hat.resize(6);

        p0.setZero(model.nv);       // Assume that the robot is initially stationary
        tau.setZero(model.nv);      // All zero values for now
        integral_value.setZero(model.nv);
    }

    ~PositionAndWrenchPublisher()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Dynamixel port");
        portHandler->closePort();
    }

    private:
    void publish_position_and_wrench()
    {
        // ========== Time ==========
        curr_time = this->get_clock()->now();
        dt_sec = (curr_time-last_time).seconds();
        last_time = curr_time;

        // ========== Joint position message ==========
        sensor_msgs::msg::JointState joint_pos_msg;
        joint_pos_msg.header.stamp = curr_time;
        joint_pos_msg.name = joint_names;
        joint_pos_msg.position.resize(NUMBER_OF_DXL);
        joint_pos_msg.velocity.resize(NUMBER_OF_DXL);
        joint_pos_msg.effort.resize(NUMBER_OF_DXL, 0.0);

        // Getting current positions and velocities
        dxl_comm_result_pos = groupSyncRead_pos.txRxPacket();
        if (dxl_comm_result_pos != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result_pos));
            return;
        }

        dxl_comm_result_vel = groupSyncRead_vel.txRxPacket();
        if (dxl_comm_result_vel != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result_vel));
            return;
        }

        for (int i = 0; i < NUMBER_OF_DXL; i++) {
            dxl_position_raw = groupSyncRead_pos.getData(dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            joint_pos_msg.position[i] = (dxl_position_raw-POS_OFFSET)*POS_SCALE_FACTOR;

            dxl_velocity_raw = groupSyncRead_vel.getData(dxl_ids[i], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
            joint_pos_msg.velocity[i] = dxl_velocity_raw*VEL_SCALE_FACTOR;
        }
        
        // ========== End effector wrench message ==========
        geometry_msgs::msg::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = curr_time;
        wrench_msg.header.frame_id = "link_7";  //base_link, link_7

        for (int i = 0; i < NUMBER_OF_DXL; i++) {
            q[i] = joint_pos_msg.position[i];
            q_dot[i] = joint_pos_msg.velocity[i];
        }
        estimateWrench(model, data, end_effector_frame_id, q, q_dot, tau, p0, dt_sec, wrench_hat, integral_value);

        wrench_msg.wrench.force.x = wrench_hat[0];
        wrench_msg.wrench.force.y = wrench_hat[1];
        wrench_msg.wrench.force.z = wrench_hat[2];

        wrench_msg.wrench.torque.x = wrench_hat[3];
        wrench_msg.wrench.torque.y = wrench_hat[4];
        wrench_msg.wrench.torque.z = wrench_hat[5];

        // ========== Publish ==========
        joint_states_publisher->publish(joint_pos_msg);
        wrench_publisher->publish(wrench_msg);
    }

    // ========== Private Class Members ==========
    // Dynamixel Related
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncRead groupSyncRead_pos;
    dynamixel::GroupSyncRead groupSyncRead_vel;

    // ROS Related
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time last_time;
    rclcpp::Time curr_time;

    // Pinochio Related
    pinocchio:: Model model;
    pinocchio:: Data data;
    pinocchio::FrameIndex end_effector_frame_id;

    // For Wrench Estimation
    Eigen::VectorXd q;
    Eigen::VectorXd q_dot;
    Eigen::VectorXd tau;
    Eigen::VectorXd p0;
    Eigen::VectorXd wrench_hat;
    Eigen::VectorXd integral_value;

    // Miscellaneous
    int dxl_ids[NUMBER_OF_DXL] = {DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID, DXL7_ID};
    int dxl_comm_result_pos = COMM_TX_FAIL;
    int dxl_comm_result_vel = COMM_TX_FAIL;
    bool dxl_addparam_result_pos = false;
    bool dxl_addparam_result_vel = false;
    double dt_sec = 0.0;
    int32_t dxl_position_raw = 0;
    int32_t dxl_velocity_raw = 0;
    std::vector<std::string> joint_names;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionAndWrenchPublisher>());
    rclcpp::shutdown();

    return 0;
}