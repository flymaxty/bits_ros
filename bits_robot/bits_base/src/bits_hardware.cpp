#include "bits_hardware.h"

#include <math.h>
#include <errno.h>

/**
* Initialize Bits hardware
*/
BitsHardware::BitsHardware() {

}

void BitsHardware::ControllerTimerCallback(const ros::TimerEvent&) {
  UpdateHardwareStatus();
  controller_manager_->update(ros::Time::now(), ros::Duration(0.1));
  WriteCommandsToHardware();
}

void BitsHardware::Initialize(std::string node_name, ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  node_name_ = node_name;
  nh_ = nh;
  private_nh_ = private_nh;

  private_nh_.param<std::string>("can_send_topic", send_topic_, "sent_messages");
  private_nh_.param<std::string>("can_receive_topic", receive_topic_, "received_messages");
  private_nh_.param<int>("can_id", can_id_, 1);
  private_nh_.param<std::string>("left_wheel_joint", left_wheel_joint_, "drive_wheel_left_joint");
  private_nh_.param<std::string>("right_wheel_joint", right_wheel_joint_, "drive_wheel_right_joint");

  can_pub_ = nh_.advertise<can_msgs::Frame>(send_topic_, 1000);
  can_sub_ = nh_.subscribe<can_msgs::Frame>(receive_topic_, 10, &BitsHardware::CanReceiveCallback, this);

  set_io_service_ = private_nh_.advertiseService("set_digital_output", &BitsHardware::SetDigitalOutputCB, this);

  controller_manager_ = new controller_manager::ControllerManager(this, nh);
  timer_ = nh.createTimer(ros::Duration(0.025), &BitsHardware::ControllerTimerCallback, this);

  RegisterControlInterfaces();

  diagnostic_updater_.setHardwareID("caster_robot");
  diagnostic_updater_.add("Left motor", this, &BitsHardware::LeftMotorCheck);
  diagnostic_updater_.add("Right motor", this, &BitsHardware::RightMotorCheck);
  diagnostic_updater_.add("Status", this, &BitsHardware::StatusCheck);
  diagnostic_updater_.add("Controller", this, &BitsHardware::ControllerCheck);

  // Clear();

  ROS_INFO("can_pub: %s, can_sub: %s, can_id: %d", send_topic_.c_str(), receive_topic_.c_str(), can_id_);
  ROS_INFO("caster base initialized");
}

void BitsHardware::UpdateHardwareStatus() {
  bool success = false;
  uint32_t data;

  /* request motor speed */
  // int16_t l_rpm=-1, r_rpm=-1;
  // uint32_t left_rpm=-1, right_rpm=-1;
  success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kLeftMotor+1), 2);
  success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kRightMotor+1), 2);
  // l_rpm = static_cast<int16_t>(left_rpm);
  // r_rpm = static_cast<int16_t>(right_rpm);

  /* request motor counter */
  // int32_t l_count=-1, r_count=-1;
  success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kLeftMotor+1), 4);
  success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kRightMotor+1), 4);

  // uint8_t status_flag;
  success = Query(kReadStatusFlags, 0x00, 1);
  // status_flag = static_cast<uint8_t>(data);

  // uint8_t fault_flag;
  success = Query(kReadFaultFlags, 0x00, 1);
  // fault_flag = static_cast<uint8_t>(data);

  /* TODO: strange rules for opencan id */
  // uint8_t left_motor_flag, right_motor_flag;
  success = Query(kReadMotorStatusFlags, 0x01, 4);
  // left_motor_flag = data;
  // right_motor_flag = data >> 16;

  success = Query(kReadMotorAmps, 0x01, 4);
  success = Query(kReadMotorAmps, 0x02, 4);

  joints_[kLeftMotor].velocity = motor_status_[kLeftMotor].rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor].velocity = motor_status_[kRightMotor].rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0 * -1.0;

  joints_[kLeftMotor].position = (motor_status_[kLeftMotor].counter-motor_status_[kLeftMotor].counter_offset) / 30.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor].position = (motor_status_[kRightMotor].counter-motor_status_[kRightMotor].counter_offset) / 30.0 / REDUCTION_RATIO * M_PI * 2.0 * -1.0;

  diagnostic_updater_.update();

  // ROS_INFO("motor counter: %d, %d, %d, %d", motor_status_[kLeftMotor].counter, motor_status_[kRightMotor].counter, motor_status_[kLeftMotor].rpm, motor_status_[kRightMotor].rpm);
  // ROS_INFO("motor counter: %f, %f, %d, %d", joints_[0].position, joints_[1].position, l_rpm, r_rpm);
  // ROS_INFO("status: %s, fault: %s, left: %s, right: %s", \
            ToBinary(status_flag, sizeof(status_flag)).c_str(), ToBinary(fault_flag, sizeof(fault_flag)).c_str(), \
            ToBinary(left_motor_flag, sizeof(left_motor_flag)).c_str(), ToBinary(right_motor_flag, sizeof(right_motor_flag)).c_str());
}

/**
* Register interfaces with the RobotHW interface manager, allowing ros_control operation
*/
void BitsHardware::RegisterControlInterfaces() {
  hardware_interface::JointStateHandle left_wheel_joint_state_handle(left_wheel_joint_, &joints_[0].position, &joints_[0].velocity, &joints_[0].effort);
  joint_state_interface_.registerHandle(left_wheel_joint_state_handle);

  hardware_interface::JointHandle left_wheel_joint_handle(left_wheel_joint_state_handle, &joints_[0].velocity_command);
  velocity_joint_interface_.registerHandle(left_wheel_joint_handle);

  hardware_interface::JointStateHandle right_wheel_joint_state_handle(right_wheel_joint_, &joints_[1].position, &joints_[1].velocity, &joints_[1].effort);
  joint_state_interface_.registerHandle(right_wheel_joint_state_handle);

  hardware_interface::JointHandle right_wheel_joint_handle(right_wheel_joint_state_handle, &joints_[1].velocity_command);
  velocity_joint_interface_.registerHandle(right_wheel_joint_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

void BitsHardware::WriteCommandsToHardware() {
  int32_t speed[2];

  speed[0] = static_cast<int32_t>(joints_[0].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
  // int16_t s_v = ntohl(speed);
  // memcpy(buf+5, &s_v, 4);
  Command(kSetVelocity, static_cast<uint8_t>(kLeftMotor+1), static_cast<uint32_t>(speed[0]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kLeftMotor), static_cast<uint32_t>(speed[0]), 4);

  speed[1] = static_cast<int32_t>(joints_[1].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60) * -1.0;
  // s_v = ntohl(speed);
  // memcpy(buf+9, &s_v, 4);
  Command(kSetVelocity, static_cast<uint8_t>(kRightMotor+1), static_cast<uint32_t>(speed[1]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kRightMotor), static_cast<uint32_t>(speed[1]), 4);

  // ROS_INFO("command: %f, %f; rad: %d, %d", joints_[0].velocity_command, joints_[1].velocity_command, speed[0], speed[1]);
}
