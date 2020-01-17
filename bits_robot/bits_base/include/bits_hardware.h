#ifndef BITS_HARDWARE_H_
#define BITS_HARDWARE_H_

#include <string>

#include <cstdlib>
#include <cstring>

#include <sys/types.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

/**
* Class representing Caster hardware, allows for ros_control to modify internal state via joint interfaces
*/
class BitsHardware : public hardware_interface::RobotHW {
  public:
    BitsHardware();

    bool Connect();
    void Initialize(std::string node_name, ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    void UpdateHardwareStatus();
    void WriteCommandsToHardware();

  private:
    void RegisterControlInterfaces();

    void ControllerTimerCallback(const ros::TimerEvent&);

    std::string node_name_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Timer timer_;
    controller_manager::ControllerManager *controller_manager_;

    std::string left_wheel_joint_, right_wheel_joint_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0)
      { }
    } joints_[2];
};

#endif  // BITS_HARDWARE_H_
