#pragma once

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <mujoco_ros_control/mujoco_visualization_utils.h>
#include <mujoco_ros_control/mujoco_robot_hw_sim.h>

#include <mujoco/mujoco.h>

namespace mujoco_ros_control
{
  class MujocoRosControl
  {
  public:
    MujocoRosControl(ros::NodeHandle &nh, ros::NodeHandle &nhp);
    ~MujocoRosControl();

    bool init();
    void update();
    void publishSimTime();
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    bool headless_;
    double render_fps_;
    bool vsync_;
    bool render_shadows_;
    bool render_reflections_;
    int window_width_;
    int window_height_;

  protected:
    boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim> > robot_hw_sim_loader_;
    std::vector<boost::shared_ptr<mujoco_ros_control::RobotHWSim> > robot_hw_sims_;
    std::vector<std::string> robot_namespaces_;

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Time last_update_sim_time_ros_;
    std::vector<boost::shared_ptr<controller_manager::ControllerManager> > controller_managers_;

    ros::Publisher clock_pub_;
    ros::Subscriber control_input_sub_;

    int clock_pub_freq_ = 1000;
    ros::Time last_clock_pub_time_;


  };
}
