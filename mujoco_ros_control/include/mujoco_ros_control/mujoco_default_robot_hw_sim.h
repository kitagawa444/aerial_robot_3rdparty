#pragma once

#include <urdf/model.h>
#include <pluginlib/class_list_macros.h>
#include <mujoco_ros_control/mujoco_robot_hw_sim.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>


namespace mujoco_ros_control
{
  class DefaultRobotHWSim : public mujoco_ros_control::RobotHWSim
  {
  public:
    DefaultRobotHWSim() {};
    ~DefaultRobotHWSim() {}

    bool virtual init(const std::string& robot_namespace,
              ros::NodeHandle model_nh,
              mjModel* mujoco_model,
              mjData* mujoco_data
              );

    void virtual read(const ros::Time& time, const ros::Duration& period);

    void virtual write(const ros::Time& time, const ros::Duration& period);

    void controlInputCallback(const sensor_msgs::JointState & msg);

    void jointPositionCallback(const sensor_msgs::JointState & msg);
    void rootPoseCallback(const geometry_msgs::Pose & msg);

  protected:
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    std::string robot_namespace_;
    std::string name_prefix_;

    hardware_interface::JointStateInterface  jnt_state_interface_;
    hardware_interface::EffortJointInterface effort_jnt_interface_;

    std::vector<std::string> joint_list_;
    std::vector<int> actuator_id_list_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber control_input_sub_;
    ros::Subscriber dierct_joint_position_sub_;
    ros::Subscriber dierct_root_pose_sub_;
    double joint_state_pub_rate_ = 0.02;
    std::vector<double> control_input_;

    ros::Time last_joint_state_time_;

    sensor_msgs::JointState direct_joint_position_;
    geometry_msgs::Pose direct_root_pose_;
    bool direct_root_pose_flag_;

        // 関節名・状態・コマンド格納
      std::vector<std::string> model_joint_names_;
      std::vector<std::string> ros_joint_names_;
    std::vector<double> pos_;
    std::vector<double> vel_;
    std::vector<double> eff_;
    std::vector<double> cmd_eff_;

    // 関節インデックス → アクチュエータ ID（ctrl 配列のインデックス）対応
    std::vector<int> act_id_by_joint_idx_;
    std::vector<int> managed_actuator_ids_;
    int root_joint_id_ = -1;

    bool use_ros_control_{false};
    bool allow_direct_state_set_{true};        // テレポート許可

    bool matchesRobotNamespace(const std::string& name) const;
    std::string stripNamePrefix(const std::string& name) const;
    std::string qualifyName(const std::string& name) const;
    void registerManagedActuator(int actuator_id);
    void resolveRootJoint();

  };
}
