#include <mujoco_ros_control/mujoco_default_robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool DefaultRobotHWSim::matchesRobotNamespace(const std::string& name) const
  {
    if(name_prefix_.empty()) return true;
    return name.find(name_prefix_) == 0;
  }

  std::string DefaultRobotHWSim::stripNamePrefix(const std::string& name) const
  {
    if(name_prefix_.empty()) return name;
    if(name.find(name_prefix_) == 0) return name.substr(name_prefix_.size());
    return name;
  }

  std::string DefaultRobotHWSim::qualifyName(const std::string& name) const
  {
    if(name_prefix_.empty() || name.find(name_prefix_) == 0) return name;
    return name_prefix_ + name;
  }

  void DefaultRobotHWSim::registerManagedActuator(int actuator_id)
  {
    if(actuator_id < 0) return;
    if(std::find(managed_actuator_ids_.begin(), managed_actuator_ids_.end(), actuator_id) == managed_actuator_ids_.end())
      {
        managed_actuator_ids_.push_back(actuator_id);
      }
  }

  void DefaultRobotHWSim::resolveRootJoint()
  {
    root_joint_id_ = -1;

    for (int body_id = 0; body_id < mujoco_model_->nbody; ++body_id)
      {
        const char* body_name = mj_id2name(mujoco_model_, mjOBJ_BODY, body_id);
        if (body_name && !matchesRobotNamespace(body_name)) continue;
        if (mujoco_model_->body_jntnum[body_id] <= 0) continue;

        int joint_id = mujoco_model_->body_jntadr[body_id];
        if (joint_id < 0) continue;

        int joint_type = mujoco_model_->jnt_type[joint_id];
        if (joint_type == mjJNT_FREE || joint_type == mjJNT_BALL)
          {
            root_joint_id_ = joint_id;
            return;
          }
      }
  }

  bool DefaultRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    robot_namespace_ = robot_namespace;
    name_prefix_ = robot_namespace.empty() ? std::string("") : robot_namespace + "_";

    model_nh.param("use_ros_control", use_ros_control_, false);
    model_nh.param("allow_direct_state_set", allow_direct_state_set_, true);

    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;

    actuator_id_list_.resize(0);
    int* actuator_trntype = mujoco_model_->actuator_trntype;
    int* actuator_trnid = mujoco_model_->actuator_trnid;

    control_input_.resize(mujoco_model_->nu);
    joint_list_.resize(0);
    managed_actuator_ids_.clear();

    // get joint names from mujoco model
    for(int i = 0; i < mujoco_model_->njnt; i++)
      {
        if(mujoco_model_->jnt_type[i] > 1)
          {
            const char* joint_name = mj_id2name(mujoco_model_, mjtObj_::mjOBJ_JOINT, i);
            if(!joint_name || !matchesRobotNamespace(joint_name)) continue;
            joint_list_.push_back(joint_name);

            for(int j = 0; j < mujoco_model_->nu; j++) {
              if (actuator_trnid[2 * j] == i && actuator_trntype[j] == mjtTrn_::mjTRN_JOINT) {
                actuator_id_list_.push_back(j);
                registerManagedActuator(j);
              }
            }
          }
      }

    joint_state_pub_ = model_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    control_input_sub_ = model_nh.subscribe("mujoco/ctrl_input", 1, &DefaultRobotHWSim::controlInputCallback, this);

    dierct_joint_position_sub_ = model_nh.subscribe("mujoco/direct_joint_position", 1, &DefaultRobotHWSim::jointPositionCallback, this);
    dierct_root_pose_sub_ = model_nh.subscribe("mujoco/direct_root_pose", 1, &DefaultRobotHWSim::rootPoseCallback, this);
    direct_root_pose_flag_ = false;


    // 1DOF 
    model_joint_names_.clear();
    ros_joint_names_.clear();
    for (int j = 0; j < mujoco_model_->njnt; ++j)
      {
        if (mujoco_model_->jnt_type[j] > 1)
          {
            const char* jname = mj_id2name(mujoco_model_, mjOBJ_JOINT, j);
            if (jname && matchesRobotNamespace(jname))
              {
                model_joint_names_.push_back(std::string(jname));
                ros_joint_names_.push_back(stripNamePrefix(jname));
              }
          }
      }

    resolveRootJoint();

    const size_t nj = model_joint_names_.size();
    pos_.assign(nj, 0.0);
    vel_.assign(nj, 0.0);
    eff_.assign(nj, 0.0);
    cmd_eff_.assign(nj, 0.0);
    act_id_by_joint_idx_.assign(nj, -1);

    // 名前一致: joint 名と同名の actuator を探す
    for (size_t i = 0; i < nj; ++i)
      {
        int act_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, model_joint_names_[i].c_str());
        act_id_by_joint_idx_[i] = act_id; // 無ければ -1 のまま
        registerManagedActuator(act_id);
        if (act_id < 0)
          {
            ROS_WARN_STREAM("No actuator found for joint " << model_joint_names_[i]
                            << ". This joint will be read-only.");
          }
      }

    // JointStateInterface 登録
    for (size_t i = 0; i < nj; ++i)
      {
        hardware_interface::JointStateHandle sh(ros_joint_names_[i], &pos_[i], &vel_[i], &eff_[i]);
        jnt_state_interface_.registerHandle(sh);
      }
    registerInterface(&jnt_state_interface_);

    // EffortJointInterface 登録（書き込み先は cmd_eff_）
    for (size_t i = 0; i < nj; ++i)
      {
        hardware_interface::JointHandle eh(jnt_state_interface_.getHandle(ros_joint_names_[i]), &cmd_eff_[i]);
        effort_jnt_interface_.registerHandle(eh);
      }
    registerInterface(&effort_jnt_interface_);
    ROS_INFO_STREAM("Registered " << nj << " joints to EffortJointInterface.");


    return true;
  }

  void DefaultRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    const mjtNum* qpos = mujoco_data_->qpos;
    const mjtNum* qvel = mujoco_data_->qvel;

    // 1) ros_control 用バッファへ反映
    const mjtNum* qfrc_act = mujoco_data_->qfrc_actuator;
    for (size_t i = 0; i < model_joint_names_.size(); ++i)
      {
        int j = mj_name2id(mujoco_model_, mjOBJ_JOINT, model_joint_names_[i].c_str());
        if (j < 0) continue;

        int qposadr = mujoco_model_->jnt_qposadr[j];
        int dofadr  = mujoco_model_->jnt_dofadr[j];

        pos_[i] = qpos[qposadr];
        vel_[i] = qvel[dofadr];
        eff_[i] = qfrc_act[dofadr];
      }

    // 2) joint_states トピックを publish
    if((time - last_joint_state_time_).toSec() >= joint_state_pub_rate_)
      {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = time;
        joint_state_msg.name = ros_joint_names_;

        mjtNum* actuator_force = mujoco_data_->actuator_force;
        int* jnt_qposadr = mujoco_model_->jnt_qposadr;
        int* jnt_dofadr = mujoco_model_->jnt_dofadr;

        for(size_t k = 0; k < model_joint_names_.size(); k++)
          {
            int jid = mj_name2id(mujoco_model_, mjOBJ_JOINT, model_joint_names_[k].c_str());
            if (jid < 0) continue;
            joint_state_msg.position.push_back(qpos[jnt_qposadr[jid]]);
            joint_state_msg.velocity.push_back(qvel[jnt_dofadr[jid]]);
            int act = act_id_by_joint_idx_[k];
            joint_state_msg.effort.push_back(act >= 0 ? actuator_force[act] : 0.0);
          }

        joint_state_pub_.publish(joint_state_msg);
        last_joint_state_time_ = time;
      }
  }

  static inline double clip(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}


void DefaultRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
{
  // 1) control_input_（トピック経由）を全アクチュエータの ctrl に書き込む
  for(size_t idx = 0; idx < managed_actuator_ids_.size(); idx++)
    {
      int actuator_id = managed_actuator_ids_[idx];
      mujoco_data_->ctrl[actuator_id] = control_input_[actuator_id];
    }

  // 2) ros_control の cmd_eff_ を加算
  if (use_ros_control_)
    {
      for (size_t i = 0; i < model_joint_names_.size(); ++i)
        {
          int act = act_id_by_joint_idx_[i];
          if (act < 0) continue;
          mujoco_data_->ctrl[act] += cmd_eff_[i];
        }
    }

  // 3) ctrl limit を適用
  for (int i = 0; i < mujoco_model_->nu; ++i)
    {
      if (mujoco_model_->actuator_ctrllimited[i])
        {
          double lo = mujoco_model_->actuator_ctrlrange[2*i+0];
          double hi = mujoco_model_->actuator_ctrlrange[2*i+1];
          mujoco_data_->ctrl[i] = clip(mujoco_data_->ctrl[i], lo, hi);
        }
    }

  // 4) 直接書き換え（テレポート）。頻繁にやると数値的に不安定になるので注意
  if (allow_direct_state_set_)
    {
      bool teleported = false;

      // root pose
      if (direct_root_pose_flag_ && root_joint_id_ >= 0)
        {
          mjtNum* qpos = mujoco_data_->qpos;
          int qposadr = mujoco_model_->jnt_qposadr[root_joint_id_];
          switch (mujoco_model_->jnt_type[root_joint_id_]) {
          case mjJNT_FREE:
            qpos[qposadr + 0] = direct_root_pose_.position.x;
            qpos[qposadr + 1] = direct_root_pose_.position.y;
            qpos[qposadr + 2] = direct_root_pose_.position.z;
            qpos[qposadr + 3] = direct_root_pose_.orientation.w;
            qpos[qposadr + 4] = direct_root_pose_.orientation.x;
            qpos[qposadr + 5] = direct_root_pose_.orientation.y;
            qpos[qposadr + 6] = direct_root_pose_.orientation.z;
            teleported = true;
            break;
          case mjJNT_BALL:
            qpos[qposadr + 0] = direct_root_pose_.orientation.w;
            qpos[qposadr + 1] = direct_root_pose_.orientation.x;
            qpos[qposadr + 2] = direct_root_pose_.orientation.y;
            qpos[qposadr + 3] = direct_root_pose_.orientation.z;
            teleported = true;
            break;
          default:
            break;
          }
          direct_root_pose_flag_ = false;
        }

      // joint positions
      if (!direct_joint_position_.name.empty())
        {
          auto& names = direct_joint_position_.name;
          auto& target_positions = direct_joint_position_.position;

          for (size_t k = 0; k < names.size(); ++k)
            {
              const std::string joint_name = qualifyName(names[k]);
              int jid = mj_name2id(mujoco_model_, mjOBJ_JOINT, joint_name.c_str());
              if (jid < 0) { ROS_WARN_STREAM("mujoco: joint name " << joint_name << " does not exist"); continue; }

              int qposadr = mujoco_model_->jnt_qposadr[jid];
              mujoco_data_->qpos[qposadr] = target_positions[k];
              teleported = true;
            }
          direct_joint_position_.name.clear();
          direct_joint_position_.position.clear();
        }

      // 状態を直接いじった場合、整合性を取る
      if (teleported)
        {
          // 速度をゼロにするなどの安定化（任意）
          // std::fill(mujoco_data_->qvel, mujoco_data_->qvel + mujoco_model_->nv, 0.0);
          // 完全再計算（mj_step1/mj_step2 の間だが、forward で一旦一貫性を確保）
          mj_forward(mujoco_model_, mujoco_data_);
        }
    }
}

  void DefaultRobotHWSim::controlInputCallback(const sensor_msgs::JointState & msg)
  {
    if(msg.name.size() != msg.position.size())
      {
        ROS_WARN("mujoco: size of actuator names and size of input is not same.");
      }
    for(int i = 0; i < msg.name.size(); i++)
      {
        const std::string actuator_name = qualifyName(msg.name.at(i));
        int actuator_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, actuator_name.c_str());
        if(actuator_id == -1)
          {
            ROS_WARN_STREAM("mujoco: joint name " <<  actuator_name << " does not exist");
          }
        else
          {
            control_input_.at(actuator_id) = msg.position.at(i);
          }
      }
  }

  void DefaultRobotHWSim::jointPositionCallback(const sensor_msgs::JointState & msg)
  {
    if(msg.name.size() != msg.position.size())
      {
        ROS_WARN("mujoco: size of actuator names and size of input is not same.");
        return;
      }

    direct_joint_position_ = msg;
  }

  void DefaultRobotHWSim::rootPoseCallback(const geometry_msgs::Pose & msg)
  {
    // check the validity of quaternion

    direct_root_pose_ = msg;
    direct_root_pose_flag_ = true;
  }

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::DefaultRobotHWSim, mujoco_ros_control::RobotHWSim)
