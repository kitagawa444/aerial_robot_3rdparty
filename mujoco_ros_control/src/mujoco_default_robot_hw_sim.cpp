#include <mujoco_ros_control/mujoco_default_robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool DefaultRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    // // init 内で rosparam を読む例
    // model_nh.param("use_ros_control", use_ros_control_, true);
    // model_nh.param("use_control_input", use_control_input_, false);
    // model_nh.param("control_input_as_feedforward", control_input_as_feedforward_, false);
    // model_nh.param("publish_joint_states_manually", publish_joint_states_manually_, false);
    // model_nh.param("allow_direct_state_set", allow_direct_state_set_, true);

    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;

    actuator_id_list_.resize(0);
    int* actuator_trntype = mujoco_model_->actuator_trntype;
    int* actuator_trnid = mujoco_model_->actuator_trnid;

    control_input_.resize(mujoco_model_->nu);
    joint_list_.resize(0);

    // get joint names from mujoco model
    for(int i = 0; i < mujoco_model_->njnt; i++)
      {
        if(mujoco_model_->jnt_type[i] > 1)
          {
            joint_list_.push_back(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_JOINT, i));

            for(int j = 0; j < mujoco_model_->nu; j++) {
              if (actuator_trnid[2 * j] == i && actuator_trntype[j] == mjtTrn_::mjTRN_JOINT) {
                actuator_id_list_.push_back(j);
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
    joint_names_.clear();
    for (int j = 0; j < mujoco_model_->njnt; ++j)
      {
        if (mujoco_model_->jnt_type[j] > 1)
          {
            const char* jname = mj_id2name(mujoco_model_, mjOBJ_JOINT, j);
            if (jname) joint_names_.push_back(std::string(jname));
          }
      }

    const size_t nj = joint_names_.size();
    pos_.assign(nj, 0.0);
    vel_.assign(nj, 0.0);
    eff_.assign(nj, 0.0);
    cmd_eff_.assign(nj, 0.0);
    act_id_by_joint_idx_.assign(nj, -1);

    // 名前一致: joint 名と同名の actuator を探す
    for (size_t i = 0; i < nj; ++i)
      {
        int act_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, joint_names_[i].c_str());
        act_id_by_joint_idx_[i] = act_id; // 無ければ -1 のまま
        if (act_id < 0)
          {
            ROS_WARN_STREAM("No actuator found for joint " << joint_names_[i]
                            << ". This joint will be read-only.");
          }
      }

    // JointStateInterface 登録
    for (size_t i = 0; i < nj; ++i)
      {
        hardware_interface::JointStateHandle sh(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]);
        jnt_state_interface_.registerHandle(sh);
      }
    registerInterface(&jnt_state_interface_);

    // EffortJointInterface 登録（書き込み先は cmd_eff_）
    for (size_t i = 0; i < nj; ++i)
      {
        hardware_interface::JointHandle eh(jnt_state_interface_.getHandle(joint_names_[i]), &cmd_eff_[i]);
        effort_jnt_interface_.registerHandle(eh);
      }
    registerInterface(&effort_jnt_interface_);
    ROS_INFO_STREAM("Registered " << nj << " joints to EffortJointInterface.");


    return true;
  }

  void DefaultRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    // 1) ros_control 用バッファへ反映
    const mjtNum* qpos = mujoco_data_->qpos;
    const mjtNum* qvel = mujoco_data_->qvel;
    const mjtNum* qfrc_act = mujoco_data_->qfrc_actuator; // DOF ごとの一般化力

    for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        int j = mj_name2id(mujoco_model_, mjOBJ_JOINT, joint_names_[i].c_str());
        if (j < 0) continue;

        int qposadr = mujoco_model_->jnt_qposadr[j];
        int dofadr  = mujoco_model_->jnt_dofadr[j];

        pos_[i] = qpos[qposadr];
        vel_[i] = qvel[dofadr];
        eff_[i] = qfrc_act[dofadr];
      }

    // 2) 自前 joint_states を出す場合（joint_state_controller を使わない時のみ）
    if (publish_joint_states_manually_)
      {
        if ((time - last_joint_state_time_).toSec() >= joint_state_pub_rate_)
          {
            sensor_msgs::JointState js;
            js.header.stamp = time;
            js.name = joint_names_;
            js.position = pos_;
            js.velocity = vel_;
            js.effort   = eff_;
            joint_state_pub_.publish(js);
            last_joint_state_time_ = time;
          }
      }
  }

  static inline double clip(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}


void DefaultRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
{
  // 1) ctrl を決定
  //    - use_ros_control_ が true の場合: cmd_eff_ を使用
  //    - use_control_input_ が true の場合:
  //        * control_input_as_feedforward_ なら cmd_eff_ に加算
  //        * そうでなければ control_input_ を単独で使用（旧経路優先）
  for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      int act = act_id_by_joint_idx_[i];
      if (act < 0) continue;

      double u = 0.0;
      if (use_ros_control_) u += cmd_eff_[i];
      if (use_control_input_)
        {
          if (control_input_as_feedforward_) u += (act < (int)control_input_.size() ? control_input_[act] : 0.0);
          else                               u  = (act < (int)control_input_.size() ? control_input_[act] : 0.0);
        }

      double lo = -std::numeric_limits<double>::infinity();
      double hi =  std::numeric_limits<double>::infinity();
      if (mujoco_model_->actuator_ctrllimited[act])
        {
          lo = mujoco_model_->actuator_ctrlrange[2*act+0];
          hi = mujoco_model_->actuator_ctrlrange[2*act+1];
        }
      mujoco_data_->ctrl[act] = clip(u, lo, hi);
    }

  // 2) 直接書き換え（テレポート）。頻繁にやると数値的に不安定になるので注意
  if (allow_direct_state_set_)
    {
      bool teleported = false;

      // root pose
      if (direct_root_pose_flag_)
        {
          mjtNum* qpos = mujoco_data_->qpos;
          switch (mujoco_model_->jnt_type[0]) {
          case mjJNT_FREE:
            qpos[0] = direct_root_pose_.position.x;
            qpos[1] = direct_root_pose_.position.y;
            qpos[2] = direct_root_pose_.position.z;
            qpos[3] = direct_root_pose_.orientation.w;
            qpos[4] = direct_root_pose_.orientation.x;
            qpos[5] = direct_root_pose_.orientation.y;
            qpos[6] = direct_root_pose_.orientation.z;
            teleported = true;
            break;
          case mjJNT_BALL:
            qpos[0] = direct_root_pose_.orientation.w;
            qpos[1] = direct_root_pose_.orientation.x;
            qpos[2] = direct_root_pose_.orientation.y;
            qpos[3] = direct_root_pose_.orientation.z;
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
              int jid = mj_name2id(mujoco_model_, mjOBJ_JOINT, names[k].c_str());
              if (jid < 0) { ROS_WARN_STREAM("mujoco: joint name " << names[k] << " does not exist"); continue; }

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
        int actuator_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, msg.name.at(i).c_str());
        if(actuator_id == -1)
          {
            ROS_WARN_STREAM("mujoco: joint name " <<  msg.name.at(i) << " does not exist");
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
