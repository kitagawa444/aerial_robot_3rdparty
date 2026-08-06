#include <mujoco_ros_control/mujoco_ros_control.h>

namespace mujoco_ros_control
{
  MujocoRosControl::MujocoRosControl(ros::NodeHandle &nh, ros::NodeHandle &nhp):
    nh_(nh), nhp_(nhp)
  {
  }

  MujocoRosControl::~MujocoRosControl()
  {
    mj_deleteData(mujoco_data_);
    mj_deleteModel(mujoco_model_);
  }

  bool MujocoRosControl::init()
  {
    std::string xml_path;
    nhp_.getParam("mujoco_model_path", xml_path);
    nhp_.param("headless", headless_, false);
    nhp_.param("render_fps", render_fps_, 30.0);
    nhp_.param("vsync", vsync_, false);
    nhp_.param("render_shadows", render_shadows_, false);
    nhp_.param("render_reflections", render_reflections_, false);
    nhp_.param("window_width", window_width_, 960);
    nhp_.param("window_height", window_height_, 720);

    if(render_fps_ <= 0.0)
      {
        ROS_WARN("render_fps must be positive; use 30 Hz");
        render_fps_ = 30.0;
      }
    if(window_width_ <= 0 || window_height_ <= 0)
      {
        ROS_WARN("MuJoCo window dimensions must be positive; use 960x720");
        window_width_ = 960;
        window_height_ = 720;
      }

    if(!nhp_.getParam("mujoco_model_path", xml_path))
      {
        ROS_INFO("Could not get xml path from rosparam\n");
        return false;
      }

    const char* xml_path_char = xml_path.c_str();

    char error[1000];
    mujoco_model_ = mj_loadXML(xml_path_char, NULL, error, 1000);
    if(!mujoco_model_)
      {
        ROS_INFO("Could not load mujoco model with error %s.\n", error);
        return false;
      }

    mujoco_model_->opt.timestep = 1.0 / clock_pub_freq_;

    mujoco_data_ = mj_makeData(mujoco_model_);
    if(!mujoco_data_)
      {
        ROS_INFO("Could not create mujoco data from model\n");
        return false;
      }
    else
      {
        ROS_INFO_STREAM("Created mujoco model from " << xml_path);
      }

    /* hardware interface  */
    robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim>("mujoco_ros_control", "mujoco_ros_control::RobotHWSim"));

    const bool use_model_name_prefixes =
      nhp_.getParam("robot_namespaces", robot_namespaces_) && !robot_namespaces_.empty();
    if(!use_model_name_prefixes)
      {
        // Legacy single-model mode:
        // Keep the node's existing ROS namespace for parameters, topics and the
        // controller manager, but do not interpret that namespace as a MuJoCo
        // model-name prefix.  For example, a node launched in /world must keep
        // using /world/simulation and must manage joints such as black_* rather
        // than looking for non-existent world_* joints.
        robot_namespaces_.clear();
        robot_namespaces_.push_back("");
      }

    robot_hw_sims_.clear();
    controller_managers_.clear();
    try
      {
        for(size_t i = 0; i < robot_namespaces_.size(); i++)
          {
            const std::string& robot_ns = robot_namespaces_[i];
            ros::NodeHandle robot_nh = robot_ns.empty() ? nh_ : ros::NodeHandle(nh_, robot_ns);
            ros::NodeHandle simulation_nh(robot_nh, "simulation");
            std::string plugin_name;
            simulation_nh.param("robot_hw_sim_plugin_name", plugin_name, std::string("mujoco_ros_control/DefaultRobotHWSim"));
            ROS_INFO_STREAM("Creating MuJoCo robot interface for namespace '"
                            << robot_nh.getNamespace()
                            << "' with plugin '" << plugin_name << "'");

            boost::shared_ptr<mujoco_ros_control::RobotHWSim> robot_hw_sim = robot_hw_sim_loader_->createInstance(plugin_name);
            ROS_INFO_STREAM("Created MuJoCo robot plugin instance for namespace '"
                            << robot_nh.getNamespace()
                            << "'");
            if(!robot_hw_sim->init(robot_ns, robot_nh, mujoco_model_, mujoco_data_))
              {
                ROS_ERROR_STREAM("Failed to initialize MuJoCo robot interface for namespace '"
                                 << robot_nh.getNamespace()
                                 << "' with plugin '" << plugin_name << "'");
                return false;
              }
            robot_hw_sims_.push_back(robot_hw_sim);
            controller_managers_.push_back(boost::shared_ptr<controller_manager::ControllerManager>(new controller_manager::ControllerManager(robot_hw_sim.get(), robot_nh)));
            ROS_INFO_STREAM("Initialized MuJoCo robot interface for namespace '"
                            << robot_nh.getNamespace()
                            << "' with plugin '" << plugin_name << "'");
          }
      }
    catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        return false;
      }

    clock_pub_ =  nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);

    return true;
  }


  void MujocoRosControl::publishSimTime()
  {
    ros::Time sim_time = (ros::Time) mujoco_data_->time;
    if((sim_time - last_clock_pub_time_).toSec() < 1.0 /(double) clock_pub_freq_ )
      {
        return;
      }
    ros::Time current_time = (ros::Time) mujoco_data_->time;
    rosgraph_msgs::Clock ros_time;
    ros_time.clock.fromSec(current_time.toSec());
    last_clock_pub_time_ = sim_time;
    clock_pub_.publish(ros_time);
  }

  void MujocoRosControl::update()
  {
    publishSimTime();

    ros::Time sim_time = (ros::Time) mujoco_data_->time;
    ros::Time sim_time_ros(sim_time.sec, sim_time.nsec);

    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

    mj_step1(mujoco_model_, mujoco_data_);

    for(size_t i = 0; i < robot_hw_sims_.size(); i++)
      {
        robot_hw_sims_[i]->read(sim_time_ros, sim_period);
      }

    for(size_t i = 0; i < controller_managers_.size(); i++)
      {
        controller_managers_[i]->update(sim_time_ros, sim_period);
      }

    for(size_t i = 0; i < robot_hw_sims_.size(); i++)
      {
        robot_hw_sims_[i]->write(sim_time_ros, sim_period);
      }

    mj_step2(mujoco_model_, mujoco_data_);

    last_update_sim_time_ros_ = sim_time_ros;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mujoco_ros_control");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  mujoco_ros_control::MujocoRosControl mujoco_ros_control(nh, nhp);

  if(!mujoco_ros_control.init()) return 1;

  bool headless = mujoco_ros_control.headless_;

  // viewer definition
  MujocoVisualizationUtils &mujoco_visualization_utils = MujocoVisualizationUtils::getInstance();
  GLFWwindow* window;

  if(!headless)
    {
      // Rendering runs on the simulation thread. Under WSLg, reducing render
      // frequency and disabling v-sync prevents buffer presentation from
      // stalling physics and controller updates.
      if(!glfwInit())
        {
          ROS_ERROR("Failed to initialize GLFW");
          return 1;
        }
      window = glfwCreateWindow(mujoco_ros_control.window_width_,
                                mujoco_ros_control.window_height_,
                                "MuJoCo ROS Control", NULL, NULL);
      if(!window)
        {
          ROS_ERROR("Failed to create MuJoCo GLFW window");
          glfwTerminate();
          return 1;
        }
      glfwMakeContextCurrent(window);
      glfwSwapInterval(mujoco_ros_control.vsync_ ? 1 : 0);

      ROS_INFO("MuJoCo viewer: %.1f renders/sim-second, vsync=%s, shadows=%s, reflections=%s, window=%dx%d",
               mujoco_ros_control.render_fps_,
               mujoco_ros_control.vsync_ ? "on" : "off",
               mujoco_ros_control.render_shadows_ ? "on" : "off",
               mujoco_ros_control.render_reflections_ ? "on" : "off",
               mujoco_ros_control.window_width_,
               mujoco_ros_control.window_height_);

      // make context current
      glfwMakeContextCurrent(window);

      // initialize mujoco visualization functions
      mujoco_visualization_utils.init(mujoco_ros_control.mujoco_model_,
                                      mujoco_ros_control.mujoco_data_, window,
                                      mujoco_ros_control.render_shadows_,
                                      mujoco_ros_control.render_reflections_);
    }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
    {
      bool paused = false;
      if (!headless)
        {
          paused = mujoco_visualization_utils.isPaused();
        }

      if (!paused)
        {
          mjtNum sim_start = mujoco_ros_control.mujoco_data_->time;
          while(mujoco_ros_control.mujoco_data_->time - sim_start <
                1.0 / mujoco_ros_control.render_fps_ && ros::ok())
            {
              mujoco_ros_control.update();
            }
        }

      if(!headless)
        {
          mujoco_visualization_utils.update(window);

          if (glfwWindowShouldClose(window))
            {
              break;
            }
        }
    }

  if(!headless) mujoco_visualization_utils.terminate();
  return 0;
}
