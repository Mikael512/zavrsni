#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <zavrsni/program2.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>

#include "moveit_msgs/RobotTrajectory.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace zavrsni;
using namespace tesseract_rosutils;

typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;


/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "/my_gen3/robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "/my_gen3/robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "zavrsni";


std::vector<double> eigen2std(Eigen::VectorXd input_vec)
{
  std::vector<double> return_vec;
  for(uint32_t i = 0; i < input_vec.rows(); i++)
  {
    return_vec.push_back(input_vec(i));
  }
  return return_vec;
}

moveit_msgs::RobotTrajectory tesseract2moveit(const tesseract_common::JointTrajectory &trajectory)
{
  moveit_msgs::RobotTrajectory moveit_traj;
  trajectory_msgs::JointTrajectory joint_traj;
  joint_traj.header.stamp = ros::Time::now();
  joint_traj.joint_names = trajectory.states[0].joint_names;

  auto joint_states = trajectory.states;
  for(auto joint_state : joint_states)
  {
    trajectory_msgs::JointTrajectoryPoint temp_point;
    temp_point.time_from_start = ros::Duration(joint_state.time);
    temp_point.positions = eigen2std(joint_state.position);
    temp_point.velocities = eigen2std(joint_state.velocity);
    temp_point.accelerations = eigen2std(joint_state.acceleration);
    //temp_point.effort = eigen2std(joint_state.effort);

    joint_traj.points.push_back(temp_point);
  }

  moveit_traj.joint_trajectory = joint_traj;
  return moveit_traj;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "program2_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /**
   *  Moveit stuff 
  */
  auto joint_model_group_name = "arm";
  auto robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
  auto kinematic_model = new robot_state::RobotModelPtr();
  *kinematic_model = robot_model_loader->getModel();

  std::cout << "Model frame: " << (*kinematic_model)->getModelFrame().c_str() << "\n";

  if (*kinematic_model) {
      auto joint_model_group = (*kinematic_model)->getJointModelGroup(joint_model_group_name);
      if (joint_model_group) {
          auto kinematic_state = new robot_state::RobotStatePtr(new robot_state::RobotState(*kinematic_model));
          (*kinematic_state)->setToDefaultValues();
      }
  }
  
  std::cout << "move_group planning interface init!\n";
  MoveGroupInterface::Options opt(joint_model_group_name, "robot_description");
  auto move_group = std::make_shared<MoveGroupInterface> (opt);
  std::cout << "init successfull!\n\n";

  move_group->getCurrentState();

  /**
   * End moveit stuff
  */

  bool plotting = true;
  bool rviz = true;
  bool ifopt = false;
  bool debug = false;


  // goto start joint position
  std::vector<double> start_joint_pos{  0.73,
                                        0.53,
                                        0.0,
                                        1.75,
                                        0.066,
                                        0.806,
                                        -0.06};

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  move_group->setJointValueTarget(start_joint_pos);
  move_group->setMaxAccelerationScalingFactor(1.0);
  move_group->setMaxVelocityScalingFactor(1.0);
  move_group->plan(plan);

  auto err = move_group->execute(plan);
  //sleep(3.0);
  ros::Duration(5.0).sleep();

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("ifopt", ifopt, ifopt);
  pnh.param("debug", debug, debug);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator))
    exit(1);

  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz)
    monitor->startPublishingEnvironment();

  ROSPlottingPtr plotter;
  if (plotting)
    plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());

  Program2 example(env, plotter, ifopt, debug);
  auto trajectory = example.run();
  plan.trajectory_ = tesseract2moveit(trajectory);
  move_group->execute(plan);
}
