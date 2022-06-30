
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <zavrsni/program2.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/timer.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;

namespace zavrsni
{
Command::Ptr Program2::addPointCloud()
{
  // Create octomap and add it to the local environment
  pcl::PointCloud<pcl::PointXYZ> full_cloud;
  double delta = 0.05;
  auto length = static_cast<int>(1 / delta);

  for (int x = 0; x < length; ++x)
    for (int y = 0; y < length; ++y)
      for (int z = 0; z < length; ++z)
        full_cloud.push_back(pcl::PointXYZ(-0.5F + static_cast<float>(x * delta),
                                           -0.5F + static_cast<float>(y * delta),
                                           -0.5F + static_cast<float>(z * delta)));

  //  sensor_msgs::PointCloud2 pointcloud_msg;
  //  pcl::toROSMsg(full_cloud, pointcloud_msg);

  //  octomap::Pointcloud octomap_data;
  //  octomap::pointCloud2ToOctomap(pointcloud_msg, octomap_data);
  //  std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(2 * delta);
  //  octree->insertPointCloud(octomap_data, octomap::point3d(0, 0, 0));

  // Add octomap to environment
  Link link_octomap("octomap_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(1, 0, 0);
  visual->geometry =
      std::make_shared<tesseract_geometry::Octree>(full_cloud, 2 * delta, tesseract_geometry::Octree::BOX, true);
  link_octomap.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_octomap.collision.push_back(collision);

  Joint joint_octomap("joint_octomap_attached");
  joint_octomap.parent_link_name = "base_link";
  joint_octomap.child_link_name = link_octomap.getName();
  joint_octomap.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_octomap, joint_octomap);
}

Program2::Program2(tesseract_environment::Environment::Ptr env,
                                             tesseract_visualization::Visualization::Ptr plotter,
                                             bool ifopt,
                                             bool debug)
  : Example(std::move(env), std::move(plotter)), ifopt_(ifopt), debug_(debug)
{
}

tesseract_common::JointTrajectory Program2::run()
{
  // Create octomap and add it to the local environment
  //Command::Ptr cmd = addPointCloud();
  //if (!env_->applyCommand(cmd)){
  //  tesseract_common::JointTrajectory empty_traj;
  //  return empty_traj;
  //}

  if (plotter_ != nullptr)
    plotter_->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("joint_1");
  joint_names.emplace_back("joint_2");
  joint_names.emplace_back("joint_3");
  joint_names.emplace_back("joint_4");
  joint_names.emplace_back("joint_5");
  joint_names.emplace_back("joint_6");
  joint_names.emplace_back("joint_7");



  Eigen::VectorXd zero_pos(7);
  zero_pos(0) = 0.73;
  zero_pos(1) = 0.53;
  zero_pos(2) = 0.0;
  zero_pos(3) = 1.75;
  zero_pos(4) = 0.066;
  zero_pos(5) = 0.806;
  zero_pos(6) = -0.06;

  env_->setState(joint_names, zero_pos);

  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Create Program
  CompositeInstruction program(
      "cartesian_program", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool_frame")); //promjena

  // Start Joint Position for the program
  Waypoint wp0 = StateWaypoint(joint_names, zero_pos);

  Eigen::Isometry3d matrica1;
  matrica1.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  matrica1.translation() = Eigen::Vector3d(0.2, -0.2, 0.5);  // Offset for the table
  Waypoint wp1 = CartesianWaypoint(matrica1);

  Eigen::Isometry3d matrica12;
  matrica12.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  matrica12.translation() = Eigen::Vector3d(0.4, -0.2, 0.5);  // Offset for the table
  Waypoint wp12 = CartesianWaypoint(matrica12);

  Eigen::Isometry3d matrica3;
  matrica3.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  matrica3.translation() = Eigen::Vector3d(0.4, -0.3, 0.5);  // Offset for the table
  Waypoint wp3 = CartesianWaypoint(matrica3);

  Eigen::Isometry3d matrica34;
  matrica34.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  matrica34.translation() = Eigen::Vector3d(0.2, -0.3, 0.5);  // Offset for the table
  Waypoint wp34 = CartesianWaypoint(matrica34);

  Eigen::Isometry3d matrica5;
  matrica5.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  matrica5.translation() = Eigen::Vector3d(0.2, -0.4, 0.5);  // Offset for the table
  Waypoint wp5 = CartesianWaypoint(matrica5);

  Eigen::Isometry3d matrica56;
  matrica56.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  matrica56.translation() = Eigen::Vector3d(0.4, -0.4, 0.5);  // Offset for the table
  Waypoint wp56 = CartesianWaypoint(matrica56);

  Waypoint wp7 = StateWaypoint(joint_names, zero_pos);


  PlanInstruction start_instruction(wp0, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  PlanInstruction plan_1(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
  plan_1.setDescription("to_start");

  PlanInstruction plan_2(wp12, PlanInstructionType::LINEAR, "LINEAR");
  plan_2.setDescription("grid");

  PlanInstruction plan_3(wp3, PlanInstructionType::FREESPACE, "FREESPACE");
  plan_3.setDescription("grid");

  PlanInstruction plan_4(wp34, PlanInstructionType::LINEAR, "LINEAR");
  plan_4.setDescription("grid");

  PlanInstruction plan_5(wp5, PlanInstructionType::FREESPACE, "FREESPACE");
  plan_5.setDescription("grid");

  PlanInstruction plan_6(wp56, PlanInstructionType::LINEAR, "LINEAR");
  plan_6.setDescription("grid");

  PlanInstruction plan_7(wp7, PlanInstructionType::FREESPACE, "FREESPACE");
  plan_7.setDescription("to_end");

  program.push_back(plan_1);
  program.push_back(plan_2);
  program.push_back(plan_3);
  program.push_back(plan_4);
  program.push_back(plan_5);
  program.push_back(plan_6);
  program.push_back(plan_7);

  CONSOLE_BRIDGE_logInform("basic cartesian plan example");

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  if (ifopt_)
  {
    auto composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    composite_profile->collision_cost_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_constraint_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false; //ovo je bilo false prije
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    planning_server.getProfiles()->addProfile<TrajOptIfoptCompositeProfile>(
        profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "cartesian_program", composite_profile);

    auto plan_profile = std::make_shared<TrajOptIfoptDefaultPlanProfile>();
    plan_profile->cartesian_coeff = Eigen::VectorXd::Ones(6);
    plan_profile->joint_coeff = Eigen::VectorXd::Ones(7);
    planning_server.getProfiles()->addProfile<TrajOptIfoptPlanProfile>(
        profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "RASTER", plan_profile);
    planning_server.getProfiles()->addProfile<TrajOptIfoptPlanProfile>(
        profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "freespace_profile", plan_profile);

    request.name = process_planner_names::TRAJOPT_IFOPT_PLANNER_NAME;
  }
  else
  {
    auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    composite_profile->collision_cost_config.enabled = true;
    composite_profile->collision_constraint_config.enabled = true;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;   //bilo je false
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    planning_server.getProfiles()->addProfile<TrajOptCompositeProfile>(
        profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "cartesian_program", composite_profile);

    auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
    plan_profile->cartesian_coeff = Eigen::VectorXd::Ones(6);
    plan_profile->joint_coeff = Eigen::VectorXd::Ones(7);
    planning_server.getProfiles()->addProfile<TrajOptPlanProfile>(
        profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "RASTER", plan_profile);
    planning_server.getProfiles()->addProfile<TrajOptPlanProfile>(
        profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "freespace_profile", plan_profile);

    request.name = process_planner_names::TRAJOPT_PLANNER_NAME;
  }
  request.instructions = Instruction(program);

  // Print Diagnostics
  request.instructions.print("Program: ");

  if (plotter_ != nullptr && plotter_->isConnected())
    plotter_->waitForInput("Hit Enter to solve for trajectory.");
  std::cout << "tu sam1\n";
  // Solve process plan
  tesseract_common::Timer stopwatch;
  stopwatch.start();
  std::cout << "tu sam2\n";
  ProcessPlanningFuture response = planning_server.run(request);
  std::cout << "tu sam3\n";
  planning_server.waitForAll();
  std::cout << "tu sam4\n";
  stopwatch.stop();
  CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());

  tesseract_common::JointTrajectory trajectory;
  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    const auto& ci = response.problem->results->as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  return trajectory;
}

}  // namespace zavrsni
