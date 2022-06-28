#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <zavrsni/program1.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/timer.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/default_planner_namespaces.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;

namespace zavrsni
{
Program1::Program1(tesseract_environment::Environment::Ptr env,
                                         tesseract_visualization::Visualization::Ptr plotter,
                                         bool ifopt,
                                         bool debug)
  : Example(std::move(env), std::move(plotter)), ifopt_(ifopt), debug_(debug)
{
}



tesseract_environment::Command::Ptr Program1::addSphere()
{
  // Add sphere to environment
  Link link_sphere("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, -0.2, 0.4);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.15);
  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "base_link";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}




tesseract_common::JointTrajectory Program1::run()
{

  //Add sphere to environment
  Command::Ptr cmd = addSphere();
  
  if (!env_->applyCommand(cmd))
  {
    tesseract_common::JointTrajectory empty_traj;
    return empty_traj;
  }
  
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

  Eigen::VectorXd joint_start_pos(7);
  joint_start_pos(0) = 0.73;
  joint_start_pos(1) = 0.53;
  joint_start_pos(2) = 0.0;
  joint_start_pos(3) = 1.75;
  joint_start_pos(4) = 0.066;
  joint_start_pos(5) = 0.806;
  joint_start_pos(6) = -0.06;

  Eigen::VectorXd joint_end_pos(7);
  joint_end_pos(0) = -0.07;
  joint_end_pos(1) = 0.77;
  joint_end_pos(2) = 0.0;
  joint_end_pos(3) = 1.38;
  joint_end_pos(4) = 0.02;
  joint_end_pos(5) = 0.96; //0.806
  joint_end_pos(6) = -0.06;

  env_->setState(joint_names, joint_start_pos);

  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Solve Trajectory
  CONSOLE_BRIDGE_logInform("primjer s kinovom koji radi :D");

  // Create Program
  CompositeInstruction program(
      "UPRIGHT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool_frame")); //ovdje sam promijenio "tool_frame"

  // Start and End Joint Position for the program
  Waypoint wp0 = StateWaypoint(joint_names, joint_start_pos);
  Waypoint wp1 = StateWaypoint(joint_names, joint_end_pos);

  PlanInstruction start_instruction(wp0, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  std::cout << joint_end_pos << "\n\n";
  
  // Plan freespace from start
  // Assign a linear motion so cartesian is defined as the target
  PlanInstruction plan_f0(wp1, PlanInstructionType::LINEAR, "UPRIGHT"); //ovo sam mijenjao iz linear upright
  plan_f0.setDescription("freespace_plan");

  // Add Instructions to program
  program.push_back(plan_f0);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  if (ifopt_)
  {
    std::cout << "using ifopt!\n\n";
    auto composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    composite_profile->collision_cost_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_cost_config->contact_manager_config = tesseract_collision::ContactManagerConfig(0.01);
    composite_profile->collision_cost_config->collision_margin_buffer = 0.01;
    composite_profile->collision_constraint_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_constraint_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.01);
    composite_profile->collision_constraint_config->collision_margin_buffer = 0.01;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    planning_server.getProfiles()->addProfile<TrajOptIfoptCompositeProfile>(
        profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "UPRIGHT", composite_profile);

    auto plan_profile = std::make_shared<TrajOptIfoptDefaultPlanProfile>();
    plan_profile->joint_coeff = Eigen::VectorXd::Ones(7);
    plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
    plan_profile->cartesian_coeff(0) = 0;
    plan_profile->cartesian_coeff(1) = 0;
    plan_profile->cartesian_coeff(2) = 0;
    planning_server.getProfiles()->addProfile<TrajOptIfoptPlanProfile>(
        profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "UPRIGHT", plan_profile);

    request.name = tesseract_planning::process_planner_names::TRAJOPT_IFOPT_PLANNER_NAME;
  }
  else
  {
    std::cout << "not using ifopt!\n\n";
    auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    composite_profile->collision_cost_config.enabled = true;
    composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
    composite_profile->collision_cost_config.safety_margin = 0.01;
    composite_profile->collision_cost_config.safety_margin_buffer = 0.01;
    composite_profile->collision_cost_config.coeff = 1;
    composite_profile->collision_constraint_config.enabled = true;
    composite_profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
    composite_profile->collision_constraint_config.safety_margin = 0.01;
    composite_profile->collision_constraint_config.safety_margin_buffer = 0.01;
    composite_profile->collision_constraint_config.coeff = 1;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    planning_server.getProfiles()->addProfile<TrajOptCompositeProfile>(
        profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UPRIGHT", composite_profile);

    auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
    plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
    plan_profile->cartesian_coeff(0) = 0;
    plan_profile->cartesian_coeff(1) = 0;
    plan_profile->cartesian_coeff(2) = 0;

    // Add profile to Dictionary
    planning_server.getProfiles()->addProfile<TrajOptPlanProfile>(
        profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UPRIGHT", plan_profile);

    request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  }
  request.instructions = Instruction(program);

  // Print Diagnostics
  request.instructions.print("Program: ");

  if (plotter_ != nullptr && plotter_->isConnected())
    plotter_->waitForInput("Hit Enter to solve for trajectory.");

  // Solve process plan
  tesseract_common::Timer stopwatch;
  stopwatch.start();
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();
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
  //return response.interface->isSuccessful();
  return trajectory;
}
}  // namespace zavrsni
