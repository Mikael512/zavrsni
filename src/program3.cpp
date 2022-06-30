#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <zavrsni/program3.h>
#include <tesseract_environment/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/types.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;

namespace zavrsni
{
tesseract_common::VectorIsometry3d 
Program3::makePuzzleToolPoses(const tesseract_common::ResourceLocator::ConstPtr& locator)
{
  tesseract_common::VectorIsometry3d path;  // results
  std::ifstream indata;                     // input file

  // You could load your parts from anywhere, but we are transporting them with
  // the git repo
  auto resource = locator->locateResource("package://tesseract_support/urdf/puzzle_bent.csv");

  // In a non-trivial app, you'll of course want to check that calls like 'open'
  // succeeded
  indata.open(resource->getFilePath());

  std::string line;
  int lnum = 0;
  while (std::getline(indata, line))
  {
    ++lnum;
    if (lnum < 3)
      continue;

    std::stringstream lineStream(line);
    std::string cell;
    Eigen::Matrix<double, 6, 1> xyzijk;
    int i = -2;
    while (std::getline(lineStream, cell, ','))
    {
      ++i;
      if (i == -1)
        continue;

      xyzijk(i) = std::stod(cell);
    }

    Eigen::Vector3d pos = xyzijk.head<3>();
    pos = pos / 1000.0;  // Most things in ROS use meters as the unit of length.
                         // Our part was exported in mm.
    Eigen::Vector3d norm = xyzijk.tail<3>();
    norm.normalize();

    // This code computes two extra directions to turn the normal direction into
    // a full defined frame. Descartes
    // will search around this frame for extra poses, so the exact values do not
    // matter as long they are valid.
    Eigen::Vector3d temp_x = (-1 * pos).normalized();
    Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
    Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
    Eigen::Isometry3d pose;
    pose.matrix().col(0).head<3>() = x_axis;
    pose.matrix().col(1).head<3>() = y_axis;
    pose.matrix().col(2).head<3>() = norm;
    pose.matrix().col(3).head<3>() = pos;

    path.push_back(pose);
  }
  indata.close();

  return path;
}

Program3::Program3(tesseract_environment::Environment::Ptr env,
                                       tesseract_visualization::Visualization::Ptr plotter)
  : Example(std::move(env), std::move(plotter))
{
}

tesseract_common::JointTrajectory Program3::run()
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

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

  Eigen::VectorXd joint_pos(7);
  joint_pos(0) = -0.785398;
  joint_pos(1) = 0.4;
  joint_pos(2) = 0.0;
  joint_pos(3) = -1.9;
  joint_pos(4) = 0.0;
  joint_pos(5) = 1.0;
  joint_pos(6) = 0.0;

  env_->setState(joint_names, joint_pos);

  // Get Tool Poses
  tesseract_common::VectorIsometry3d tool_poses = makePuzzleToolPoses(env_->getResourceLocator());

  // Create manipulator information for program
  ManipulatorInfo mi;
  mi.manipulator = "manipulator";
  mi.working_frame = "part";
  mi.tcp_frame = "grinder_frame";

  // Create Program
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, mi);

  // Create cartesian waypoint
  Waypoint wp = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wp, PlanInstructionType::START, "CARTESIAN");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  for (std::size_t i = 1; i < tool_poses.size(); ++i)
  { 
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "CARTESIAN");
    plan_instruction.setDescription("waypoint_" + std::to_string(i));
    program.push_back(plan_instruction);
  }
  std::cout << "tu sam1\n";
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 5);
  planning_server.loadDefaultProcessPlanners();
  std::cout << "tu sam2\n";
  // Create a trajopt taskflow without post collision checking
  /** @todo This matches the original example, but should update to include post collision check */
  const std::string new_planner_name = "TRAJOPT_NO_POST_CHECK";
  planning_server.registerProcessPlanner(new_planner_name, createTrajOptGenerator(true, false));
  std::cout << "tu sam3\n";
  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);
  trajopt_plan_profile->cartesian_coeff(5) = 0;
  std::cout << "tu sam4\n";
  auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.enabled = true;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.025;
  trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  trajopt_composite_profile->collision_cost_config.coeff = 20;
  std::cout << "tu sam5\n";
  auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_solver_profile->opt_info.max_iter = 200;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
  std::cout << "tu sam6\n";
  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<TrajOptPlanProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile);
  planning_server.getProfiles()->addProfile<TrajOptCompositeProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<TrajOptSolverProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);
  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = new_planner_name;
  request.instructions = Instruction(program);
  std::cout << "tu sam7\n";
  // Create Naive Seed
  /** @todo Need to improve simple planners to support external tcp definitions */
  CompositeInstruction naive_seed;
  {
    auto lock = env_->lockRead();
    naive_seed = generateNaiveSeed(program, *env_);
  }
  request.seed = Instruction(naive_seed);
  std::cout << "tu sam8\n";
  // Print Diagnostics
  request.instructions.print("Program: ");
  request.seed.print("Seed: ");
  std::cout << "tu sam9\n";
  if (plotter_ != nullptr)
    plotter_->waitForInput();
  std::cout << "tu sam10\n";
  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();
  std::cout << "tu sam11\n";

  tesseract_common::JointTrajectory trajectory;
  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    const auto& ci = response.problem->results->as<CompositeInstruction>();
    trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  return trajectory;
}
}  // namespace tesseract_examples
