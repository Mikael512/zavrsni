#include <zavrsni/program1.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <kortex_driver/PlayPreComputedJointTrajectory.h>
#include <kortex_driver/PreComputedJointTrajectory.h>
#include <kortex_driver/PreComputedJointTrajectoryElement.h>


using namespace zavrsni;
using namespace tesseract_rosutils;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "/my_gen3/robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "/my_gen3/robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "zavrsni";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "program1 node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  bool ifopt = false;
  bool debug = false;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("ifopt", ifopt, ifopt);
  pnh.param("debug", debug, debug);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  // service setup
  ros::ServiceClient client = nh.serviceClient<kortex_driver::PlayPreComputedJointTrajectory>("precomputed_traj");
  kortex_driver::PlayPreComputedJointTrajectory srv;
  kortex_driver::PreComputedJointTrajectory precomputed_trajectory;
  

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

  Program1 example(env, plotter, ifopt, debug);
  
  auto trajectory = example.run();
  auto joint_states = trajectory.states;
  std::cout << "trajectory size = " << joint_states.size() << "\n";

  for(auto joint_state : joint_states)
  { 
    
    kortex_driver::PreComputedJointTrajectoryElement traj_element;
    traj_element.joint_accelerations(joint_state.acceleration);
    traj_element.joint_angles(joint_state.position);
    traj_element.joint_speeds(joint_state.velocity);
    traj_element.time_from_start(joint_state.time);
    precomputed_trajectory.trajectory_elements.push_back(traj_element);
  }
  srv.request.input = precomputed_trajectory;

  if (client.call(srv))
  {
    
  }
  else
  {
    std::cout << "nesto nije uspjelo";
  }



}
