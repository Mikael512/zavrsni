#include <zavrsni/program3.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace zavrsni;
using namespace tesseract_rosutils;
using namespace tesseract_common;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "/my_gen3/robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "/my_gen3/robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "zavrsni";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "program3_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  tesseract_common::fs::path urdf_path =
      locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.urdf")->getFilePath();
  tesseract_common::fs::path srdf_path =
      locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.srdf")->getFilePath();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator))
    exit(1);


  // ovaj dio koda je iz extra nodea
  //auto locator = std::make_shared<TesseractSupportResourceLocator>();
  //tesseract_common::fs::path urdf_path =
  //    locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.urdf")->getFilePath();
  //tesseract_common::fs::path srdf_path =
  //    locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.srdf")->getFilePath();
  //auto env = std::make_shared<Environment>();
  //if (!env->init(urdf_path, srdf_path, locator))
  //  exit(1);
//
  //Program3 example(env, nullptr);
  //if (!example.run())
  //  exit(1);
  //gotov dio koda iz extra nodeas


  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz)
    monitor->startPublishingEnvironment();

  tesseract_common::JointTrajectory trajectory;
  ROSPlottingPtr plotter;
  if (plotting)
    plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());

  Program3 example(env, plotter);
  trajectory = example.run();
}
