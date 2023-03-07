#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include<iostream>
using namespace std;
using namespace std::chrono_literals;
#define Z_BASE_LINK 1.79
#define Z_DESK 0.867
static const rclcpp::Logger LOGGER = rclcpp::get_logger("keyboard_control.interface.cpp");

// First we declare pointers to the node and publisher that will publish commands to Servo
rclcpp::Node::SharedPtr node_;

// Next we will set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_node", node_options);

  // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  //rclcpp::sleep_for(std::chrono::seconds(4));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }


  // Initializing Servo
  // ^^^^^^^^^^^^^^^^^^
  // Servo requires a number of parameters to dictate its behavior. These can be read automatically by using the
  // :code:`makeServoParameters` helper function
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

  // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode, a ROS
  // service is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
  servo->start();
  //servo->setPaused(true);


  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}