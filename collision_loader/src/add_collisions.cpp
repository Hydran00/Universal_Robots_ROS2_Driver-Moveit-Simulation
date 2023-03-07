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

// BEGIN_TUTORIAL

// Setup
// ^^^^^
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
  rclcpp::sleep_for(std::chrono::seconds(5));

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

  // Next we will create a collision object in the way of the arm. As the arm is servoed towards it, it will slow down
  // and stop before colliding
  moveit_msgs::msg::CollisionObject desk;
  moveit_msgs::msg::CollisionObject electric_panel;
  moveit_msgs::msg::CollisionObject wall;
  moveit_msgs::msg::CollisionObject top_plate;
  moveit_msgs::msg::CollisionObject plug;


  // define reference frame
  desk.header.frame_id = "base_link";
  electric_panel.header.frame_id = "base_link";
  wall.header.frame_id = "base_link";
  top_plate.header.frame_id = "base_link";
  plug.header.frame_id = "base_link";

  desk.id = "desk";
  electric_panel.id = "electric_panel";
  wall.id = "wall";
  top_plate.id = "top_plate";

  shape_msgs::msg::SolidPrimitive primitive1, primitive2, primitive3, primitive4, primitive5;
  // defining desk's collision
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.85;
  primitive1.dimensions[1] = 2.5;
  primitive1.dimensions[2] = 0.8675;
  // defining electric panel's collision
  primitive2.type = primitive2.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 0.175;
  primitive2.dimensions[1] = 1;
  primitive2.dimensions[2] = 0.175;
  // defining wall's collision
  primitive3.type = primitive3.BOX;
  primitive3.dimensions.resize(3);
  primitive3.dimensions[0] = 0.05;
  primitive3.dimensions[1] = 1;
  primitive3.dimensions[2] = 2;
  // defining top plate's collision
  primitive4.type = primitive4.BOX;
  // primitive4.type = primitive3.BOX;
  primitive4.dimensions.resize(3);
  primitive4.dimensions[0] = 0.5;
  primitive4.dimensions[1] = 1;
  primitive4.dimensions[2] = 0.05;
  // defining electric plug's collision
  primitive5.type = primitive5.BOX;
  primitive5.dimensions.resize(3);
  primitive5.dimensions[0] = 0.1;
  primitive5.dimensions[1] = 0.12;
  primitive5.dimensions[2] = 0.09;

  geometry_msgs::msg::Pose desk_pose, electric_panel_pose, wall_pose, top_plate_pose, plug_pose;
  // defining desk's pose
  desk_pose.orientation.w = -0.707;
  desk_pose.orientation.z = 0.707;
  desk_pose.position.x = -0.75;
  desk_pose.position.y = 0.08;
  desk_pose.position.z = Z_BASE_LINK - 0.867 / 2;
  // defining electric panel's pose
  electric_panel_pose.orientation.w = -0.707;
  electric_panel_pose.orientation.z = 0.707;
  electric_panel_pose.position.x = 0;
  electric_panel_pose.position.y = -0.255;
  electric_panel_pose.position.z = 0.92 - 0.075;
  // defining wall's pose
  wall_pose.orientation.z = 0.707;
  wall_pose.orientation.w = -0.707;
  wall_pose.position.x = 0;
  wall_pose.position.y = -0.375;
  wall_pose.position.z = 1;
  // defining top plate's pose
  top_plate_pose.orientation.z = 0.707;
  top_plate_pose.orientation.w = -0.707;
  top_plate_pose.position.x = 0;
  top_plate_pose.position.y = -0.15;
  top_plate_pose.position.z = -0.026;
  // defining plug's pose
  plug_pose.orientation.z = 0.707;
  plug_pose.orientation.w = -0.707;
  plug_pose.position.x = 0.44;
  plug_pose.position.y = -0.2;
  plug_pose.position.z = 0.92 - 0.06;

  desk.primitives.push_back(primitive1);
  desk.primitive_poses.push_back(desk_pose);
  desk.operation = desk.ADD;

  electric_panel.primitives.push_back(primitive2);
  electric_panel.primitive_poses.push_back(electric_panel_pose);
  electric_panel.operation = electric_panel.ADD;

  wall.primitives.push_back(primitive3);
  wall.primitive_poses.push_back(wall_pose);
  wall.operation = wall.ADD;

  top_plate.primitives.push_back(primitive4);
  top_plate.primitive_poses.push_back(top_plate_pose);
  top_plate.operation = top_plate.ADD;

  plug.primitives.push_back(primitive5);
  plug.primitive_poses.push_back(plug_pose);
  plug.operation = plug.ADD;

  // Create the message to publish the collision object
  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(desk);
  psw.collision_objects.push_back(electric_panel);
  psw.collision_objects.push_back(wall);
  psw.collision_objects.push_back(top_plate);
  psw.collision_objects.push_back(plug);
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw; 
  // Publish the collision object to the planning scene
  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  scene_pub->publish(ps);

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}