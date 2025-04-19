/**
 * @file hello_moveit.cpp
 * @brief A basic ROS 2 and MoveIt 2 program to control a robot arm
 *
 * This program demonstrates how to use ROS 2 and MoveIt 2 to control a robot arm.
 * It sets up a node, creates a MoveGroupInterface for the arm, sets a target pose for the gripper_base,
 * plans a trajectory, and executes the planned motion.
 *
 * @author Addison Sears-Collins
 * @date December 15, 2024
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>

/**
 * @brief The main function that starts our program.
 *
 * This function sets up our ROS 2 environment and prepares it for robot control.
 *
 * @param argc The number of input arguments our program receives.
 * @param argv The list of input arguments our program receives.
 * @return int A number indicating if our program finished successfully (0) or not.
 */
int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Creates a node named "hello_moveit". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interfaces
  // These interfaces are used to plan and execute movements, set target poses,
  // and perform other motion-related tasks for each respective part of the robot.
  // The use of auto allows the compiler to automatically deduce the type of variable.
  // Source: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");

  // Specify a planning pipeline to be used for further planning
  arm_group_interface.setPlanningPipelineId("ompl");

  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectGugukConfigDefault");

  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(1.0);

  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(1.0);

  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());


  // // 1. Set a target pose for the end effector of the arm（gtipper_base）
  // auto const arm_target_pose = [&node]{
  //   geometry_msgs::msg::PoseStamped msg;
  //   msg.header.frame_id = "base_link";
  //   msg.header.stamp = node->now();
  //   msg.pose.position.x = 0.061;
  //   msg.pose.position.y = -0.176;
  //   msg.pose.position.z = 0.168;
  //   msg.pose.orientation.x = 1.0;
  //   msg.pose.orientation.y = 0.0;
  //   msg.pose.orientation.z = 0.0;
  //   msg.pose.orientation.w = 0.0;
  //   return msg;
  // }();
  // arm_group_interface.setPoseTarget(arm_target_pose);

  // // 2. Set a joint space target
  // std::vector<double> joint_group_positions = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
  // bool within_bounds = arm_group_interface.setJointValueTarget(joint_group_positions);
  // if (!within_bounds)
  // {
  //   RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  // }

  // 3. Plan around object
  // 3.1 Set a target pose for the end effector of the arm
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.25958;
    msg.pose.position.y = -0.11054;
    msg.pose.position.z = 0.10749;
    msg.pose.orientation.x = -0.33038;
    msg.pose.orientation.y = 0.34634;
    msg.pose.orientation.z = -0.33997;
    msg.pose.orientation.w = 0.80951;
    return msg;
  }();
  arm_group_interface.setPoseTarget(arm_target_pose);

  // // 3.2 Create collision object for the robot to avoid
  // auto const collision_object = [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {

  //   // Print the planning frame for debugging purposes
  //   RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

  //   // Initialize a CollisionObject message
  //   moveit_msgs::msg::CollisionObject collision_object;

  //   // Set the frame ID for the collision object
  //   collision_object.header.frame_id = frame_id;

  //   // Set the timestamp to the current time
  //   collision_object.header.stamp = node->now();

  //   // Assign a unique ID to the collision object
  //   collision_object.id = "box1";

  //   // Define the shape of the collision object as a box
  //   shape_msgs::msg::SolidPrimitive primitive;
  //   primitive.type = primitive.BOX;
  //   primitive.dimensions.resize(3);

  //   // Set the dimensions of the box (in meters)
  //   primitive.dimensions[primitive.BOX_X] = 0.2;  // Width
  //   primitive.dimensions[primitive.BOX_Y] = 0.05;  // Depth
  //   primitive.dimensions[primitive.BOX_Z] = 0.50;  // Height

  //   // Define the pose (position and orientation) of the box
  //   geometry_msgs::msg::Pose box_pose;

  //   // Set the position of the box center
  //   box_pose.position.x = 0.25;  // meters in x-direction
  //   box_pose.position.y = 0.0;   // Centered in y-direction
  //   box_pose.position.z = 0.25;  // meters in z-direction

  //   // Set the orientation of the box (no rotation in this case)
  //   box_pose.orientation.x = 0.0;
  //   box_pose.orientation.y = 0.0;
  //   box_pose.orientation.z = 0.0;
  //   box_pose.orientation.w = 1.0;

  //   // Add the shape and pose to the collision object
  //   collision_object.primitives.push_back(primitive);
  //   collision_object.primitive_poses.push_back(box_pose);

  //   // Set the operation to add the object to the planning scene
  //   collision_object.operation = collision_object.ADD;

  //   // Log information about the created collision object
  //   RCLCPP_INFO(logger, "Created collision object: %s", collision_object.id.c_str());

  //   RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f",
  //     primitive.dimensions[primitive.BOX_X],
  //     primitive.dimensions[primitive.BOX_Y],
  //     primitive.dimensions[primitive.BOX_Z]);

  //   RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
  //     box_pose.position.x, box_pose.position.y, box_pose.position.z);

  //   // Return the fully defined collision object
  //   return collision_object;
  // }();

  // // 3.3 Set up a virtual representation of the robot's environment
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // // 3.4 Add an object to this virtual environment that the robot needs to avoid colliding with
  // planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  // This will give us two things:
  // 1. Whether the planning was successful (stored in 'success')
  // 2. The actual motion plan (stored in 'plan')
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  std::string s;
  std::cout << "Press any key to execute the plan: ";
  std::cin >> s;
  // Try to execute the movement plan if it was created successfully
  // If the plan wasn't successful, report an error
  // Execute the plan
  if (success)
  {
    arm_group_interface.execute(plan);
  }
    else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  
  rclcpp::spin(node);
  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}