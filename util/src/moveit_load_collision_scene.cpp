// Mabel Zhang
// 30 Jan 2018
//
// ROS service to load a plain-text .scene file into MoveIt planning collision
//   scene.
//
// Example usage:
//   The launch file puts this node in the /iiwa namespace, which the 
//     robot_description_semantic rosparam and apply_planning_scene rosservice
//     are in.
//   $ roslaunch gym_gazebo moveit_load_collision_scene.launch
//
// Ref PlanningScene API: http://docs.ros.org/kinetic/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html
// Planning scene tutorial code https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pr2_tutorials/planning/src/planning_scene_tutorial.cpp
// It says recommended way is to instantiate PlanningSceneMonitor, not PlanningScene directly, but I could not instantiate PlanningSceneMonitor correctly in ROS Kinetic, see include/util/moveit_collision_util_interactive.h.
//

#include <iostream>  // std::ios, std::istream, std::cout
#include <fstream>  // std::filebuf

#include <ros/ros.h>

// MoveIt ROS API
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "moveit_load_collision_scene");
  ros::NodeHandle nh;

  // Parse cmd line args
  // Example: "/home/master/graspingRepo/task_grasping/catkin_ws/src/gym_gazebo/config/iiwa_perch.scene"
  std::string scene_name = "";
  if (argc > 1)
  {
    scene_name = argv [1];
  }
  else
  {
    ROS_ERROR ("ERROR: scene_name not specified on command line. Must specify a plain-text scene file as input.");
    ROS_ERROR ("Usage: rosrun util moveit_load_collision_scene file.scene");
    return 0;
  }


  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);


  printf ("Loading collision scene objects from %s\n", scene_name.c_str ());

  // Ref: http://www.cplusplus.com/reference/istream/istream/istream/
  std::filebuf fb;
  if (fb.open (scene_name, std::ios::in))
  {
    std::istream is (&fb);
    // Load collision scene from plain-text .scene file
    planning_scene.loadGeometryFromStream (is);
    fb.close ();
  }

  planning_scene.printKnownObjects (std::cout);


  // Package the loaded scene into a moveit_msgs msg
  moveit_msgs::PlanningScene scene_msg;
  planning_scene.getPlanningSceneMsg (scene_msg);

  printf ("Calling rosservice apply_planning_scene to add collision objects to MoveIt scene in ROS...\n");

  // Call rosservice
  // Ref MoveIt ROS API tutorial https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pr2_tutorials/planning/src/planning_scene_ros_api_tutorial.cpp
  ros::ServiceClient planning_scene_diff_client =
    nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  // and send the diffs to the planning scene via a service call:
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = scene_msg;
  planning_scene_diff_client.call(srv);

  printf ("Service returned.\n");


  return 0;
}

