// Mabel Zhang
// 15 May 2017
//
// Tests functionality of moveit_collision_util_interactive.h.
//
// Usage:
//   Make sure MoveIt RViz is launched! Otherwise this file will hang waiting
//     for InteractiveMarkerServer and MoveIt to start!
//     e.g.
//     $ roslaunch iiwa_moveit moveit_planning_execution.launch model:=iiwa7
//     or whatever robot you have that has MoveIt interface, e.g. PR2, Baxter.
//   Then run
//     $ rosrun util moveit_collision_util_interactive
//     $ rosrun util test_moveit_collision_util_interactive
//

#include <ros/ros.h>

// Local
#include "util/moveit_collision_util_interactive.h"


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "test_moveit_collision_util_interactive");
  ros::NodeHandle nh;


  // Add collision box
  ros::ServiceClient colli_srv = nh.serviceClient
    <util::MoveItAddCollisionSrv> ("/util/add_collision_box_interactive");

  util::MoveItAddCollisionSrv colli_req;

  colli_req.request.pose_init.pose.position.x = 0.722;
  colli_req.request.pose_init.pose.position.y = 0.247;
  colli_req.request.pose_init.pose.position.z = -0.035;

  colli_req.request.pose_init.pose.orientation.x = 0;
  colli_req.request.pose_init.pose.orientation.y = 0;
  colli_req.request.pose_init.pose.orientation.z = 0;
  colli_req.request.pose_init.pose.orientation.w = 1;

  colli_req.request.dims_init.x = 0.30; //0.08;
  colli_req.request.dims_init.y = 0.20; //0.04;
  colli_req.request.dims_init.z = 0.10; //0.02;

  // Set to true to let user adjust via InteractiveMarker
  colli_req.request.ask_user_input = true;

  colli_req.request.adjust_orientation.resize (3);
  colli_req.request.adjust_orientation [0] = true;
  colli_req.request.adjust_orientation [1] = false;
  colli_req.request.adjust_orientation [2] = true;


  ros::Rate wait_rate (10);

  while (ros::ok ())
  {
    ROS_INFO ("Calling service to add collision box through InteractiveMarkers...");
    colli_srv.call (colli_req);
    if (! colli_req.response.success)
    {
      ROS_INFO ("Service call failed\n");
    }
    else
    {
      geometry_msgs::Pose pose = colli_req.response.pose_fin.pose;
      geometry_msgs::Vector3 dims = colli_req.response.dims_fin;
      ROS_INFO ("Service call succeeded. Object collision box pose: "
        "t %.2f %.2f %.2f, q %.2f %.2f %.2f %.2f, dims %.2f %.2f %.2f\n",
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        dims.x, dims.y, dims.z);
      break;
    }

    ros::spinOnce ();
    wait_rate.sleep ();
  }


  return 0;
}

