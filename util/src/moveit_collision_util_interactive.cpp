// Mabel Zhang
// 13 May 2017
//
// Runs the rosservice server handled in
//   ../include/util/moveit_collision_util_interactive.h
//
// Copied from my baxter_reflex moveit_planner.cpp.
// Refactored from ../include/util/interactive_marker_util.cpp,
//   so that other packages can simply include the header file,
//   and so that header file can be checked for compilation success by being
//   included in a .cpp file to be built by compiler.
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
//     or can also test with
//     $ rosrun util test_moveit_collision_util_interactive.py
//


// Local
#include "util/create_interactive_markers.h"
#include "util/interactive_marker_util.h"
#include "util/moveit_collision_util_interactive.h"


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "moveit_collision_util_interactive");
  ros::NodeHandle nh;


  // Must use AsyncSpinner! Otherwise MoveGroup.plan() hangs and doesn't
  //   return, even if it is out of a rosservice call!!
  // Ref: https://groups.google.com/forum/#!topic/moveit-users/s9b7IJuKRKY
  //ros::AsyncSpinner spinner (4);

  // Interactive marker
  server.reset (new interactive_markers::InteractiveMarkerServer(
    "moveit_collision_util_interactive", "", false));
  ros::Duration(0.1).sleep();


  int hertz = 10;

  // Size of interactive marker for center of box, and for size of box
  float c_scale = 0.6;
  float r_scale = 0.3;

  // NOTE: Frame name cannot have a preceding slash! Otherwise RViz won't plot
  //   the InteteractiveMarkers!
  MoveItCollisionUtil this_node = MoveItCollisionUtil (nh, hertz, c_scale,
    r_scale, "world");

  // Must start spinning AFTER IMarkerUtil is instantiated, `.` there
  //   are subscribers and publishers being initialized in its ctor. If spin
  //   before, those won't register. MoveGroup.plan() will only plan the first
  //   goal and then hang.
  //spinner.start ();

  printf ("ROS service is listening for calls...\n");


  ros::Rate wait_rate (hertz);

  // You can't call your own service. You have to just spin() the
  //   whole time! Test from a node outside (test_moveit_collision_util.cpp).
  while (ros::ok ())
  {
    // Publish the marker that changes size live responding to user input
    this_node.get_imarker_node () -> pub_markers ();
    //fprintf (stderr, "imarker pub_markers() called\n");
    ros::Duration (0.5).sleep ();

    ros::spinOnce ();
    wait_rate.sleep ();
  }

  this_node.cleanup_collision ();


  // You MUST have this line at the end, otherwise you get boost mutex error
  //   like this!:
  //   terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  //   what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
  //   Aborted (core dumped)
  server.reset();

  return 0;
}

