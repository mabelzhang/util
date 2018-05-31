// Mabel Zhang
// 13 May 2017
//
// Demonstration of example usage of ../include/util/interactive_marker_util.h
//
// Copied from my baxter_reflex moveit_planner.cpp.
// Refactored from ../include/util/interactive_marker_util.h,
//   so that other packages can simply include the header file,
//   and so that header file can be checked for compilation success by being
//   included in a .cpp file to be built by compiler.
//


#include "util/interactive_marker_util.h"


////////////////////////////////////////////////////////////////////// Main //

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "interactive_marker_util");
  ros::NodeHandle nh;

  // Interactive marker
  server.reset (new interactive_markers::InteractiveMarkerServer(
    "interactive_marker_util", "", false));
  ros::Duration(0.1).sleep();


  int hertz = 10;
  IMarkerUtil this_node = IMarkerUtil (nh, hertz, 0.06, 0.03, "world");


  // Make some InteractiveMarkers
  InteractiveMarker center_imarker;

  std::string center_imarker_name_ = "Object Center";
  make6DofMarker (visualization_msgs::InteractiveMarkerControl::NONE,
    center_imarker_name_,
    // http://stackoverflow.com/questions/5245072/pass-and-call-a-member-function-boostbind-boostfunction
    boost::bind (&IMarkerUtil::obj_center_fb, this_node, _1), center_imarker,
    tf::Vector3 (1, 2, 3),
    true, true, true,
    false, false, true, 1, "/world");
 

  ros::Rate wait_rate (hertz);

  while (ros::ok ())
  {
    this_node.pub_markers ();

    ros::spinOnce ();
    wait_rate.sleep ();
  }


  // You MUST have this line at the end, otherwise you get boost mutex error
  //   like this!:
  //   terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  //   what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
  //   Aborted (core dumped)
  server.reset();

  return 0;
}

