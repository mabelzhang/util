// Mabel Zhang
// 26 Feb 2018
//
// Runs the node with rosservice handler defined in pcd_to_ros.h.
//
// Usage:
//   $ rosrun util pcd_to_ros gz
//   or
//   $ rosrun util pcd_to_ros real
//   or
//   $ rosrun util pcd_to_ros /camera/depth_registered/points
// Pass in the rostopic name of the desired point cloud in command line args.
//   Real Kinect is /camera/depth_registered/points
//   Gazebo Kinect in gym_gazebo camera_table.world is /gz_camera/points
//

#include "ros/ros.h"

// My packages
#include <util/ansi_colors.h>

// Local
#include <util/pcd_to_ros.h>


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "pcd_to_ros");
  ros::NodeHandle nh;


  // Not publishing cloud anymore. Just return in rossrv so caller can publish
  //   as many times as it wants, as opposed to just one time here
  /*
  std::string cloud_topic = "";

  //printf ("argc: %d\n", argc);

  // Parse cmd line args
  // argv[0] is program name.
  if (argc > 1)
  {
    // Shorthand, so I don't have to type the topic name on command line every time
    if (! strcmp (argv [1], "real"))
    {
      cloud_topic = std::string ("/camera/depth_registered/points");
    }
    // Shorthand, so I don't have to type the topic name on command line every time
    else if (! strcmp (argv [1], "gz"))
    {
      cloud_topic = std::string ("/gz_camera/points");
    }
    else
    {
      cloud_topic = std::string (argv [1]);
    }
    printf ("%sWill publish point clouds to rostopic %s%s\n", OKCYAN, cloud_topic.c_str (), ENDC);
  }
  else
  {
    printf ("%sERROR: topic_name is unspecified in command line arguments.\n%s",
      FAIL, ENDC);
    printf ("Usage: $ rosrun util pcd_to_ros <real | gz | topic_name>\n");
    printf ("topic_name: \"real\" or \"gz\" to use the default rostopics defined in code, or literal name of the rostopic (type sensor_msgs/PointCloud2) that you want to record to .pcd file.\n");
    return 0;
  }
  */


  PCDtoROS this_node = PCDtoROS (nh); // cloud_topic);

  ros::spin ();

  return 0;
}

