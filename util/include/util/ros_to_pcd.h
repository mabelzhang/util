#ifndef _ROS_TO_PCD_H_
#define _ROS_TO_PCD_H_

// Mabel Zhang
// 26 Jan 2018
//
// ROS service to write a point cloud on rostopic to .pcd file.
//
// To test, run the .cpp node:
// $ rosrun util ros_to_pcd
// Then call the service handled in this file, save to a temporary path:
// $ rosservice call /ros_to_pcd "{pcd_name: \"/home/master/graspingRepo/reFlexHand/catkin_ws/src/util/test.pcd\"}"
//

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

// Dependency in CMakeLists.txt and package.xml: pcl_ros
#include <pcl/io/pcd_io.h>

// My packages
#include <util_msgs/ROSToPCD.h>

class ROStoPCD
{
  private:

    std::string ros_to_pcd_srv_name_;

    ros::ServiceServer ros_to_pcd_srv_;
    ros::Subscriber cloud_sub_;

    sensor_msgs::PointCloud2 cloud_ros_;

    bool first_cloud_rcvd_;

  public:

    ROStoPCD (ros::NodeHandle nh, std::string & cloud_topic)
    {
      // Subscribe to the point cloud on rostopic
      cloud_sub_ = nh.subscribe (cloud_topic, 1, &ROStoPCD::cloud_cb, this);

      // Advertise the rosservice
      ros_to_pcd_srv_name_ = "/ros_to_pcd";
      ros_to_pcd_srv_ = nh.advertiseService (ros_to_pcd_srv_name_,
        &ROStoPCD::ros_to_pcd_handler, this);

      first_cloud_rcvd_ = false;

      printf ("Listening to rosservice call on %s\n", ros_to_pcd_srv_name_.c_str ());
    }

    void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
      if (! first_cloud_rcvd_)
      {
        first_cloud_rcvd_ = true;
      }
      cloud_ros_ = *msg;
    }

    // ROS service handler
    // Save a ROS topic to .pcd file
    bool ros_to_pcd_handler (util_msgs::ROSToPCD::Request & req,
                             util_msgs::ROSToPCD::Response & resp)
    {
      if (! first_cloud_rcvd_)
      {
        resp.success = false;
        resp.message = "No clouds received yet";
        return true;
      }

      pcl::PointCloud <pcl::PointXYZ> cloud_pcl;
      pcl::fromROSMsg (cloud_ros_, cloud_pcl);

      pcl::PCDWriter cloud_writer = pcl::PCDWriter ();
      cloud_writer.write (req.pcd_name, cloud_pcl);

      printf ("Written rostopic to PCD file %s\n", req.pcd_name.c_str ());

      resp.success = true;
      resp.message = "";

      return true;
    }

};

#endif

