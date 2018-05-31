#ifndef _PCD_TO_ROS_H_
#define _PCD_TO_ROS_H_

// Mabel Zhang
// 26 Feb 2018
//
// ROS service to load a .pcd file and publish it on rostopic point cloud.
//
// To test, run the .cpp node: TODO to be written:
// $ rosrun util pcd_to_ros
// Then call the service handled in this file, save to a temporary path:
// $ rosservice call /pcd_to_ros "{pcd_name: \"/home/master/graspingRepo/reFlexHand/catkin_ws/src/util/test.pcd\"}"
//

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

// PCL
#include <pcl/io/pcd_io.h>
// Dependency in CMakeLists.txt and package.xml: pcl_ros
// Needed for publishing pcl::PointCloud type on rostopic
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// My packages
#include <util_msgs/PCDToROS.h>
#include <util/pcd_util.h>


class PCDtoROS
{
  private:

    std::string pcd_to_ros_srv_name_;

    ros::ServiceServer pcd_to_ros_srv_;
    //ros::Publisher cloud_pub_;
    //std::string cloud_topic_;
    int seq_;


  public:

    PCDtoROS (ros::NodeHandle nh) //, std::string & cloud_topic)
    {
      // Large queue size is needed to prevent buffer overruns, in case of
      //   large point clouds.
      // This will be published as sensor_msgs/PointCloud2 automatically. Don't
      //   need to convert to sensor_msgs manually.
      // Ref: http://www.ros.org/wiki/pcl_ros
      //cloud_topic_ = cloud_topic;
      //cloud_pub_ = nh.advertise <pcl::PointCloud <pcl::PointXYZ> > (
      //  cloud_topic_, 20);
      seq_ = 0;

      // Advertise the rosservice
      pcd_to_ros_srv_name_ = "/pcd_to_ros";
      pcd_to_ros_srv_ = nh.advertiseService (pcd_to_ros_srv_name_,
        &PCDtoROS::pcd_to_ros_handler, this);

      printf ("Listening to rosservice call on %s\n", pcd_to_ros_srv_name_.c_str ());
    }

    // ROS service handler
    // Load the next point cloud from file, publish onto rostopic
    bool pcd_to_ros_handler (util_msgs::PCDToROS::Request & req,
                             util_msgs::PCDToROS::Response & resp)
    {
      // Load point cloud from .pcd file
      pcl::PCDReader cloud_reader = pcl::PCDReader ();
      pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud <pcl::PointXYZ> ());
      cloud_reader.read (req.pcd_name, *cloud_pcl);

      // Instead of publishing, return in the service call. `.` only publishing
      //   once is not safety guarantee, packet might get dropped and caller
      //   never hears the point cloud!
      // Publish on rostopic
      /*
      cloud_pcl.header.seq = seq_;
      cloud_pcl.header.frame_id = req.frame_id;
      cloud_pcl.header.stamp = ros::Time::now ().toNSec () / 1000ull;

      cloud_pub_.publish (cloud_pcl);
      // Long wait time in between publishes is needed to prevent buffer
      //   overruns, in case of large point clouds.
      ros::Duration (1).sleep ();
      printf ("Cloud published on %s with seq %d\n", cloud_topic_.c_str (), seq_);
      */

      // Convert to ROS msg
      sensor_msgs::PointCloud2 cloud_ros;
      pcl_to_ros (cloud_pcl, cloud_ros);
      cloud_ros.header.seq = seq_;
      cloud_ros.header.frame_id = req.frame_id;
      cloud_ros.header.stamp = ros::Time::now ();
      resp.cloud = cloud_ros;

      resp.success = true;
      resp.message = "";

      printf ("Cloud with seq %d returned from service\n", seq_);
      seq_++;

      return true;
    }

};

#endif

