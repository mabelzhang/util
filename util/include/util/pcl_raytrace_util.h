#ifndef PCL_RAYTRACE_UTIL_H
#define PCL_RAYTRACE_UTIL_H

// Mabel Zhang
// 5 Sep 2018
//
// Utility functions for ray tracing using PCL octree.
//
// Ref:
//   Tutorial: http://pointclouds.org/documentation/tutorials/octree.php
//   API: http://docs.pointclouds.org/trunk/classpcl_1_1octree_1_1_octree_point_cloud_search.html
//


#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

// Only used for RViz visualization
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Local
#include <util/lin_alg_util.h>  // project_pts_onto_line()
#include <util/pcd_util.h>  // publish_cloud()
#include <util/skeleton.h>  // create_marker()


//#include <iostream>


class RayTracer
{

  private:

    pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::Ptr octree_;
    pcl::octree::OctreePointCloud <pcl::PointXYZ>::AlignedPointTVector
      voxels_;

    bool vis_;
    ros::NodeHandle * nh_;
    std::string frame_id_;
    ros::Publisher vis_pub_;

    float resolution_;


  public:

    // Parameters:
    //   resolution: size of leaf voxels, in meters.
    RayTracer (pcl::PointCloud <pcl::PointXYZ>::Ptr & input_cloud,
      float resolution, bool vis=false, ros::NodeHandle * nh=NULL,
      std::string frame_id="/world")
    {
      vis_ = vis;
      nh_ = nh;
      frame_id_ = frame_id;
      resolution_ = resolution;

      // Instantiate this early, else will not be created in time to publish
      //   the first markers in this function below.
      vis_pub_ = nh_ -> advertise <visualization_msgs::Marker> (
        "visualization_marker", 5);

      octree_ = pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::Ptr (
        new pcl::octree::OctreePointCloudSearch <pcl::PointXYZ> (resolution));
      octree_ -> setInputCloud (input_cloud);
      octree_ -> addPointsFromInputCloud ();

      octree_ -> getOccupiedVoxelCenters (voxels_);

      if (vis_)
      {
        // This uses a publisher of different type (pcl::PointCloud), so no
        //   conflict with visualization_msgs::Marker publisher.
        publish_cloud (input_cloud, frame_id_, "cloud", *nh_);

        // Visualize voxels
        printf ("Number of voxels: %ld\n", voxels_.size ());

        visualization_msgs::Marker marker_vx;
        create_marker (visualization_msgs::Marker::CUBE_LIST, frame_id_, 0,
          0, 0, 0, 0.8, 0.8, 0.8, 0.5, resolution_, resolution_, resolution_,
          marker_vx, "voxel", 1, 0, 0, 0, 0);

        for (int i = 0; i < voxels_.size (); i ++)
        {
          //printf ("Voxel: %g %g %g\n", voxels_.at (i).x, voxels_.at (i).y,
          //  voxels_.at (i).z);

          geometry_msgs::Point pt;
          pt.x = voxels_.at (i).x;
          pt.y = voxels_.at (i).y;
          pt.z = voxels_.at (i).z;
          marker_vx.points.push_back (pt);
        }
        vis_pub_.publish (marker_vx);
        ros::Rate (4).sleep ();
      }
    }

    ~RayTracer ()
    {
    }

    size_t get_n_voxels ()
    {
      return voxels_.size ();
    }

    void get_voxel (int i, pcl::PointXYZ & pt)
    {
      // Call copy ctor
      pt = pcl::PointXYZ (voxels_.at (i));
    }
 
    // Cast a ray into the point cloud. Test whether the ray goes through the
    //   point cloud, or stop before reaching the point cloud; i.e. along the
    //   ray, test whether the endpoint of ray is occluded by any points in
    //   the point cloud.
    // Parameters:
    //   origin: Starting point of ray to shoot into the point cloud.
    //   endpt: Endpoint of ray to shoot into the point cloud.
    //     Endpoint will be tested if in front of intersected voxel, or behind,
    //     from the perspective of the origin point, along the ray.
    //     If the ordering of the points projected onto the ray is (origin,
    //     endpoint, intersection), then endpoint is in front of intersection.
    //     If the ordering is (origin, intersection, endpoint), then endpoint
    //     is behind intersection.
    //   vis: Whether to visualize point cloud and raytracing in RViz.
    //   nh: Only used if vis == true.
    // Returns true if point is behind intersection (i.e. point cloud occludes
    //   the given point. Else returns false (point is in front of, or at
    //   intersection; point cloud does not occlude the given point).
    bool raytrace_occlusion_test (Eigen::Vector3f origin,
      Eigen::Vector3f endpt)
    {
      pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::
        AlignedPointTVector vx_centers;
      // Ray trace. Find centers of voxels that the ray intersects
      octree_ -> getIntersectedVoxelCenters (origin, endpt - origin,
        vx_centers);
      fprintf (stderr, "%ld voxels intersected by ray: ", vx_centers.size ());


      // Test occlusion of the given point, by the point in point cloud
      //   intersecting ray
      // n x 3
      Eigen::MatrixXf vx_centers_eg = Eigen::MatrixXf::Zero (
        vx_centers.size (), 3);
      for (int i = 0; i < vx_centers.size (); i ++)
      {
        // Convert to Eigen format
        vx_centers_eg.row (i) = Eigen::Vector3f (
          vx_centers.at (i).x, vx_centers.at (i).y, vx_centers.at (i).z);
      }


      // Calculate distance each, from the endpoint of ray, and from the point
      //   cloud frontier points, to the origin of ray.

      // Project the intersected voxel centers (part of the point cloud)
      //   onto ray.
      // Return value is distance from starting point of ray. Positive means
      //   along ray, negative means in opposite direction of ray.
      Eigen::VectorXf proj_vxs;
      project_pts_onto_line (origin, endpt, vx_centers_eg,
        proj_vxs);

      // Distance of endpoint of ray from origin of ray
      float proj_endpt = (endpt - origin).norm ();

      // If distance from ray origin to intersection in point cloud <
      //   distance from ray origin to the given point, then the given point
      //   is occluded by the cloud.
      //   ray_origin --- cloud --- endpoint
      // When there are multiple intersections in the cloud, as long as the
      //   given point is behind at least one point in the cloud, it is
      //   occluded by the cloud.
      bool occluded = false;
      if ((proj_vxs.array () < proj_endpt).count () > 0)
      {
        occluded = true;
        std::cerr << "Voxels occluding endpoint of ray: " << std::endl;
        for (int i = 0; i < proj_vxs.size (); i ++)
        {
          if (proj_vxs (i) < proj_endpt)
          {
            fprintf (stderr, "%g %g %g (distance %g)\n",
              vx_centers_eg (i, 0), vx_centers_eg (i, 1), vx_centers_eg (i, 2),
              proj_vxs (i));
          }
        }
      }
      else
        occluded = false;


      if (vis_)
      {
        /////
        // Visualize ray and occlusion in RViz. If occluded (i.e. ray goes
        //   through cloud), red ray. Else (ray goes through free space), green.

        float r = 0.0, g = 0.0, b = 0.0;
        // Color ray red, to indicate it hit something
        if (occluded)
        {
          r = 1.0;
          g = 0.0;
          b = 0.0;
        }

        // Color ray green, to indicate it went through free space.
        else
        {
          r = 0.0;
          g = 1.0;
          b = 0.0;
        }

        visualization_msgs::Marker marker_ray;
        create_marker (visualization_msgs::Marker::ARROW, frame_id_, 0,
          // sx: shaft diameter, sy: head diameter, sz if != 0: head length
          0, 0, 0, r, g, b, 0.8, 0.01, 0.02, 0,
          marker_ray, "ray", 1, 0, 0, 0, 0);

        // Start point of ray
        geometry_msgs::Point p1;
        p1.x = origin [0];
        p1.y = origin [1];
        p1.z = origin [2];
        marker_ray.points.push_back (p1);

        // End point of ray
        geometry_msgs::Point p2;
        p2.x = endpt [0];
        p2.y = endpt [1];
        p2.z = endpt [2];
        marker_ray.points.push_back (p2);


        // Intersected voxel
        visualization_msgs::Marker marker_vx;
        create_marker (visualization_msgs::Marker::CUBE_LIST, frame_id_, 0,
          0, 0, 0, 0, 0, 0, 0, resolution_, resolution_, resolution_,
          marker_vx, "voxel_intersected", 1, 0, 0, 0, 0);

        for (int i = 0; i < vx_centers.size (); i ++)
        {
          geometry_msgs::Point pt;
          pt.x = vx_centers.at (i).x;
          pt.y = vx_centers.at (i).y;
          pt.z = vx_centers.at (i).z;
          marker_vx.points.push_back (pt);

          std_msgs::ColorRGBA color;
          // Red, occluded
          if (proj_vxs (i) < proj_endpt)
            color.r = 1.0;
          // Green, ray through free space
          else
            color.g = 1.0;
          color.a = 0.5;
          marker_vx.colors.push_back (color);
        }

        // Publish Markers
        for (int i = 0; i < 5; i ++)
        {
          vis_pub_.publish (marker_vx);
          vis_pub_.publish (marker_ray);
          ros::Rate (10).sleep ();
        }
      }

      return occluded;
    }
};

#endif
