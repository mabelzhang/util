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

// Local
#include <util/lin_alg_util.h>  // project_pts_onto_line()
#include <util/pcd_util.h>  // publish_cloud()


//#include <iostream>


class RayTracer
{

  private:

    pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::Ptr octree_;

    bool vis_;
    ros::NodeHandle * nh_;


  public:

    RayTracer (pcl::PointCloud <pcl::PointXYZ>::Ptr & input_cloud,
      float resolution, bool vis=false, ros::NodeHandle * nh=NULL)
    {
      octree_ = pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::Ptr (
        new pcl::octree::OctreePointCloudSearch <pcl::PointXYZ> (resolution));
      octree_ -> setInputCloud (input_cloud);
      octree_ -> addPointsFromInputCloud ();

      vis_ = vis;
      nh_ = nh;

      if (vis_)
      {
        publish_cloud (input_cloud, "/world", "cloud", *nh_);
      }
    }

    ~RayTracer ()
    {
    }
 
    // Cast a ray into the point cloud. Test whether the ray goes through the
    //   point cloud, or stop before reaching the point cloud; i.e. along the
    //   ray, test whether the endpoint of ray is occluded by any points in
    //   the point cloud.
    // Parameters:
    //   origin, endpoint: Ray to shoot into the point cloud.
    //     Endpoint will be tested if in front of intersected voxel, or behind,
    //     from the perspective of the origin point, along the ray.
    //     If the ordering of the points projected onto the ray is (origin,
    //     point, intersection), then point is in front of intersection.
    //     If the ordering is (origin, intersection, point), then point is
    //     behind intersection.
    //   vis: Whether to visualize point cloud and raytracing in RViz.
    //   nh: Only used if vis == true.
    // Returns true if point is behind intersection (i.e. point cloud occludes
    //   the given point. Else returns false (point is in front of, or at
    //   intersection; point cloud does not occlude the given point).
    bool raytrace_occlusion_test (Eigen::Vector3f origin,
      Eigen::Vector3f endpoint)
    {
      pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::
        AlignedPointTVector vx_centers;
      // Ray trace. Find centers of voxels that the ray intersects
      octree_ -> getIntersectedVoxelCenters (origin, endpoint - origin,
        vx_centers);


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
      project_pts_onto_line (origin, endpoint, vx_centers_eg,
        proj_vxs);

      // Distance of endpoint of ray from origin of ray
      float proj_endpoint = (endpoint - origin).norm ();


      // TODO: Visualize in RViz
      if (vis_)
      {

      }


      // If distance from ray origin to intersection in point cloud <
      //   distance from ray origin to the given point, then the given point
      //   is occluded by the cloud.
      //   ray_origin --- cloud --- given_point
      // When there are multiple intersections in the cloud, as long as the
      //   given point is behind at least one point in the cloud, it is
      //   occluded by the cloud.
      if ((proj_vxs.array () < proj_endpoint).count () > 0)
        return true;
      else
        return false;
    }
};

#endif
