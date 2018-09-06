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

//#include <iostream>


class RayTracer
{

  private:

    pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::Ptr octree_;


  public:

    RayTracer (pcl::PointCloud <pcl::PointXYZ>::Ptr & input_cloud,
      float resolution)
    {
      octree_ = pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::Ptr (
        new pcl::octree::OctreePointCloudSearch <pcl::PointXYZ> (resolution));
      octree_ -> setInputCloud (input_cloud);
      octree_ -> addPointsFromInputCloud ();

    }

    ~RayTracer ()
    {
    }
 
    // Parameters:
    //   origin, direction: Ray to shoot into the point cloud
    //   point: Point to test if in front of intersected voxel, or behind,
    //     from the perspective of the origin point, along the ray.
    //     If the ordering of the points projected onto the ray is (origin,
    //     point, intersection), then point is in front of intersection.
    //     If the ordering is (origin, intersection, point), then point is
    //     behind intersection.
    // Returns true if point is behind intersection (i.e. point cloud occludes
    //   the given point. Else returns false (point is in front of, or at
    //   intersection; point cloud does not occlude the given point).
    bool raytrace_occlusion_test (Eigen::Vector3f origin,
      Eigen::Vector3f direction, Eigen::Vector3f point)
    {
      // Find centers of voxels that the ray intersects
      pcl::octree::OctreePointCloudSearch <pcl::PointXYZ>::
        AlignedPointTVector vx_centers;

      octree_ -> getIntersectedVoxelCenters (origin, direction,
        vx_centers);


      float dist_intersect = 0.0, dist_point = 0.0;

      // Test occlusion of point by point in point cloud intersecting ray
      // TODO: There should really be only 1 intersection. What to do if there
      //   are multiple? Intersection could be in between two. As long as it
      //   is behind at least one frontier of the cloud, it is occluded.
      for (int i = 0; i < vx_centers.size (); i ++)
      {
        // TODO: Project the intersected voxel center (part of the point cloud)
        //   in vx_centers onto ray

        // TODO: Project point given in parameters onto ray

        // TODO: Calculate distance of the two projected points, each, to
        //   origin of ray.
        //dist_intersect = 
        //dist_point = 


        // TODO: Visualize in RViz

        // If distance from ray origin to intersection in point cloud <
        //   distance from ray origin to the given point, then the given point
        //   is occluded by the cloud.
        //   ray_origin --- cloud --- given_point
        if (dist_intersect < dist_point)
          return true;
        else
          continue;
      }

      return false;
    }




};

#endif
