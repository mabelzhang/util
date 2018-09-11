// Mabel Zhang
// 5 Sep 2018
//
// Load .pcd scene files outputted from BlenSor, by depth_scene_rendering
//   scene_generation.py.
// Given a contact point, test whether the point is in front of, or behind,
//   depth camera's reach (point cloud). If the point is behind any point in
//   the point cloud, it is considered occluded (behind).
//
// Usage:
//   $ rosrun tactile_graspit_collection occlusion_test
//

// C++
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Custom packages
#include <util/io_util.h>  // join_paths ()
#include <util/pcd_util.h>
#include <util/pcl_raytrace_util.h>  // RayTracer


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "occlusion_test");
  ros::NodeHandle nh;

  // Random seed
  srand (time (NULL));


  // Get path of package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");

  // Text file with list of .pcd scene names
  std::string noisy_scene_list_path = "";
  join_paths (pkg_path, "config/scenes_noisy.txt", noisy_scene_list_path);
  std::ifstream noisy_scene_list_f (noisy_scene_list_path.c_str ());

  // Octree resolution, in meters
  float octree_res = 0.005;

  // Read text file line by line. Each line is the path to a .pcd scene file
  std::string scene_name = "";
  while (std::getline (noisy_scene_list_f, scene_name))
  {
    // Instantiate cloud
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr =
      pcl::PointCloud <pcl::PointXYZ>::Ptr (
        new pcl::PointCloud <pcl::PointXYZ> ());
 
    // Load scene cloud
    load_cloud_file (scene_name, cloud_ptr);
    printf ("Cloud size: %ld points\n", cloud_ptr->size ());
    printf ("Organized? %s\n", cloud_ptr->isOrganized () ? "true" : "false");

    // Make octree to hold point cloud, for raytrace test
    // Ref: http://pointclouds.org/documentation/tutorials/octree.php
    RayTracer raytracer = RayTracer (cloud_ptr, octree_res, true, &nh);


    printf ("Testing ray-tracing...\n");
    // Origin of ray is always from camera center, 0 0 0.
    Eigen::Vector3f origin (0, 0, 0);


    // 1 m along z of camera frame, i.e. straight out of and normal to image
    //   plane.
    // Blender camera faces -z. So will shoot to -z.
    //Eigen::Vector3f endpoint (0, 0, -1);

    // Generate a number between 1 and 10
    int nPts = rand () % 10 + 1;
    printf ("Generated %d random points\n", nPts);
    // Generate a set of random endpoints
    // TODO: Generate integer indices of the voxels, and just as many points
    //   that aren't part of the voxels.
    Eigen::MatrixXf endpoints = Eigen::MatrixXf::Random (nPts, 3);

    // Must test endpoints one by one, not an n x 3 matrix, `.` octree
    //   getIntersectedVoxelCenters() only takes one ray at a time.
    for (int i = 0; i < nPts; i ++)
    {
      std::cout << "Ray through " << endpoints.row (i) << std::endl;

      // Ray trace
      bool occluded = raytracer.raytrace_occlusion_test (origin,
        endpoints.row (i));
      printf ("Occluded? %s\n", occluded ? "true" : "false");

      //char enter;
      //std::cout << "Press any character, then press enter: ";
      //std::cin >> enter;
    }
  }

  return 0;
}

