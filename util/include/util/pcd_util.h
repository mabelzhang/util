#ifndef PCD_UTIL_H
#define PCD_UTIL_H

// Mabel Zhang
// 6 Feb 2018
//
// Utility for .pcd files
//
// This file is copied from different header files in active_visual_tactile
//   package.
//

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
// Dependency in CMakeLists.txt and package.xml: pcl_ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>  // rescale_cloud dependency
#include <pcl_ros/point_cloud.h>  // rescale_cloud dependency
#include <pcl/features/normal_3d.h>  // For estimating normals of a cloud
// For table segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// For min cut segmentation
#include <pcl/filters/passthrough.h>  // for min cut segmentation
#include <pcl/segmentation/min_cut_segmentation.h>  // for min cut segmentation

// Local
#include <util/io_util.h>
#include <util/ansi_colors.h>
#include <util/skeleton.h>


// Function declarations

// File I/O
bool load_cloud_file (const std::string & cloud_name,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr, float scale=1.0);
bool save_cloud_file (const std::string & file_name,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr);

// Preprocessing
void rescale_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  float factor);
void flip_z (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p);
// Plane segmentation
void discard_beyond_plane (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  Eigen::Vector3f & n, const Eigen::Vector3f & n_start,
  const std::string frame_id, ros::Publisher & vis_pub);
void segment_out_table (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & segmented_ptr,
  pcl::PointIndices::Ptr & inliers,
  pcl::ModelCoefficients::Ptr & coefficients,
  const std::string frame_id, ros::NodeHandle & nh, bool negative=true,
  bool publish=true);
// Set operation NOT
void negate_indices (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices_in, pcl::PointIndices::Ptr & indices_out);
void negate_indices (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices_in, std::vector <int> & indices_out);
void negate_indices (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices_in,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_out);
void min_cut_segment (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  const std::string frame_id, ros::Publisher & vis_pub);

// Conversions between pcl::PointCloud and ROS point cloud
void pcl_to_ros (const pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  sensor_msgs::PointCloud2 & msg);
void ros_to_pcl (const sensor_msgs::PointCloud2 & msg,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p);

// Visualization
//   Publish as ROS PointCloud (which is displayable in RViz)
void publish_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  const std::string frame_id, const std::string ns, ros::NodeHandle & nh,
  int sleep_time=5);
// Publish selected indices in the cloud
void publish_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices,
  const std::string frame_id, const std::string ns, ros::NodeHandle & nh,
  int sleep_time=5);

// Publish as ROS RViz POINTS Markers
// All markers will be of same specified color.
void visualize_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud,
  const std::string frame_id, const std::string ns,
  std_msgs::ColorRGBA & color, ros::Publisher & vis_pub);
// Each point marker can have a different color, as specified in the
//   PointXYZRGB.
void visualize_cloud (pcl::PointCloud <pcl::PointXYZRGB>::Ptr & cloud_rgb,
  const std::string frame_id, const std::string ns, ros::Publisher & vis_pub);


// ====================================================== Cloud file I/O ==

// Copied from active_visual_tactile input_cloud.h and feature_fpfh.h
// Load cloud from .pcd file
// Example use: visual_tactile_pipeline extend_table_infinitely.cpp
// Parameters:
//   pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr_ =
//     pcl::PointCloud <pcl::PointXYZ>::Ptr (
//       new pcl::PointCloud <pcl::PointXYZ> ());
bool load_cloud_file (const std::string & cloud_name,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr, float scale)
{
  if (! boost::filesystem::exists (cloud_name))
  {
    fprintf (stderr, "ERROR: File does not exist: %s\n", cloud_name.c_str ());
    return false;
  }

  printf ("Loading cloud from pcd file %s\n", cloud_name.c_str ());

  // Load a point cloud from .pcd file
  //   Copied from triangle_sampling sample_pcl.cpp
  //   API: http://docs.pointclouds.org/trunk/group__io.html
  pcl::io::loadPCDFile (cloud_name, *cloud_ptr);

  //printf ("Cloud size: %ld points\n", cloud_ptr->size ());
  //printf ("Organized? %s\n", cloud_ptr->isOrganized () ? "true" : "false");

  if (scale != 1.0)
    rescale_cloud (cloud_ptr, scale);

  // For some reason my MacBook Air doesn't have 1.8 anymore!!!???? What happened???
  /*
  // loadObjFile() only exists in PCL 1.8.0.
  // Ref: http://stackoverflow.com/questions/32539837/how-to-determine-pcl-point-cloud-library-version-in-c-code
  #if PCL_VERSION_COMPARE(>=, 1, 8, 0)

    pcl::io::loadOBJFile (cloud_name, cloud);

    // Shrink model, because orig points are in hundreds scale, too big for
    //   meters. I'm guessing they are in milimeters?
    // This is just for Menglong's OBJ files. For my PCD files, I resize in
    //   Blender manually to real-world size.
    for (int i = 0; i < cloud_ptr->size (); i ++)
    {
      // pcl::PointCloud API http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html
      cloud [i].x *= 0.001;
      cloud [i].y *= 0.001;
      cloud [i].z *= 0.001;
    }

  #else
    printf ("Not on MacBook Air. loadOBJFile() requires PCL 1.8, will not load any models!\n");

  #endif
  */

  return true;
}


bool save_cloud_file (const std::string & file_name,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr)
{
  // Check if parent directory of file exists. If not, create it.
  std::string dir_path;
  dirname (file_name, dir_path);
  create_dir_if_nonexist (dir_path);

  pcl::io::savePCDFileASCII (file_name, *cloud_ptr);
  //pcl::io::savePCDFileBinary (file_name, *cloud_ptr);

  printf ("Written cloud to pcd file %s\n", file_name.c_str ());
  return true;
}


// ================================================= Cloud preprocessing ==

// Rescale cloud by a constant factor. Useful for rescaling huge training
//   object models e.g. 3DNet.
// Refactored from input_cloud.h
void rescale_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  float factor)
{
  // Block is 3 x nPts, starting at 0 0
  //   MatrixXf is 3 x nPts, or 4 x nPts if recorded from xtion
  // Don't save getMatrixXfMap to a local var! It seems to be a temporary
  //   pointer, changes to the Eigen::MatrixXf are only local, they don't
  //   get saved to the pcl::PointCloud object!
  cloud_p -> getMatrixXfMap ().block (0, 0, 3, cloud_p -> size ()) *= factor;

  fprintf (stderr, "Rescaled cloud by %g\n", factor);
}

// Flip z component of points to -z.
// Useful for cameras that face -z, to flip recorded point clouds to +z.
void flip_z (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p)
{
  // Block is 1 x nPts, the z row of matrix
  //   MatrixXf is 3 x nPts, or 4 x nPts if recorded from xtion
  // Don't save getMatrixXfMap to a local var! It seems to be a temporary
  //   pointer, changes to the Eigen::MatrixXf are only local, they don't
  //   get saved to the pcl::PointCloud object!
  cloud_p -> getMatrixXfMap ().block (2, 0, 1, cloud_p -> size()) *= -1;

  fprintf (stderr, "Flipped cloud z to -z\n");
}


// Overloaded function. Call this version when don't want to
//   modify the cloud in an instance of the class, or don't have a
//   class instance, just want to operate on some cloud in caller.
// Preprocessing.
// Discard all point beyond a plane, given by its normal. (By definition,
//   a plane is defined by its normal.) A plane has two sides; the points
//   on the opposite side of the normal (dot product < 0) will be
//   discarded.
// Parameters:
//   cloud: Input and ret val, will be modified. Cloud in which to discard
//     points beyond plane defined by normal n.
//   n: Unit normal vector of plane
//   n_start: 3D point lying in plane. Each point will subtract this point
//     first, to establish a vector from the plane to that point, before
//     taking dot product of the point and the normal, to decide which side
//     of the plen that point lies on. Without this point, you don't know
//     where the plane starts! Normal vector only gives you orientation of
//     plane.
void discard_beyond_plane (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  Eigen::Vector3f & n, const Eigen::Vector3f & n_start,
  const std::string frame_id, ros::Publisher & vis_pub)
{
  bool do_remove_nans = false;


  fprintf (stderr, "Cloud size before discarding points beyond plane: %lu\n", cloud_p->size ());

  // Make sure n is unit vector
  // Ref Eigen norm(), normalize(), normalized():
  //   http://stackoverflow.com/questions/5613041/member-function-in-eigen-math-library-for-vector-magnitude
  //   https://eigen.tuxfamily.org/dox/classEigen_1_1MatrixBase.html#a680e8b0963abb141fc572c2d37d0496a
  if (n.norm () != 1)
  {
    fprintf (stderr, "WARNING in InputCloud::discard_beyond_plane(): "
      "normal vector does not have unit length. Normalizing it. "
      "This will alter the copy in caller too!\n");
    n.normalize ();
  }


  // Find two vectors lying in the plane. These two vectors will both be
  //   normal to n, by definition of plane.

  // Use cross product with some arbitrary vector to find one vector normal
  //   to n.
  // Use x-basis vector. If it's parallel to n (therefore cross product
  //   will be 0), then use z-basis vector.
  Eigen::Vector3f basis = Eigen::Vector3f (1, 0, 0);
  if ((n - basis).norm () < 1e-6)
    basis = Eigen::Vector3f (0, 0, 1);

  // Two vectors normal to n
  Eigen::Vector3f v1 = basis.cross (n);
  Eigen::Vector3f v2 = v1.cross (n);


  // Visualize the plane as two triangles in RViz
  //  2 ___ 3
  //   |  /|
  //   | / |
  //  1|/__|4
  //
  // Treat v1 as horizontal, v2 as vertical.
  // Then the 4 points can be obtained by adding to the center point
  //   of the plane:
  //     p1 = c - v1 - v2, p2 = c - v1 + v2,
  //     p3 = c + v1 + v2, p4 = c + v1 - v2

  // Copied and adapted from my continuumRepo active_chords action_util.py

  visualization_msgs::Marker marker_p;
  create_marker (visualization_msgs::Marker::TRIANGLE_LIST, frame_id, 0,
    0, 0, 0, 1, 1, 1, 0.8, 1, 1, 1,
    marker_p, "plane", 1, 0, 0, 0, 0);

  // Plane is infinite. Visualize just 1 m of it.
  float plane_len = 1;

  Eigen::Vector3f p1 = n_start + 0.5 * (- v1 - v2);
  Eigen::Vector3f p2 = n_start + 0.5 * (- v1 + v2);
  Eigen::Vector3f p3 = n_start + 0.5 * (v1 + v2);
  Eigen::Vector3f p4 = n_start + 0.5 * (v1 - v2);

  // Construct msgs
  geometry_msgs::Point p1m, p2m, p3m, p4m;
  p1m.x = p1.x ();
  p1m.y = p1.y ();
  p1m.z = p1.z ();

  p2m.x = p2.x ();
  p2m.y = p2.y ();
  p2m.z = p2.z ();

  p3m.x = p3.x ();
  p3m.y = p3.y ();
  p3m.z = p3.z ();

  p4m.x = p4.x ();
  p4m.y = p4.y ();
  p4m.z = p4.z ();

  // Define points on the triangle in clockwise order
  marker_p.points.push_back (p1m);
  marker_p.points.push_back (p2m);
  marker_p.points.push_back (p3m);

  marker_p.points.push_back (p3m);
  marker_p.points.push_back (p4m);
  marker_p.points.push_back (p1m);

  vis_pub.publish (marker_p);
  ros::Duration (0.1).sleep ();


  // Visualize normal of plane as an arrow

  // Length of arrow to visualize. Arbitrary
  float n_len = 0.3;

  Eigen::Vector3f n_end = n_start + n_len * n;

  visualization_msgs::Marker marker_n;
  // ARROW: scale.x is shaft diameter, scale.y is head diameter, scale.z is
  //   head height
  create_marker (visualization_msgs::Marker::ARROW, frame_id, 1,
    0, 0, 0, 1, 1, 1, 0.8, 0.02, 0.03, n_len * 0.2,
    marker_n, "plane", 1, 0, 0, 0, 0);

  geometry_msgs::Point n_start_msg;
  n_start_msg.x = n_start.x ();
  n_start_msg.y = n_start.y ();
  n_start_msg.z = n_start.z ();

  geometry_msgs::Point n_end_msg;
  n_end_msg.x = n_end.x ();
  n_end_msg.y = n_end.y ();
  n_end_msg.z = n_end.z ();

  marker_n.points.push_back (n_start_msg);
  marker_n.points.push_back (n_end_msg);

  vis_pub.publish (marker_n);
  ros::Duration (0.1).sleep ();

  fprintf (stderr, "Plane visualized\n");


  // Check for points to remove

  // Get matrix form of point cloud, for fast vectorized operations
  // 4 x nPts. The 4 dims are probably X Y Z Depth, `.` I recorded from
  //   xtion/depth_registered/points. There's no RGB in these. The field in
  //   .pcd file says type is "x y z rgb", not sure how one number can be
  //   rgb! Maybe grayscale heatmap? Then it is still depth.
  // API http://docs.pointclouds.org/1.7.1/classpcl_1_1_point_cloud.html
  Eigen::Map <Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >
    cloud_eigen = cloud_p -> getMatrixXfMap ();
  printf ("Cloud is %u x %u\n", cloud_p -> width, cloud_p -> height);
  printf ("Eigen map of cloud is %lu x %lu\n", cloud_eigen.rows (),
    cloud_eigen.cols ());
  /*
  // This looks right. Eigen::MatrixXf (0:2, i) == points[i].x:z
  // Is the first 3 elements not x y z??? Does PCL order them weird in the Eigen matrix??
  for (int i = 0; i < cloud_p -> size (); i ++)
  {
    fprintf (stderr, "pcl::PointCloud: %f %f %f. Eigen::MatrixXf: %f %f %f\n",
      cloud_p -> at (i).x,
      cloud_p -> at (i).y,
      cloud_p -> at (i).z,
      cloud_eigen (0, i),
      cloud_eigen (1, i),
      cloud_eigen (2, i)
    );

    if (! ros::ok ())
      break;
  }
  */

  // Mark points that contains nans and infs
  Eigen::Matrix <bool, Eigen::Dynamic, 1> isfin;
  if (do_remove_nans)
  {
    // nPts x 1
    // If a number is finite, it should equal itself. Else it's inf or nan.
    // Ref: https://forum.kde.org/viewtopic.php?f=74&t=91514
    isfin = 
      (cloud_eigen.array () == cloud_eigen.array ()).colwise ().all ();
    //printf ("isfin is %lu x %lu\n", isfin.rows(), isfin.cols());

    // Debug printout
    int n_finites = 0;
    for (int i = 0; i < isfin.rows (); i ++)
    {
      if (isfin (i))
        n_finites ++;
      //printf ("%d\n", isfin (i));
    }
    printf ("%d rows are finite.\n", n_finites);
  }
  else
  {
    // Init to all true, so no points get removed
    isfin = Eigen::Matrix <bool, Eigen::Dynamic, 1>::Constant (
      cloud_p -> size (), 1, true);
  }


  // Find points beyond the plane. Vectorized way.
  //   No for-looping through each point in cloud and dot product with each
  //     row.
  //   Uses vectorized form, pcl::PointCloud::getMatrixXfMap (). Just one
  //     matrix multiplication with normal vector, and checks the sign of
  //     each Eigen::MatrixXf row. Removes all rows with negative sign (i.e.
  //     point is beyond the plane).

  // Discard points on the normal's side of the plane.
  // To find which side of the plane a point is on, take its dot product
  //   with the normal vector of the plane. If dot product > 0, then the
  //   point is lying on same side as normal, discard it. Else keep it.
  // Dot product (which is cosine):
  //     0
  // -1 _|_ 1 (= n)
  //     |
  //     0

  // Dot product each point with normal
  //   REMEMBER to subtract n_start!!!!! Wasted me hours trying to figure
  //     out why this behaves different from for-loop version, as this got
  //     0 points in front of plane (dot prod > 0)!!!!!!!!!!!
  Eigen::Matrix <float, 3, Eigen::Dynamic> n_start_tiled =
    n_start.replicate (1, cloud_eigen.cols ());
  // (nPts x 1) = (nPts x 3) * (3 x 1)
  Eigen::Matrix <float, Eigen::Dynamic, 1> dot_prods =
    (cloud_eigen.block (0, 0, 3, cloud_eigen.cols ()) - n_start_tiled).transpose () * n;
  //printf ("dot_prods is %lu x %lu\n", dot_prods.rows (), dot_prods.cols ());
  // Mark points that have a negative dot product with normal
  //Eigen::Matrix <bool, Eigen::Dynamic, 1> is_beyond_plane =
  //  (dot_prods.array () < 0);

  // Debug printout
  int n_beyonds = 0;
  //for (int i = 0; i < is_beyond_plane.rows (); i ++)
  for (int i = 0; i < dot_prods.rows (); i ++)
  {
    //fprintf (stderr, "%f ", dot_prods (i));
    //if (! isnan (dot_prods (i)))
    //  fprintf (stderr, "%dth row in dot_prods is not nan\n", i);
    //if (dot_prods (i) > 0)
    //  fprintf (stderr, "%dth row in dot_prods > 0\n", i);

    //if (is_beyond_plane (i))
    if (dot_prods (i) < 0)
      n_beyonds ++;
    if (! ros::ok ())
      break;
  }
  printf ("%d points are beyond plane\n", n_beyonds);


  // Profile the loop's running time
  // Copied from sample_pcl.cpp
  // clock() doesn't work. It times ~190 s as 17 s, ridiculous.
  //   http://www.cplusplus.com/reference/ctime/time/
  struct tm y2k = {0};
  y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
  y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;
  time_t start_time = time (NULL);

  //printf ("isfin is %lu x %lu, is_beyond_plane is %lu x %lu\n",
  //  dot_prods.rows (), dot_prods.cols (),
  //  is_beyond_plane.rows (), is_beyond_plane.cols ());

  pcl::PointCloud <pcl::PointXYZ>::Ptr new_cloud_p (new
    pcl::PointCloud <pcl::PointXYZ> ());
  for (int i = 0; i < cloud_p -> size (); i ++)
  {
    //if (isfin (i) && (! is_beyond_plane (i)))
    if (isfin (i) && (dot_prods (i) >= 0))
      new_cloud_p -> push_back (cloud_p -> at (i));
  }
  fprintf (stderr, "Cloud size after discarding points beyond plane: %lu\n", new_cloud_p -> size ());
  // TODO: Hopefully this persists after function returns?
  cloud_p = new_cloud_p;

  /*
  // This loop is too slow. Probably because std::vector::erase() is
  //   super slow, as it needs to shift all elements up, so it's an n x n
  //   loop. You are better off just creating a new point cloud with
  //   the points (like the loop above)!

  // Discard points, using the vectorized boolean vectors
  int fin_i = 0;
  for (pcl::PointCloud <pcl::PointXYZ>::iterator it = cloud_p -> begin ();
    it < cloud_p -> end (); it ++)
  {
    if ((! isfin (fin_i)) || is_beyond_plane (fin_i))
    {
      cloud_p -> erase (it);
      // Decrement iterator after removing an elt!
      it --;
    }
    fin_i ++;

    if (! ros::ok ())
      break;
  }
  */


  /*
  // Discard points

  // To index isfin
  int fin_i = 0;

  //for (pcl::PointCloud <pcl::PointXYZ>::iterator it = cloud_p -> begin ();
  //  it < cloud_p -> end (); it ++)
  for (int i = 0; i < cloud_p -> size (); i ++)
  {
    if (! ros::ok ())
      break;

    // If point is infinite, just remove
    if (! isfin (fin_i))
    {
      //cloud_p -> erase (it);
      //it --;
      cloud_p -> erase (cloud_p -> begin () + i);
      i --;
    }

    // Else test if the point is beyond the plane, if so, remove it
    else
    {
      //Eigen::Vector3f pt = Eigen::Vector3f (
      //  it -> x, it -> y, it -> z);
      // Using i here: Hopefully cloud_eigen's rows decrease as points get erase()d from the pcl::PointCloud?
      // 83 secs
      Eigen::Vector3f pt = cloud_eigen.block <3, 1> (0, i);
      // 82 secs
      //Eigen::Vector3f pt = Eigen::Vector3f (
      //  cloud_p -> at(i).x, cloud_p -> at(i).y, cloud_p -> at(i).z);
      pt -= n_start;

      //printf ("Dot prod: %f\n", n.dot (pt));
     
      // Discard all < 0 will keep points lying on the plane.
      // Discard all <= 0 will throw away points lying on the plane.
      // Ref remove an item from std::vector http://stackoverflow.com/questions/39912/how-do-i-remove-an-item-from-a-stl-vector-with-a-certain-value
      if (n.dot (pt) < 0)
      {
        //cloud_p -> erase (it);
        // Decrement iterator after removing an item!
        //it --;
        cloud_p -> erase (cloud_p -> begin () + i);
        i --;
      }
    }

    // Increment the counter over isfin matrix. Don't decrement, `.` local
    //   boolean matrix didn't decrease in size, only the point cloud did.
    fin_i ++;
  }
  */

  // Copied from sample_pcl.cpp
  time_t end_time = time (NULL);
  double duration = difftime (end_time, mktime (&y2k)) -
    difftime (start_time, mktime (&y2k));
  fprintf (stderr, "Time elapsed: %f seconds. \n", duration);

  fprintf (stderr, "Cloud size after discarding points beyond plane: %lu\n", cloud_p -> size ());
}

// Preprocessing
// Call discard_beyond_plane first to remove everything that is beyond
//   the table. That gives you a clean cloud to segment out table.
//   Otherwise, because the PCL segmentation method simply finds the
//   largest plane in scene, it will find the floor instead of table!
// If segmented_ptr == cloud_ptr, will modify cloud_ptr in place.
// Parameters:
//   segmented_ptr: Return value
//   inliers: Return value. Initialize like so:
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   coefficients: Return value. Initialize like so:
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   frame_id: Only for visualization of cloud
//   negative: If true, will return point cloud of objects, i.e. with table
//     removed. If false, will return point cloud of table.
//   publish: Whether to publish cloud on rostopic. Takes longer.
// Returns inliers, coefficients, segmented_ptr.
//   If SACSegmentation::segment() errors and cannot find any planes, then
//   these are returned as is. Caller should check their sizes.
// Ref planar segmentation http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
//   ModelCoefficients API http://docs.pointclouds.org/trunk/structpcl_1_1_model_coefficients.html
void segment_out_table (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & segmented_ptr,
  pcl::PointIndices::Ptr & inliers,
  pcl::ModelCoefficients::Ptr & coefficients,
  const std::string frame_id, ros::NodeHandle & nh, bool negative,
  bool publish) //, Eigen::Vector3f axis)
{
  // Create the segmentation object
  // API http://docs.pointclouds.org/1.7.1/classpcl_1_1_s_a_c_segmentation.html
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  //   DistanceThreshold: Set a larger one to handle noise
  //     For my real Kinect data in Dec 2016, 0.01 still leaves some noise.
  //       0.02 no more noise.
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);

  // Set axis along which we should search for a plane perpendicular to
  //seg.setAxis (axis);

  // Default is 50, too low, doesn't get right table for Cody_bottle_42 and 44
  seg.setMaxIterations (500);
  fprintf (stderr, "Segmentation max iterations: %d\n", seg.getMaxIterations ());

  //fprintf (stderr, "Size of point cloud: %lu\n", cloud_ptr->size ());

  seg.setInputCloud (cloud_ptr);
  // coefficients are 4 float values, in plane eqn ax+by+cz+d = 0
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("ERROR: Could not estimate a planar model for the given dataset. Returning empty inliers and coefficients.");
    return;
  }

  /*
  std::cerr << "Model coefficients: "
    << coefficients->values[0] << " " 
    << coefficients->values[1] << " "
    << coefficients->values[2] << " " 
    << coefficients->values[3] << std::endl;
  */

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  /*
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    "
      << cloud_ptr->points[inliers->indices[i]].x << " "
      << cloud_ptr->points[inliers->indices[i]].y << " "
      << cloud_ptr->points[inliers->indices[i]].z << std::endl;
  */


  // Publish the inliers to see if segmented table out correctly
  if (publish)
  {
    fprintf (stderr, "Publishing segmented table cloud...\n");
    publish_cloud (cloud_ptr, inliers, frame_id, "/table", nh);
  }


  // Remove table from point cloud passed in as parameter

  // Flip the inlier indices, i.e. keep just the outliers (non-table)
  // Ref negating indices: http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html#add1af519a1a4d4d2665e07a942262aac
  //   http://www.pcl-users.org/Removing-points-using-vector-lt-int-gt-indices-td4023004.html
  pcl::ExtractIndices <pcl::PointXYZ> eifilter (true);
  eifilter.setInputCloud (cloud_ptr);
  eifilter.setIndices (inliers);
  // Set negative, to get indices not in table inliers. These are of object
  eifilter.setNegative (negative);
  eifilter.filter (*segmented_ptr);


  // If negative=true,
  //   this publishes the remaining cloud, to see if objects are kept correctly.
  // If negative=false,
  //   this publishes the table cloud again.
  if (publish)
  {
    fprintf (stderr, "Publishing segmentation return cloud...\n");
    publish_cloud (segmented_ptr, frame_id, "/segmented", nh);
  }
}

void negate_indices (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices_in, pcl::PointIndices::Ptr & indices_out)
{
  negate_indices (cloud_ptr, indices_in, indices_out->indices);
}

// Overloaded function. Output negated indices.
// Ref negating indices:
//   API and code example: http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html#add1af519a1a4d4d2665e07a942262aac
//   http://www.pcl-users.org/Removing-points-using-vector-lt-int-gt-indices-td4023004.html
void negate_indices (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices_in, std::vector <int> & indices_out)
{
  pcl::ExtractIndices <pcl::PointXYZ> eifilter (true);

  eifilter.setInputCloud (cloud_ptr);
  eifilter.setIndices (indices_in);

  eifilter.setNegative (true);
  eifilter.filter (indices_out);
}

// Overloaded function. Output point cloud with indices points removed.
// Ref negating indices:
//   API and code example: http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html#add1af519a1a4d4d2665e07a942262aac
//   http://www.pcl-users.org/Removing-points-using-vector-lt-int-gt-indices-td4023004.html
void negate_indices (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices_in,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_out)
{
  pcl::ExtractIndices <pcl::PointXYZ> eifilter (true);

  eifilter.setInputCloud (cloud_ptr);
  eifilter.setIndices (indices_in);

  eifilter.setNegative (true);
  eifilter.filter (*cloud_out);
}

// Segment out individual objects in a clutter.
// Parameters:
//   frame_id: Only for visualization of cloud
// Ref min cut segmentation http://www.pointclouds.org/documentation/tutorials/min_cut_segmentation.php#min-cut-segmentation
void min_cut_segment (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  const std::string frame_id, ros::Publisher & vis_pub)
{
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_ptr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::MinCutSegmentation<pcl::PointXYZ> seg;
  seg.setInputCloud (cloud_ptr);
  seg.setIndices (indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  // TODO: This probably matters. But how do I know object center before I
  //   even segment?? That's stupid.
  pcl::PointXYZ point;
  point.x = cloud_ptr -> at (0).x;
  point.y = cloud_ptr -> at (0).y;
  point.z = cloud_ptr -> at (0).z;
  foreground_points->points.push_back(point);
  seg.setForegroundPoints (foreground_points);

  seg.setSigma (0.25);
  seg.setRadius (3.0433856);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);

  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);

  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  for (int i = 0; i < clusters.size (); i ++)
  {
    printf ("Min cut cluster %d has %lu points\n", i, clusters [i].indices.size ());
  }

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
  // TODO: Visualize the output. Use the colored_cloud, write a fn to
  //   extract cloud colors and publish as POINTS_LIST with .colors field.
  visualize_cloud (colored_cloud, frame_id, "min_cut", vis_pub);
}


// ========================================================= Conversions ==

// Convert pcl::PointCloud to ROS point cloud
void pcl_to_ros (const pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  sensor_msgs::PointCloud2 & msg)
{
  // From pcl_conversions.h
  //   https://github.com/ros-perception/pcl_conversions/blob/indigo-devel/include/pcl_conversions/pcl_conversions.h#L535
  toROSMsg (*cloud_p, msg);
}

// Convert ROS point cloud to pcl::PointCloud
void ros_to_pcl (const sensor_msgs::PointCloud2 & msg,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p)
{
  // From pcl_conversions.h
  //   https://github.com/ros-perception/pcl_conversions/blob/indigo-devel/include/pcl_conversions/pcl_conversions.h#L535
  fromROSMsg (msg, *cloud_p);
}


// ======================================================= Visualization ==
// Copied from active_visual_tactile input_cloud.h

// Overloaded function
// Publish a PCL point cloud onto rostopic as sensor_msgs::PointCloud2
void publish_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p,
  const std::string frame_id, const std::string ns, ros::NodeHandle & nh,
  int sleep_time)
{
  // Large queue size is needed to prevent buffer overruns, in case of
  //   large point clouds.
  // This will be published as sensor_msgs/PointCloud2 automatically. Don't
  //   need to convert to sensor_msgs manually.
  // Ref: http://www.ros.org/wiki/pcl_ros
  ros::Publisher cloud_pub = nh.advertise <pcl::PointCloud
    <pcl::PointXYZ> > (ns, 20);
  cloud_p -> header.seq = 0;
  cloud_p -> header.frame_id = frame_id;

  int i = 0;
  while (ros::ok ())
  {
    cloud_p -> header.stamp = ros::Time::now ().toNSec () / 1000ull;
    cloud_pub.publish (*cloud_p);
    // Long wait time in between publishes is needed to prevent buffer
    //   overruns, in case of large point clouds.
    ros::Duration (1).sleep ();

    i++;
    if (i >= sleep_time)
      break;
  }
}


// Overloaded function
// Publish only the specified indices of points in the member field cloud
void publish_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr,
  pcl::PointIndices::Ptr & indices,
  const std::string frame_id, const std::string ns, ros::NodeHandle & nh,
  int sleep_time)
{
  // Make a new cloud containing only indexed points.
  //   Use the pcl::PointCloud copy ctor, passing in indices.
  // API PointIndices http://docs.pointclouds.org/1.7.1/structpcl_1_1_point_indices.html
  pcl::PointCloud <pcl::PointXYZ> table = pcl::PointCloud <pcl::PointXYZ> (
    *cloud_ptr, indices -> indices);

  // Large queue size is needed to prevent buffer overruns, in case of
  //   large point clouds.
  ros::Publisher cloud_pub = nh.advertise <pcl::PointCloud
    <pcl::PointXYZ> > (ns, 20);
  table.header.seq = 0;
  table.header.frame_id = frame_id;

  int i = 0;
  while (ros::ok ())
  {
    table.header.stamp = ros::Time::now ().toNSec () / 1000ull;
    cloud_pub.publish (table);
    // Long wait time in between publishes is needed to prevent buffer
    //   overruns, in case of large point clouds.
    ros::Duration (1).sleep ();

    i++;
    if (i >= 5)
      break;
  }
}


// I don't know how to get this to compile
// Template fn. Set XYZ fields of a RViz POINTS marker, using XYZ fields
//   of a pcl::PointCloud <T>.
// Parameters:
//   marker: ret val
// Ref write template fn overloaded with specific-type fn, let compiler choose which to call
//   http://stackoverflow.com/questions/2265381/how-do-i-check-my-template-class-is-of-a-specific-classtype
/*
template <T>
void visualize_cloud (pcl::PointCloud <T>::Ptr & cloud,
  const std::string frame_id, const std::string ns,
  visualization_msgs::Marker & marker)
{
  float size = 0.002;
  
  create_marker (visualization_msgs::Marker::POINTS, frame_id, 0,
    0, 0, 0, 1.0, 1.0, 1.0, 1.0, size, size, size,
    marker, ns, 1, 0, 0, 0, 0);

  for (pcl::PointCloud <T>::iterator it = cloud -> begin ();
    it < cloud -> end (); it ++)
  {
    geometry_msgs::Point pt;
    pt.x = it -> x;
    pt.y = it -> y;
    pt.z = it -> z;
    marker.points.push_back (pt);
  }
}
*/


// Visualize a cloud in RViz Marker. This might be slower than publishing
//   the cloud directly onto a rostopic as sensor_msgs::PointCloud2.
// All markers will be of same specified color.
// Only need to use RViz POINTS when want to color each point cloud a
//   different color. RViz only lets you set one color for all PointCloud2s
//   displayed.
void visualize_cloud (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud,
  const std::string frame_id, const std::string ns,
  std_msgs::ColorRGBA & color, ros::Publisher & vis_pub)
{
  float size = 0.002;
  
  visualization_msgs::Marker marker;
  create_marker (visualization_msgs::Marker::POINTS, frame_id, 0,
    0, 0, 0, color.r, color.g, color.b, color.a, size, size, size,
    marker, ns, 1, 0, 0, 0, 0);

  for (pcl::PointCloud <pcl::PointXYZ>::iterator it = cloud -> begin ();
    it < cloud -> end (); it ++)
  {
    geometry_msgs::Point pt;
    pt.x = it -> x;
    pt.y = it -> y;
    pt.z = it -> z;
    marker.points.push_back (pt);
  }

  // Trying to use template function
  /*
  // Set XYZ points of cloud
  visualization_msgs::Marker marker;
  visualize_cloud (cloud, frame_id, ns, marker);

  // Set RGBA colors of cloud
  marker.color = color;
  */

  vis_pub.publish (marker);
}


// Overloaded function
// Visualize a point cloud in RViz Marker, each point a specified color
// Each point marker can have a different color, as specified in the
//   PointXYZRGB.
void visualize_cloud (pcl::PointCloud <pcl::PointXYZRGB>::Ptr & cloud_rgb,
  const std::string frame_id, const std::string ns, ros::Publisher & vis_pub)
{
  float size = 0.002;
  
  visualization_msgs::Marker marker;
  create_marker (visualization_msgs::Marker::POINTS, frame_id, 0,
    0, 0, 0, 1.0, 1.0, 1.0, 1.0, size, size, size,
    marker, ns, 1, 0, 0, 0, 0);

  for (pcl::PointCloud <pcl::PointXYZRGB>::iterator it = cloud_rgb -> begin ();
    it < cloud_rgb -> end (); it ++)
  {
    geometry_msgs::Point pt;
    pt.x = it -> x;
    pt.y = it -> y;
    pt.z = it -> z;
    marker.points.push_back (pt);

    std_msgs::ColorRGBA color;
    color.r = it -> r;
    color.g = it -> g;
    color.b = it -> b;
    color.a = 0.8;
    marker.colors.push_back (color);
  }


  // Trying to use template function
  /*
  // Set XYZ points of cloud
  visualization_msgs::Marker marker;
  visualize_cloud (cloud_rgb, frame_id, ns, marker);

  // Set RGBA colors of cloud
  for (pcl::PointCloud <pcl::PointXYZRGB>::iterator it = cloud_rgb -> begin ();
    it < cloud_rgb -> end (); it ++)
  {
    std_msgs::ColorRGBA color;
    color.r = it -> r;
    color.g = it -> g;
    color.b = it -> b;
    color.a = 0.8;
    marker.colors.push_back (color);
  }
  */

  vis_pub.publish (marker);
}


#endif
