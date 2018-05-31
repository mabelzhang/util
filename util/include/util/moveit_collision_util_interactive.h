// Mabel Zhang
// 13 May 2017
//
// Allows user to interactively (using RViz InteractiveMarkers) add collision
//   boxes in MoveIt scene.
// Tested in ROS Indigo.
// 30 Jan 2018: Doesn't seem to add the object into the planning scene in ROS
//   Kinetic. Not sure if it's because PlanningSceneInterface is deprecated
//   (but it isn't? Only MoveGroup is deprecated, but only the MoveGroup
//   tutorials still use PlanningSceneInterface. The new tutorials simply use
//   PlanningScene), or a small change in setup somewhere.
//   Get warning at planning_scene_interface_ -> addCollisionObjects() call:
//   [ WARN] [1517338543.781970397, 1916.905000000]: Could not call planning scene service to get object geometries
//
// Copied from my baxter_reflex moveit_planner.cpp.
// Removed routines that deal with MoveIt planning and InteractiveMarkers.
//   Only left routines for MoveIt collision scene.
//
// These utility functions are useful for adding collision objects in MoveIt
//   planning scene.
//
// Calls srv_helper() in interactive_marker_util.h, which does the heavy-lifting
//   of actual user interaction, and drawing a box of live-changing sizes based
//   on user input.
//


#ifndef MOVEIT_COLLISION_UTIL_H
#define MOVEIT_COLLISION_UTIL_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

// MoveIt
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Local
#include "util/MoveItAddCollisionSrv.h"
#include "util/interactive_marker_util.h"


// Set to 0 if just want to test interactive markers, without having to
//   launch MoveIt RViz!
#define TEST_MOVEIT 1


/////////////////////////////////////////////////////////////// Utility Fns //

// Copied from demo_pick_pen pick_place_moveit_v2.cpp
// A convenience function for creating a moveit_msgs::CollisionObject.
//   Like my create_marker().
// Parameters:
//   target_obj: ret val with fields populated.
//   tx, ty, tz: center of collision box
void create_collision_object (moveit_msgs::CollisionObject & target_obj,
  std::string frame_id, std::string id, float sx, float sy, float sz,
  float tx, float ty, float tz, float qx=0, float qy=0, float qz=0, float qw=1)
{
  // First, we will define the collision object message.
  target_obj.header.frame_id = frame_id;

  // The id of the object is used to identify it.
  target_obj.id = id;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize (3);
  primitive.dimensions[0] = sx;
  primitive.dimensions[1] = sy;
  primitive.dimensions[2] = sz;

  // A pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.x = qx;
  box_pose.orientation.y = qy;
  box_pose.orientation.z = qz;
  box_pose.orientation.w = qw;

  box_pose.position.x = tx;
  box_pose.position.y = ty;
  box_pose.position.z = tz;

  target_obj.primitives.push_back (primitive);
  target_obj.primitive_poses.push_back (box_pose);
  target_obj.operation = target_obj.ADD;
}


void setStartStateToCurrentState_withObstacles (
  planning_scene_monitor::PlanningSceneMonitorPtr & psm,
  const std::string PLANNING_SCENE_SERVICE,
  boost::shared_ptr <moveit::planning_interface::MoveGroup> & group)
{
  // To fix start state not being correctly set to current state, when
  //   object is in gripper (object doesn't get considered in collision!)
  // From Jochen Welle on
  //   https://groups.google.com/forum/#!topic/moveit-users/ZYsWL1jRWyY
  psm -> requestPlanningSceneState (PLANNING_SCENE_SERVICE);
  planning_scene_monitor::LockedPlanningSceneRW ps (psm);
  ps -> getCurrentStateNonConst ().update ();
  robot_state::RobotState current_state = ps -> getCurrentState ();
  group -> setStartState (current_state);
}


//////////////////////////////////////////////////////////////// Main class //

class MoveItCollisionUtil
{
  public:

    MoveItCollisionUtil (ros::NodeHandle, int, float, float, std::string);
    ~MoveItCollisionUtil ();

    boost::shared_ptr <IMarkerUtil> get_imarker_node ();

    //void pub_markers ();

    bool add_collision_box_srv (
      util::MoveItAddCollisionSrv::Request & req,
      util::MoveItAddCollisionSrv::Response & res);

    // MoveIt
    void add_static_collision_objs ();
    void cleanup_collision ();


  private:

    ros::NodeHandle nh_;

    boost::shared_ptr <IMarkerUtil> imarker_node_;


    // Custom rosservices

    std::string add_collision_srv_name_;
    ros::ServiceServer add_collision_srv_;

    //bool accept_obj_bbox_;

    ros::Publisher vis_pub_;
    visualization_msgs::Marker marker_obj_;


    // MoveIt

    std::string robot_frame_id_;
    //std::string obj_bbox_id_;  // Only used once, don't need var

    std::vector <moveit_msgs::CollisionObject> colli_objs_;

    #if TEST_MOVEIT == 1

    boost::shared_ptr <tf::TransformListener> tf_listener_ptr_;
    //boost::shared_ptr <planning_scene_monitor::PlanningSceneMonitorPtr> psm_;
    std::string PLANNING_SCENE_SERVICE_;

    boost::shared_ptr <moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;

    // I can't believe this type doesn't have a no-arg ctor. So I have to
    //   make it into a boost pointer. This is so retarded.
    // API http://docs.ros.org/indigo/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
    boost::shared_ptr <moveit::planning_interface::MoveGroup> group_;

    bool add_camera_collision_;
    bool add_mount_collision_;

    int action_;

    tf::TransformBroadcaster tf_broadcaster_;

    #endif
};


////////////////////////////////////////////////////////////////////// Ctor //

// Ctor
// NOTE frame should not have a preceding slash, else RViz won't plot.
MoveItCollisionUtil::MoveItCollisionUtil (ros::NodeHandle nh, int hertz,
  float c_scale=0.06, float r_scale=0.03,
  std::string robot_frame=std::string("base"))
{
  // I think I found the hard way that nh_ has to stay in scope. Otherwise
  //   message subscribers die or something.
  nh_ = nh;

  imarker_node_ = boost::shared_ptr <IMarkerUtil> (new IMarkerUtil (nh, hertz,
    c_scale, r_scale, robot_frame));

  robot_frame_id_ = robot_frame;
  //obj_bbox_id_ = "obj";

  add_collision_srv_name_ = "/util/add_collision_box_interactive";
  add_collision_srv_ = nh_.advertiseService (add_collision_srv_name_,
    &MoveItCollisionUtil::add_collision_box_srv, this);

  vis_pub_ = nh_.advertise <visualization_msgs::Marker> (
    "visualization_marker", 0);

  // Make a green box
  // NOTE ROS Kinetic: Do NOT use a slash in namespace, otherwise RViz Marker
  //   won't get published!!!
  // Shouldn't need this, `.` the live-changing-size box is published in
  //   interactive_marker_util.h srv_helper() already.
  geometry_msgs::Point obj_center = imarker_node_ -> get_obj_center ();
  geometry_msgs::Vector3 obj_dims = imarker_node_ -> get_obj_dims ();
  geometry_msgs::Quaternion obj_quat = imarker_node_ -> get_obj_quat ();
  create_marker (visualization_msgs::Marker::CUBE, robot_frame_id_, 0,
    obj_center.x, obj_center.y, obj_center.z,
    0, 1, 0, 0.5, obj_dims.x, obj_dims.y, obj_dims.z,
    marker_obj_, "moveit_obj_bbox",
    obj_quat.w, obj_quat.x, obj_quat.y, obj_quat.z,
    0);


  ////////
  // MoveIt
  ////////

  #if TEST_MOVEIT == 1

  // 21 Jan 2017: For active, if remove camera rig from physical robot, then
  //   there are less constraints, and MoveIt is more likely to find feasible
  //   poses!!
  add_camera_collision_ = false;

  add_mount_collision_ = true;

  // If launch baxter_world in sim, or nothing on real robot
  std::string ROBOT_DESCRIPTION = "robot_description";

  tf_listener_ptr_ = boost::make_shared <tf::TransformListener> ();
  PLANNING_SCENE_SERVICE_ = "get_planning_scene";
  // 15 May 2017: Worked in Ubuntu 14.04 ROS Indigo, but not in 16.04 Kinetic
  //psm_ = boost::make_shared <planning_scene_monitor::PlanningSceneMonitor> (
  //  ROBOT_DESCRIPTION, tf_listener_ptr_);
  // Tried this too. Didn't work.
  //psm_ = boost::shared_ptr <planning_scene_monitor::PlanningSceneMonitor> (
  //  new planning_scene_monitor::PlanningSceneMonitor (ROBOT_DESCRIPTION, tf_listener_ptr_));

  // Ref: https://msdn.microsoft.com/en-us/library/hh279669.aspx
  planning_scene_interface_ = 
    boost::make_shared <moveit::planning_interface::PlanningSceneInterface> ();


  ////////
  // Init collision scene

  // Add in ctor, instead of in rossrv callback, `.` I'll be moving robot all
  //   the time anyway. Doesn't make sense to keep removing and re-adding
  //   every call.
  add_static_collision_objs ();

  #endif


  ROS_INFO ("Service initialized: %s, ", add_collision_srv_name_);
}


//void MoveItCollisionUtil::pub_markers ()
//{
//  vis_pub_.publish (marker_obj_);
//}


#if TEST_MOVEIT == 1

// Dtor
MoveItCollisionUtil::~MoveItCollisionUtil ()
{
  cleanup_collision ();
}

boost::shared_ptr <IMarkerUtil> MoveItCollisionUtil::get_imarker_node ()
{
  return imarker_node_;
}

// Table, camera rig, ReFlex Hand, etc known objects that won't change
void MoveItCollisionUtil::add_static_collision_objs ()
{
  // Table size xyz in robot frame. Position of table wrt robot always fixed.
  //   Measured with ruler.
  geometry_msgs::Vector3 table_dims;
  table_dims.x = 0.913;
  table_dims.y = 1.825;
  table_dims.z = 0.735;

  // The big table is -0.15. Small short table is -0.235. Two-drawer set by
  //   Sawyer is -0.345.
  //float tabletop_z = -0.15;
  float tabletop_z = -0.235;

  // Table position in robot frame. Orientation always upright.
  //   Measured with ruler.
  // ATTENTION: If you move table wrt robot, this needs to be redefined!
  geometry_msgs::Vector3 table_center;
  table_center.x = 0.74;
  table_center.y = 0;
  // Height is defined at object center. -0.15 is tabletop z wrt robot /base
  //table_center.z = -0.53;  // True
  table_center.z = tabletop_z - (0.5 * table_dims.z);

  // Table in robot frame
  moveit_msgs::CollisionObject table;
  create_collision_object (table, robot_frame_id_, "table",
    table_dims.x, table_dims.y, table_dims.z,
    table_center.x, table_center.y, table_center.z);
  colli_objs_.push_back (table);


  // Camera rig box
  if (add_camera_collision_)
  {
    geometry_msgs::Vector3 cam_rig_dims;
    cam_rig_dims.x = 0.17;
    cam_rig_dims.y = 0.24; // 0.21 was not safe enough, arm occasionally bumps cam
    cam_rig_dims.z = 0.48;
 
    geometry_msgs::Vector3 cam_rig_center;
    cam_rig_center.x = 0.22;
    cam_rig_center.y = 0.0;
    cam_rig_center.z = 0.39;
 
    // Camera rig in robot frame
    moveit_msgs::CollisionObject cam_rig;
    create_collision_object (cam_rig, robot_frame_id_, "cam_rig",
      cam_rig_dims.x, cam_rig_dims.y, cam_rig_dims.z,
      cam_rig_center.x, cam_rig_center.y, cam_rig_center.z);
    colli_objs_.push_back (cam_rig);
  }


  planning_scene_interface_ -> addCollisionObjects (colli_objs_);
  // Wait for RViz
  sleep (2.0);

  ROS_INFO ("These objects were added:");
  for (int i = 0;
    i < planning_scene_interface_ -> getKnownObjectNames ().size (); i ++)
  {
    ROS_INFO ("%s", planning_scene_interface_ ->
      getKnownObjectNames ().at (i).c_str ());
  }
}


void MoveItCollisionUtil::cleanup_collision ()
{
  // Clear so next time we don't add repeated objects
  colli_objs_.clear ();


  // Remove obstacles from the world

  ROS_INFO("Making sure all objects are removed from the world");

  // Robust way to remove all collision objects
  // API: http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html
  planning_scene_interface_ -> removeCollisionObjects (
    planning_scene_interface_ -> getKnownObjectNames ());

  // Sleep to give Rviz time to show the object is no longer there.
  sleep (4.0);
}

#endif


///////////////////////////////////////////// Interactive Marker rosservice //

// Custom rosservice
// Lets user adjust bbox with interactive marker, for 10 secs.
//   When time out, adds bbox as a collision box for MoveIt planning.
// Parameters:
//   req.pose_init: geometry_msgs/PoseStamped
//   req.dims_init: geometry_msgs/Vector3
bool MoveItCollisionUtil::add_collision_box_srv (
  util::MoveItAddCollisionSrv::Request & req,
  util::MoveItAddCollisionSrv::Response & res)
{
  fprintf (stderr, "Received rosservice call to %s\n", add_collision_srv_name_);

  // Set it. I want to be able to measure and quickly change in Python,
  //   instead of recompiling this every time.
  // Don't set it. Looks weird that marker suddenly changes size. Looks 
  //   broken. Also the user is already ready to adjust it, then in srv call,
  //   shape changes, catching the user off guard.
  // Set dims of obj to requested
  fprintf (stderr, "Initializing object center and dimensions according to srv request: "
    "c %.2f %.2f %.2f, r %.2f %.2f %.2f\n",
    req.pose_init.pose.position.x,
    req.pose_init.pose.position.y,
    req.pose_init.pose.position.z,
    req.dims_init.x, req.dims_init.y, req.dims_init.z);


  // Grab request quantities as initialization values for InteractiveMarkers

  geometry_msgs::Point obj_center;
  obj_center.x = req.pose_init.pose.position.x;
  obj_center.y = req.pose_init.pose.position.y;
  obj_center.z = req.pose_init.pose.position.z;

  geometry_msgs::Quaternion obj_quat;
  obj_quat.x = req.pose_init.pose.orientation.x;
  obj_quat.y = req.pose_init.pose.orientation.y;
  obj_quat.z = req.pose_init.pose.orientation.z;
  obj_quat.w = req.pose_init.pose.orientation.z;

  geometry_msgs::Vector3 obj_dims;
  obj_dims.x = req.dims_init.x;
  obj_dims.y = req.dims_init.y;
  obj_dims.z = req.dims_init.z;


  // Do the heavy-lifting of actual human interaction via InteractiveMarkers
  // This fn will update the arguments (references) to the ones user adjusted
  //   by InteractiveMarkers.
  imarker_node_ -> srv_helper (obj_center, obj_quat, obj_dims,
    req.ask_user_input, req.adjust_orientation);

  fprintf (stderr, "Object center set to %f %f %f, side lengths set to %f %f %f\n",
    obj_center.x, obj_center.y, obj_center.z,
    obj_dims.x, obj_dims.y, obj_dims.z);


  // Don't just add member var vector colli_objs_ to scene. It'll end up adding
  //   hand again, which will make all plans fail `.` it'd be detected to be in
  //   collision with the reflex that's already attached to robot! MoveIt can
  //   be smarter.
  // Make a new vector to be added to planning scene. Add the object to member
  //   var, but do not add it to scene.
  std::vector <moveit_msgs::CollisionObject> colli_obj_vec;

  // Create a collision box
  moveit_msgs::CollisionObject obj_bbox;
  create_collision_object (obj_bbox, robot_frame_id_, "obj", //obj_bbox_id_,
    obj_dims.x, obj_dims.y, obj_dims.z,
    obj_center.x, obj_center.y, obj_center.z,
    obj_quat.w, obj_quat.x, obj_quat.y, obj_quat.z);
  colli_objs_.push_back (obj_bbox);
  colli_obj_vec.push_back (obj_bbox);


  #if TEST_MOVEIT == 1
  fprintf (stderr, "Adding box to MoveIt collision scene\n");
  // (Don't need to manually remove first. If ID is same as existing, it'll
  //   be automatically detected and old one is replaced.)
  planning_scene_interface_ -> addCollisionObjects (colli_obj_vec);

  // Debug: see if object is actually added!
  // API: http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html
  fprintf (stderr, "%u objects in planning scene\n", planning_scene_interface_ -> getObjects ().size ());
  #endif


  // Set response msg
  res.success = true;
  res.pose_fin.header.frame_id = req.pose_init.header.frame_id;
  res.pose_fin.header.stamp = ros::Time::now ();
  res.pose_fin.pose.position.x = obj_center.x;
  res.pose_fin.pose.position.y = obj_center.y;
  res.pose_fin.pose.position.z = obj_center.z;
  res.pose_fin.pose.orientation.x = obj_quat.x;
  res.pose_fin.pose.orientation.y = obj_quat.y;
  res.pose_fin.pose.orientation.z = obj_quat.z;
  res.pose_fin.pose.orientation.w = obj_quat.w;

  res.dims_fin = geometry_msgs::Vector3 ();
  res.dims_fin.x = obj_dims.x;
  res.dims_fin.y = obj_dims.y;
  res.dims_fin.z = obj_dims.z;

  fprintf (stderr, "Service returning\n");

  return true;
}

#endif

