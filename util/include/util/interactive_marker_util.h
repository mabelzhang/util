// Mabel Zhang
// 13 May 2017
//
// Copied from my baxter_reflex moveit_planner.cpp.
// Removed routines that deal with MoveIt. Only left routines
//   for InteractiveMarker.
//
// These utility functions are useful for user interaction to indicate regions
//   in scene, location and size of objects in scene, via InteractiveMarkers
//   in RViz.
//
// A demonstration of how these functions are used is in
//   ./moveit_collision_util.h
//

#ifndef INTERACTIVE_MARKER_UTIL_H
#define INTERACTIVE_MARKER_UTIL_H

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// My packages
#include "util/ansi_colors.h"

// Local
#include "util/skeleton.h"  // create_marker()
#include "util/create_interactive_markers.h"  // server


//////////////////////////////////////////////////////////////// Main class //

class IMarkerUtil
{
  public:

    IMarkerUtil (ros::NodeHandle, int, float, float, std::string);
    ~IMarkerUtil () {};

    geometry_msgs::Point get_obj_center ();
    geometry_msgs::Quaternion get_obj_quat ();
    geometry_msgs::Vector3 get_obj_dims ();
    visualization_msgs::Marker get_marker_obj ();
    geometry_msgs::Vector3 get_prev_obj_dims ();
    bool get_accept_user_input ();

    void set_obj_center (geometry_msgs::Point);
    void set_obj_quat (geometry_msgs::Quaternion);
    void set_obj_dims (geometry_msgs::Vector3);
    void set_prev_obj_dims (geometry_msgs::Vector3 dims);
    void set_accept_user_input (bool val);

    void obj_center_fb (
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void obj_dims_fb (
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void ok_fb (
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void pub_markers ();

    bool srv_helper (geometry_msgs::Point & obj_center,
      geometry_msgs::Quaternion & obj_quat, geometry_msgs::Vector3 & obj_dims,
      bool ask_user_input, std::vector <uint8_t> & adjust_orientation);


  private:

    ros::NodeHandle nh_;

    std::string marker_frame_;
  
  
    // InteractiveMarkers

    // To spin interactive marker at same speed as main thread, so that things
    //   work
    int hertz_;

    std::string center_imarker_name_, dims_imarker_name_, ok_imarker_name_;
    float c_scale_, r_scale_;

    bool accept_user_input_;

    // obj_dims_ set by ros service request
    // obj_center_ set by user dragging interactive marker in RViz
    geometry_msgs::Vector3 obj_dims_;
    geometry_msgs::Point obj_center_;
    geometry_msgs::Quaternion obj_quat_;
    geometry_msgs::Vector3 prev_obj_dims_;

    ros::Publisher vis_pub_;
    visualization_msgs::Marker marker_obj_;

};


////////////////////////////////////////////////////////////////////// Ctor //

// Ctor
// NOTE frame should not have a preceding slash, else RViz won't plot.
IMarkerUtil::IMarkerUtil (ros::NodeHandle nh, int hertz,
  float c_scale=0.06, float r_scale=0.03,
  std::string marker_frame=std::string ("base"))
{
  // I think I found the hard way that nh_ has to stay in scope. Otherwise
  //   message subscribers die or something.
  nh_ = nh;
  hertz_ = hertz;

  vis_pub_ = nh_.advertise <visualization_msgs::Marker> (
    "visualization_marker", 0);
  marker_frame_ = marker_frame;


  ////////
  // Interactive Marker
  ////////

  center_imarker_name_ = "Object Center";
  dims_imarker_name_ = "Radius xyz";
  ok_imarker_name_ = "Done";

  // Center marker scale
  c_scale_ = c_scale; //0.1; //0.06;
  // Radii marker scale
  r_scale_ = r_scale; //0.07; //0.03;

  accept_user_input_ = false;

  obj_center_.x = 0.722;
  obj_center_.y = 0.247;
  obj_center_.z = -0.035;

  obj_quat_.w = 1.0;

  // These will be set in rosservice call anyway. Just show a cube to start
  obj_dims_.x = 0.08;
  obj_dims_.y = 0.08;
  obj_dims_.z = 0.08;

  // Put marker on top of object. Marker z is defined in center.
  float marker_z = obj_center_.z + (0.5 * MARKER_BOX_SIDE * MARKER_SCALE);
}


/////////////////////////////////////////////////////////////////// Getters //

// Return a copy, not a reference, so changes in caller does not change the
//   value in member field of this class!
geometry_msgs::Point IMarkerUtil::get_obj_center ()
{
  return obj_center_;
}

// Return a copy, not a reference, so changes in caller does not change the
//   value in member field of this class!
geometry_msgs::Vector3 IMarkerUtil::get_obj_dims ()
{
  return obj_dims_;
}

// Return a copy, not a reference, so changes in caller does not change the
//   value in member field of this class!
geometry_msgs::Quaternion IMarkerUtil::get_obj_quat ()
{
  return obj_quat_;
}

// Return a copy, not a reference, so changes in caller does not change the
//   value in member field of this class!
visualization_msgs::Marker IMarkerUtil::get_marker_obj ()
{
  return marker_obj_;
}

bool IMarkerUtil::get_accept_user_input ()
{
  return accept_user_input_;
}


/////////////////////////////////////////////////////////////////// Setters //

void IMarkerUtil::set_obj_center (geometry_msgs::Point c)
{
  obj_center_.x = c.x;
  obj_center_.y = c.y;
  obj_center_.z = c.z;
}

void IMarkerUtil::set_obj_quat (geometry_msgs::Quaternion q)
{
  obj_quat_.x = q.x;
  obj_quat_.y = q.y;
  obj_quat_.z = q.z;
  obj_quat_.w = q.w;
}

void IMarkerUtil::set_obj_dims (geometry_msgs::Vector3 d)
{
  obj_dims_.x = d.x;
  obj_dims_.y = d.y;
  obj_dims_.z = d.z;
}

void IMarkerUtil::set_prev_obj_dims (geometry_msgs::Vector3 dims)
{
  prev_obj_dims_.x = dims.x;
  prev_obj_dims_.y = dims.y;
  prev_obj_dims_.z = dims.z;
}

void IMarkerUtil::set_accept_user_input (bool val)
{
  accept_user_input_ = val;
}

///////////////////////////////////////////// Interactive Marker //

// Interactive Marker feedback function, for adjusting object bbox
//   before moving arm.
void IMarkerUtil::obj_center_fb (
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (accept_user_input_)
  {
    // Update interactive marker
    server -> applyChanges();

    // Update object center
    obj_center_.x = feedback -> pose.position.x;
    obj_center_.y = feedback -> pose.position.y;
    // Place obj at bottom of marker, else marker gets blocked and cannot
    //   be clicked on! MARKER_SCALE is diameter of marker, so half of it is
    //   0.5 * MARKER_SCALE.
    obj_center_.z = feedback -> pose.position.z - 
      (0.5 * MARKER_SCALE * MARKER_BOX_SIDE) - (0.5 * obj_dims_.z);
 
    obj_quat_.x = feedback -> pose.orientation.x;
    obj_quat_.y = feedback -> pose.orientation.y;
    obj_quat_.z = feedback -> pose.orientation.z;
    obj_quat_.w = feedback -> pose.orientation.w;


    // Publish a regular green marker for the updated object bbox, to follow
    //   user input
    // NOTE ROS Kinetic: Do NOT use a slash in namespace, otherwise RViz Marker
    //   won't get published!!!
    create_marker (visualization_msgs::Marker::CUBE, marker_frame_, 0,
      obj_center_.x, obj_center_.y, obj_center_.z,
      0, 1, 0, 0.5, obj_dims_.x, obj_dims_.y, obj_dims_.z,
      marker_obj_, "moveit_obj_bbox",
      obj_quat_.w, obj_quat_.x, obj_quat_.y, obj_quat_.z,
      0);
    vis_pub_.publish (marker_obj_);

    printf ("center: %.2f %.2f %.2f\n", obj_center_.x, obj_center_.y, obj_center_.z);
  }
  else
  {
    ROS_INFO ("Re Interactive Marker movement: not requesting obj bbox at the moment. Will not move object pose! To set object bbox again, wait till next iteration.");
  }
}


// Interactive Marker feedback function, for adjusting object bbox
//   before moving arm.
void IMarkerUtil::obj_dims_fb (
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (accept_user_input_)
  {
    // Update interactive marker
    server -> applyChanges();

    // Update object side lengths
    obj_dims_.x += (feedback -> pose.position.x - prev_obj_dims_.x);
    obj_dims_.y += (feedback -> pose.position.y - prev_obj_dims_.y);
    obj_dims_.z += (feedback -> pose.position.z - prev_obj_dims_.z);

    // Don't let side length go below 0! Marker cube will flip to show positive
    //   magnitude, but returned dimension is actually negative.
    obj_dims_.x = std::max (obj_dims_.x, 0.0);
    obj_dims_.y = std::max (obj_dims_.y, 0.0);
    obj_dims_.z = std::max (obj_dims_.z, 0.0);

    prev_obj_dims_.x = feedback -> pose.position.x;
    prev_obj_dims_.y = feedback -> pose.position.y;
    prev_obj_dims_.z = feedback -> pose.position.z;

    // Publish a regular green marker for the updated object bbox, to follow
    //   user input
    create_marker (visualization_msgs::Marker::CUBE, marker_frame_, 0,
      obj_center_.x, obj_center_.y, obj_center_.z,
      0, 1, 0, 0.5, obj_dims_.x, obj_dims_.y, obj_dims_.z,
      marker_obj_, "moveit_obj_bbox",
      obj_quat_.w, obj_quat_.x, obj_quat_.y, obj_quat_.z,
      0);
    vis_pub_.publish (marker_obj_);

    printf ("dims: %.2f %.2f %.2f\n", obj_dims_.x, obj_dims_.y, obj_dims_.z);
  }
  else
  {
    ROS_INFO ("Re Interactive Marker movement: not requesting obj bbox at the moment. Will not move object pose! To set object bbox again, wait till next iteration.");
  }
}


// User clicks on box to indicate done
void IMarkerUtil::ok_fb (
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (accept_user_input_)
  {
    // Update interactive marker
    server -> applyChanges();

    //if (feedback -> event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    //  fprintf (stderr, "Got a OK click\n");
    // This prints 1, which is POSE_UPDATE. Oh well. Just use it as ok anyway
    //fprintf (stderr, "event type: %d\n", feedback -> event_type);

    // Indicate to add_collision_srv that user is done adjusting markers
    accept_user_input_ = false;
  }
}


///////////////////////////////////////////////////////// Class utility fns //

void IMarkerUtil::pub_markers ()
{
  // Publish a regular marker for the updated object bbox, to follow user
  //   input
  create_marker (visualization_msgs::Marker::CUBE, marker_frame_, 0,
    obj_center_.x, obj_center_.y, obj_center_.z,
    0, 1, 0, 0.5, obj_dims_.x, obj_dims_.y, obj_dims_.z,
    marker_obj_, "moveit_obj_bbox",
    obj_quat_.w, obj_quat_.x, obj_quat_.y, obj_quat_.z,
    0);
  vis_pub_.publish (marker_obj_);


  // For testing if interactive marker is flexible enough!
  // Publish a reference object, to see if you can adjust the box to this
  //   object's size. This tests flexibility of the interactive markers!
  /*
  Marker marker_ref = Marker ();
  create_marker (visualization_msgs::Marker::CUBE, marker_frame_, 0,
    0.8, 0.2, 0.0,
    1, 1, 1, 0.5, 0.03, 0.06, 0.12,
    marker_ref, "/moveit_ref_bbox",
    1.0, 0.0, 0.0, 0.0,
    0);
  vis_pub_.publish (marker_ref);
  */
}


///////////////////////// Helper function for rosservice that use this file //

// Core functionality of adding InteractiveMarker in RViz to respond to a
//   service call to let human adjust a box's center, dimensions, orientation.
// This function is only responsible for adjusting the box via InteractiveMarker
//   and updating the regular Marker. Everything else should be done in the
//   caller, so that this fn can be reused for many different services e.g.
//   MoveIt collision box, grasping region and pose request, etc.
// Ref: Bool in rosmsg is uint8_t type in C++, `.` std::vector<bool> is a
//   specialized type that's not a container! So use std::vector<uint8_t>.
//   http://wiki.ros.org/msg
bool IMarkerUtil::srv_helper (geometry_msgs::Point & obj_center,
  geometry_msgs::Quaternion & obj_quat, geometry_msgs::Vector3 & obj_dims,
  bool ask_user_input, std::vector <uint8_t> & adjust_orientation)
{
  fprintf (stderr, "Initializing interactive markers at object pose and dimensions: "
    "p %.2f %.2f %.2f, q %.2f %.2f %.2f %.2f, s %.2f %.2f %.2f\n",
    obj_center.x, obj_center.y, obj_center.z,
    obj_quat.x, obj_quat.y, obj_quat.z, obj_quat.w,
    obj_dims.x, obj_dims.y, obj_dims.z);

  // Publish a regular marker for the updated object bbox, to follow user
  //   input.
  // During user interaction, the InteractiveMarker feedback function _fb()
  //   in interactive_marker_util.h will update the regular Marker to reflect
  //   user's adjustment.
  create_marker (visualization_msgs::Marker::CUBE, marker_frame_, 0,
    obj_center.x, obj_center.y, obj_center.z,
    0, 1, 0, 0.5, obj_dims.x, obj_dims.y, obj_dims.z,
    marker_obj_, "moveit_obj_bbox",
    obj_quat.w, obj_quat.x, obj_quat.y, obj_quat.z,
    0);
  vis_pub_.publish (marker_obj_);


  // Update InteractiveMarkers
  this -> set_obj_center (obj_center);
  this -> set_obj_quat (obj_quat);
  this -> set_obj_dims (obj_dims);

  // If service caller wanted manual user adjustment, listen to interactive
  //   markers
  if (ask_user_input)
  {
    // Sanity check, else seg fault below when access adjust_orientation[0:2]!
    if (adjust_orientation.size () < 3)
    {
      fprintf (stderr, "%sWARN in srv_helper(): adjust_orientation not specified! Must be specified when ask_user_input==true! Will use default 1 m x 1 m x 1 m\n%s", WARN, ENDC);
      adjust_orientation.clear ();
      adjust_orientation.push_back (1);
      adjust_orientation.push_back (1);
      adjust_orientation.push_back (1);
    }

    // Allow user to make changes in this interval
    this -> set_accept_user_input (true);
 
    // Put here, not in ctor, `.` then in each service call, the interactive
    //   markers' positions can be corrected to correct object's location and
    //   size! If put in ctor, I have to update it myself, and it led to some
    //   errors in dimensions control.
    InteractiveMarker center_imarker, dims_imarker, ok_imarker;
 

    // For object center

    // Note that in order to put this in ctor, you must initialize interactive
    //   marker with server->reset() BEFORE instantiating an object of this
    //   class!
    //make6DofMarker (visualization_msgs::InteractiveMarkerControl::NONE,
    // Try full 6DOF translation and rotation
    //   MOVE_ROTATE_3D in http://wiki.ros.org/interaction_cursor_rviz
    //   3D Controls in http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls
    make6DofMarker (visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
      center_imarker_name_,
      // http://stackoverflow.com/questions/5245072/pass-and-call-a-member-function-boostbind-boostfunction
      boost::bind (&IMarkerUtil::obj_center_fb, this, _1), center_imarker,
      tf::Vector3 (obj_center.x, obj_center.y,
        obj_center.z + 0.5 * obj_dims.z + 0.5 * c_scale_ - 0.25 * c_scale_),
      true, true, true,
      adjust_orientation [0], adjust_orientation [1], adjust_orientation [2],
      c_scale_, marker_frame_);


    // For object dimensions

    // 0.25 is the length of the arrows you must account for too!
    geometry_msgs::Vector3 prev_obj_dims;
    prev_obj_dims.x = 
      obj_center.x - 0.5 * obj_dims.x - 0.5 * r_scale_ + 0.25 * r_scale_;
    prev_obj_dims.y = 
      obj_center.y - 0.5 * obj_dims.y - 0.5 * r_scale_ + 0.25 * r_scale_;
    prev_obj_dims.z = 
      obj_center.z + 0.5 * obj_dims.z + 0.5 * r_scale_ - 0.25 * r_scale_;
    this -> set_prev_obj_dims (prev_obj_dims);
   
    // A single dims marker, at a corner of obj bbox. Moving this marker
    //   in xyz will change object bbox's side lengths in xyz! Easiest way
    //   I came up with to change dimensions of bbox using just 1 marker.
    //make6DofMarker (visualization_msgs::InteractiveMarkerControl::NONE,
    make6DofMarker (visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
      dims_imarker_name_,
      // http://stackoverflow.com/questions/5245072/pass-and-call-a-member-function-boostbind-boostfunction
      boost::bind (&IMarkerUtil::obj_dims_fb, this, _1), dims_imarker,
      tf::Vector3 (prev_obj_dims.x, prev_obj_dims.y, prev_obj_dims.z),
      true, true, true,
      false, false, false, r_scale_, marker_frame_);
 

    // For "OK" to indicate done setting
 
    make6DofMarker (visualization_msgs::InteractiveMarkerControl::BUTTON,
      ok_imarker_name_,
      // http://stackoverflow.com/questions/5245072/pass-and-call-a-member-function-boostbind-boostfunction
      boost::bind (&IMarkerUtil::ok_fb, this, _1), ok_imarker,
      tf::Vector3 (obj_center.x, obj_center.y + 1.0, 1.0),
      false, false, false,
      false, false, false, c_scale_, marker_frame_);
 
    server -> applyChanges ();
    sleep (3.0);

    // DEBUG
    ROS_INFO ("InteractiveMarker published");
 
 
    fprintf (stderr, "%s""Use Interactive Markers in RViz to specify object CENTER and RADII. Click on the Done marker when done.\n""%s", OKCYAN, ENDC);
    // Must use a loop to spinOnce(), cannot just call sleep(10.0), `.`
    //   interactive marker feedback function is not listening when this
    //   rosservice is blocking!
    //   Also sleep() doesn't seem to take <1 args. It just returns immediately
    //int nloops = hertz_ * 15;  // 15 secs
    ros::Rate wait_rate (hertz_);
    while (ros::ok ())
    {
      ros::spinOnce ();
      wait_rate.sleep ();
 
      if (! ros::ok ())
        break;
      if (! this -> get_accept_user_input ())
        break;
    }

    // Disable after this interval
    this -> set_accept_user_input (false);
  }

  // Return updated values in parameters
  // Fetch the final values updated by user via InteractiveMarker
  obj_center = this -> get_obj_center ();
  obj_quat = this -> get_obj_quat ();
  obj_dims = this -> get_obj_dims ();

  fprintf (stderr, "Final interactive markers are at user-requested object pose and dimensions: "
    "p %.2f %.2f %.2f, q %.2f %.2f %.2f %.2f, s %.2f %.2f %.2f\n",
    obj_center.x, obj_center.y, obj_center.z,
    obj_quat.x, obj_quat.y, obj_quat.z, obj_quat.w,
    obj_dims.x, obj_dims.y, obj_dims.z);
}


#endif

