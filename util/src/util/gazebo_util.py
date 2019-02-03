#!/usr/bin/env python

# Mabel Zhang
# 16 Feb 2018
#
# Gazebo utility functions
#

import rospy
from geometry_msgs.msg import Vector3, Twist

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState, SetModelState, SetModelStateRequest
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsPropertiesRequest, \
  SetPhysicsProperties

from util.ansi_colors import ansi_colors


def gravity_on (on=True):

  # Get current physics properties
  get_srv_name = '/gazebo/get_physics_properties'
  print 'Waiting for ROS service %s...' % get_srv_name
  rospy.wait_for_service (get_srv_name)

  try:
    get_srv = rospy.ServiceProxy (get_srv_name, GetPhysicsProperties)
    get_resp = get_srv ()
  except rospy.ServiceException, e:
    print ('Service initialization failed: %s' % e)
    return False

  if not get_resp.success:
    print (get_resp.status_message)
    return False


  # Copy current physics properties, except gravity, so that we don't change
  #   anything else
  set_req = SetPhysicsPropertiesRequest ()
  set_req.time_step = get_resp.time_step
  set_req.max_update_rate = get_resp.max_update_rate
  set_req.ode_config = get_resp.ode_config

  if on:
    set_req.gravity.z = -9.8
  else:
    set_req.gravity.z = 0


  # Set physics properties, everything except gravity is same as current state
  set_srv_name = '/gazebo/set_physics_properties'
  print 'Waiting for ROS service %s...' % set_srv_name
  rospy.wait_for_service (set_srv_name)

  try:
    set_srv = rospy.ServiceProxy (set_srv_name, SetPhysicsProperties)
    set_resp = set_srv (set_req)
  except rospy.ServiceException, e:
    print ('Service initialization failed: %s' % e)
    return False

  if not set_resp.success:
    print (set_resp.status_message)
    return False

  return True


def gravity_off ():

  return (gravity_on (False))


#   model_type: 'gazebo' 'sdf' or 'urdf', used in name of rosservice! Must be
#     exact.
def spawn_model (model_name, model_xml, robot_ns, init_pose, ref_frame='world',
  model_type='sdf'):

  print ('%sSpawning %s in Gazebo world%s' % (
    ansi_colors.OKCYAN, model_name, ansi_colors.ENDC))

  # Options:
  #   /gazebo/spawn_gazebo_model
  #   /gazebo/spawn_sdf_model
  #   /gazebo/spawn_urdf_model
  spawn_srv_name = '/gazebo/spawn_%s_model' % model_type

  print 'Waiting for ROS service %s...' % spawn_srv_name
  rospy.wait_for_service (spawn_srv_name)

  spawn_req = SpawnModelRequest ()
  # Spawned with name 'robotiq' in ../launch/spawn_robotiq.launch
  spawn_req.model_name = model_name
  spawn_req.model_xml = model_xml
  spawn_req.robot_namespace = robot_ns

  # Must spawn at 0 0 0, `.` robot_state_publisher does not have access to
  #   this pose, so tf of hand will not include this pose!! So it needs to
  #   be 0, else tf of hand will be wrong.
  spawn_req.initial_pose = init_pose
  spawn_req.reference_frame = ref_frame

  try:
    spawn_srv = rospy.ServiceProxy (spawn_srv_name, SpawnModel)
    resp = spawn_srv (spawn_req)
  except rospy.ServiceException, e:
    print ("Service initialization failed: %s" % e)
    return False

  if not resp.success:
    print (resp.status_message)
    return False
  else:
    print 'Sleeping 1 s to wait for Gazebo spawn model...'
    rospy.sleep(1)
    return True


def delete_model (model_name):

  print ('%sRemoving %s from Gazebo world%s' % (
    ansi_colors.OKCYAN, model_name, ansi_colors.ENDC))

  delete_srv_name = '/gazebo/delete_model'

  print 'Waiting for ROS service %s...' % delete_srv_name
  rospy.wait_for_service (delete_srv_name)

  try:
    delete_srv = rospy.ServiceProxy (delete_srv_name, DeleteModel)
  except rospy.ServiceException, e:
    print ("Service initialization failed: %s" % e)


  resp = delete_srv (model_name)
  if not resp.success:
    print (resp.status_message)
  else:
    print 'Sleeping 1 s to wait for Gazebo delete model...'
    rospy.sleep(1)


# Not tested!
# Equivalent to:
#   $ rosservice call /gazebo/get_model_state "{model_name: robotiq, relative_entity_name: world}"
# Parameters:
#   model_name, relative_entity_name: strings
def get_model_state (model_name, relative_entity_name='world'):

  print ('Getting Gazebo model state for %s...' % model_name)

  get_srv_name = '/gazebo/get_model_state'
  print 'Waiting for ROS service %s...' % get_srv_name
  rospy.wait_for_service (get_srv_name)

  try:
    get_srv = rospy.ServiceProxy (get_srv_name, GetModelState)
  except rospy.ServiceException, e:
    print ("Service initialization failed: %s" % e)

  resp = get_srv (model_name, relative_entity_name)
  if not resp.success:
    print (resp.status_message)
  else:
    print 'Sleeping 1 s to wait for Gazebo delete model...'
    rospy.sleep(1)

  return resp


def set_model_state (model_name, pose, frame_id='world', twist=Twist()):

  print ('Setting Gazebo model state for %s...' % model_name)

  set_srv_name = '/gazebo/set_model_state'
  print 'Waiting for ROS service %s...' % set_srv_name
  rospy.wait_for_service (set_srv_name)

  try:
    set_srv = rospy.ServiceProxy (set_srv_name, SetModelState)
  except rospy.ServiceException, e:
    print ("Service initialization failed: %s" % e)

  # Populate message
  model_state = ModelState ()
  model_state.model_name = model_name
  model_state.pose = pose
  model_state.twist = twist
  model_state.reference_frame = frame_id

  resp = set_srv (model_state)
  if not resp.success:
    print (resp.status_message)
  else:
    print 'Sleeping 1 s to wait for Gazebo delete model...'
    rospy.sleep(1)

  return resp


# Assumption: model_xml contains a <static> tag that's either false or true
#   already. If not, none will be added (you need XML parser for that, a
#   simple string parser won't be enough).
def xml_replace_static (model_xml, static):

  if static:
    static_str = 'true'
  else:
    static_str = 'false'

  ORIG_FALSE = '<static>false</static>'
  ORIG_TRUE = '<static>true</static>'

  # if want to set to true
  if static:
    # and originally it was false
    # (else nothing to do)
    if model_xml.find (ORIG_FALSE) != -1:
      # Set to true
      model_xml = model_xml.replace (ORIG_FALSE, ORIG_TRUE)
  else:
    if model_xml.find (ORIG_TRUE) != -1:
      model_xml = model_xml.replace (ORIG_TRUE, ORIG_FALSE)

  return model_xml

