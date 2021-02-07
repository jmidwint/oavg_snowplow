#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
p = Pose()
p.position.x = 4
p.position.y = 0
p.position.z = 0
# Make sure the quaternion is valid and normalized
p.orientation.x = 0.0
p.orientation.y = 0.0
p.orientation.z = 0.0
p.orientation.w = 1.0
spawn_model_client(
    #model_name='construction_cone',
    #model_xml=open('/usr/share/gazebo-9/models/construction_cone/model.sdf', 'r').read(),

    model_name='wood_cube', 
    #model_xml=open('/home/paulh/.gazebo/models/wood_cube_10cm/model.sdf', 'r').read(),
    model_xml=open('../models/wood_cube_10cm/model.sdf', 'r').read(),

    robot_namespace='/oavg',
    #initial_pose=Pose({"position": "{ x: 4, y: 0, z: 0 }"}, {"orientation": "{x: 0, y: 0, z: 0, w: 0}"} ),
    #initial_pose=Pose('{ "x": 4, "y": 0, "z": 0 }", "{"x": 0, "y": 0, "z": 0, "w": 0}' ),
    #initial_pose=Pose({ 4, 0, 0 }, { 0, 0, 0, 1 }),
    #initial_pose=Pose(position={"x": 4, "y": 0, "z": 0 }, orientation={ 0, 0, 0, 1 }),
    #initial_pose=Pose({ "position.x": 4, "position.y": 0, "position.z": 0 }, {"orientation.x": 0, "orientation.y": 0, "orientation.z": 0, "orientation.w": 0} ),
    #initial_pose=Pose( position.x=4, position.y=0, position.z=0, orientation.x=0, orientation.y=0, orientation.z=0, orientation.w=0),
    #initial_pose=Pose( x=4, y=0, z=0, x=0, y=0, z=0, w=1),
    #initial_pose=Pose( x=4, y=0, z=0,  w=1),


    initial_pose=p,
    reference_frame='world'
)
