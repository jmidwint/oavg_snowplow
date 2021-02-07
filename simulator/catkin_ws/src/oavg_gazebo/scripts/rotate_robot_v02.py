#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3Stamped
import math

robot_roll = robot_pitch = robot_yaw = 0.0
target = 90
target_rad = math.pi / 2
#kp=0.1
kp=0.5
Vkp = 0.1
#origin_lat_deg = 49.9
#origin_long_deg = 8.9
origin_lat_deg = 45.407869
origin_long_deg = -75.719062
R = 6371000

target_lat_deg = 45.4078690286
target_long_deg = -75.7190107607
target_lat_rad = target_lat_deg * math.pi / 180
target_long_rad = target_long_deg * math.pi / 180

target_x = target_y = target_bearing = target_distance = 0

def get_gps_position (msg):
    global robot_roll, robot_pitch, robot_yaw
    global origin_lat_deg, origin_long_deg
    global target_x, target_y, target_bearing, target_distance
    #orientation_q = msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    lat_deg = msg.latitude
    long_deg = msg.longitude
    lat_rad = msg.latitude * math.pi / 180
    long_rad = msg.longitude * math.pi / 180
    origin_lat_rad = origin_lat_deg * math.pi / 180
    origin_long_rad = origin_long_deg * math.pi / 180

    delta_long_rad = long_rad - origin_long_rad
    delta_lat_rad = lat_rad - origin_lat_rad

    #x = math.cos(lat_rad) * math.sin(delta_long_rad)
    #y = math.cos(origin_lat_rad)*math.sin(lat_rad) - math.sin(origin_lat_rad)*math.cos(lat_rad)*math.cos(delta_long_rad)

    x = delta_long_rad * math.cos((origin_lat_rad + lat_rad)/2) * R
    y = delta_lat_rad * R

    #y = -(delta_long_rad * math.cos((origin_lat_rad + lat_rad)/2) * R)
    #x = delta_lat_rad * R

    target_x =  (target_long_rad-long_rad) * math.cos((target_lat_rad + lat_rad)/2) * R
    target_y = (target_lat_rad - lat_rad) * R

    #yaw = math.atan2(msg.vector.y, msg.vector.x)
    target_bearing = math.atan2(target_x, target_y)
    target_distance = math.sqrt((target_x*target_x)+(target_y*target_y))


    #print yaw
    print "X: " + str(x) + "Y: " + str(y) + "Yaw : " + str(robot_yaw * 180 / math.pi) + "TgtYaw : " + str(target_bearing * 180 / math.pi) + "lat: " + str(lat_deg) + "lon: " + str(long_deg)


def get_rotation (msg):
    global robot_roll, robot_pitch, robot_yaw
    #orientation_q = msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    robot_yaw = math.atan2(msg.vector.y, msg.vector.x) 
    
    #print yaw

rospy.init_node('rotate_robot')

#sub = rospy.Subscriber ('/oavg_diff_drive_controller/odom', Odometry, get_rotation)
sub = rospy.Subscriber ('/oavg/fix', NavSatFix, get_gps_position)
sub = rospy.Subscriber ('/oavg/magnetic', Vector3Stamped, get_rotation)
pub = rospy.Publisher('/oavg_diff_drive_controller/cmd_vel', Twist, queue_size=1)
#r = rospy.Rate(10)
r = rospy.Rate(10)
command =Twist()

while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    #target_rad = target*math.pi/180
    target_err = target_bearing-robot_yaw
    if (target_err < -math.pi):
        target_err = target_err + (2 * math.pi)
    elif (target_err > math.pi):
        target_err = target_err - (2 * math.pi)

    # ROS/Gazebo has positive angles counter-clockwise
    # Bearing angle is positive clock-wise
    # Need to negate target_err (bearing) to convert to ROS/Gazebo reference frame
    command.angular.z = kp * (-target_err)

    if (target_distance > 0.2):
        command.linear.x = Vkp * (target_distance)
    else:
        command.linear.x = 0

    pub.publish(command)
    print("target={} current:{}", (target_err)*180/math.pi)
    r.sleep()
