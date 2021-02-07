#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3Stamped
import math
from enum import Enum
import csv

class NavState(Enum):
     INIT = 1
     NAV_TARGET = 2
     AT_TARGET = 3
     PAUSE_TARGET = 4
     NEXT_TARGET = 5
     DONE = 99

current_state = NavState.INIT

robot_roll = robot_pitch = robot_yaw = 0.0
robot_speed = 0.0
target = 90
target_rad = math.pi / 2
#kp=0.1
kp=1.0  #0.8  #0.5
Vkp = 0.5  #0.3   #0.2   #0.1
#origin_lat_deg = 49.9
#origin_long_deg = 8.9
origin_lat_deg = 45.407869
origin_long_deg = -75.719062
R = 6371000


target_latlon_deg = {}
target_latlon_rad = {}


target_lat_deg = 45.4078690286
target_long_deg = -75.7190107607
target_lat_rad = target_lat_deg * math.pi / 180
target_long_rad = target_long_deg * math.pi / 180

target_x = target_y = target_bearing = target_distance = target_err = 0

max_speed = 0.8  #1.0  #0.7
min_speed = 0.2  #0.3

min_target_distance = 0.3

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
    print "X: {0:.2f}".format(x) + "Y: {0:.2f}".format(y) + "Yaw : {0:.2f}".format(robot_yaw * 180 / math.pi) + "TgtYaw : {0:.2f}".format(target_bearing * 180 / math.pi) + \
          "lat: " + str(lat_deg) + "lon: " + str(long_deg)


def get_rotation (msg):
    global robot_roll, robot_pitch, robot_yaw
    #orientation_q = msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    robot_yaw = math.atan2(msg.vector.y, msg.vector.x) 
    
    #print yaw


def run_state(current_state):
    global target_num, latlon_data
    global target_lat_deg, target_long_deg, target_lat_rad, target_long_rad
    global target_bearing, target_distance, target_err
    global robot_roll, robot_pitch, robot_yaw, robot_speed
    global command
    target_err = 0

    print ("State: " + current_state.name)

    #switch (current_state) {
    if current_state == NavState.INIT:  
        target_lat_deg = float(latlon_data[target_num][0])
        target_long_deg = float(latlon_data[target_num][1])
        target_lat_rad = target_lat_deg * math.pi / 180
        target_long_rad = target_long_deg * math.pi / 180
        next_state = NavState.NAV_TARGET
      
    elif current_state == NavState.NAV_TARGET:
        target_err = target_bearing-robot_yaw
        if (target_err < -math.pi):
            target_err = target_err + (2 * math.pi)
        elif (target_err > math.pi):
            target_err = target_err - (2 * math.pi)

        # ROS/Gazebo has positive angles counter-clockwise
        # Bearing angle is positive clock-wise
        # Need to negate target_err (bearing) to convert to ROS/Gazebo reference frame
        command.angular.z = kp * (-target_err)

        if (target_distance > min_target_distance):
            robot_speed = Vkp * (target_distance)
            if (robot_speed > max_speed):
                robot_speed = max_speed
            elif (robot_speed < min_speed):
                robot_speed = min_speed
            command.linear.x = robot_speed
            next_state = current_state
        else:
            command.linear.x = 0
            next_state = NavState.AT_TARGET

        #print("target={} current:{}", (target_err)*180/math.pi)

        print("tgt lat: " + str(target_lat_deg) + " lon: " + str(target_long_deg) + " err: {:.2f}".format((target_err)*180/math.pi) + \
              " dist: {0:.2f}".format(target_distance) + " spd: {0:.2f}".format(robot_speed) + " tgt#: " + str(target_num) ) 
           
    elif current_state == NavState.AT_TARGET:

        next_state = NavState.PAUSE_TARGET


    elif current_state == NavState.PAUSE_TARGET:

        next_state = NavState.NEXT_TARGET


    elif current_state == NavState.NEXT_TARGET:

        target_num = target_num + 1
        if (target_num < num_targets):
            # go to next target
            target_lat_deg = float(latlon_data[target_num][0])
            target_long_deg = float(latlon_data[target_num][1])
            target_lat_rad = target_lat_deg * math.pi / 180
            target_long_rad = target_long_deg * math.pi / 180
            next_state = NavState.NAV_TARGET
        else:
            # All targets reached, go to DONE state
            next_state = NavState.DONE


    elif current_state == NavState.DONE:
        # Navigation completed
        next_state = NavState.DONE

    else:
        print("Unkown state")

    return next_state


def get_waypoints():
    global latlon_data
    newList = read ("path.file")  # read path file 
    print(newList)
    latlon_data = newList
    
    print("Num waypoints: " + str (len(latlon_data)))

# Dump pathList in a file
import pickle

def write(data, outfile):
        f = open(outfile, "wb")
        pickle.dump(data, f)
        f.close()

def read(filename):
        f = open(filename,"rb")
        data = pickle.load(f)
        f.close()
        return data

# write (pathList,"path.file")	# write waypoints path to a file




rospy.init_node('nav_robot')

#sub = rospy.Subscriber ('/oavg_diff_drive_controller/odom', Odometry, get_rotation)
sub = rospy.Subscriber ('/oavg/fix', NavSatFix, get_gps_position)
sub = rospy.Subscriber ('/oavg/magnetic', Vector3Stamped, get_rotation)
pub = rospy.Publisher('/oavg_diff_drive_controller/cmd_vel', Twist, queue_size=1)
#r = rospy.Rate(10)
r = rospy.Rate(10)
command =Twist()
current_state = NavState.INIT


# f_read = open('target_latlon.csv', 'r')
# reader = csv.reader(f_read)
# latlon_data = list(reader)

latlon_data = []

get_waypoints()

print("Latlon data: ")
print(latlon_data)

num_targets = len(latlon_data)
target_num = 0

print("Num rows: " + str(num_targets))
for row in latlon_data:
    print("lat: " + str(row[0]) + " lon: " + str(row[1]))

while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    #target_rad = target*math.pi/180
    current_state = run_state(current_state)

    if (current_state == NavState.DONE):
        break

    pub.publish(command)
    r.sleep()
