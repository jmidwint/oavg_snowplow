import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
import math
import time


def get_waypoints(path_file):
    global latlon_data
    newList = read (path_file)  # read path file 
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

origin_lat_deg = 45.407869
origin_long_deg = -75.719062
R = 6371000

def get_latlon_position (lat_deg, long_deg):
    global robot_roll, robot_pitch, robot_yaw
    global origin_lat_deg, origin_long_deg
    global target_x, target_y, target_bearing, target_distance
    #orientation_q = msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #lat_deg = msg.latitude
    #long_deg = msg.longitude
    lat_rad = lat_deg * math.pi / 180
    long_rad = long_deg * math.pi / 180
    origin_lat_rad = origin_lat_deg * math.pi / 180
    origin_long_rad = origin_long_deg * math.pi / 180

    delta_long_rad = long_rad - origin_long_rad
    delta_lat_rad = lat_rad - origin_lat_rad

    #x = math.cos(lat_rad) * math.sin(delta_long_rad)
    #y = math.cos(origin_lat_rad)*math.sin(lat_rad) - math.sin(origin_lat_rad)*math.cos(lat_rad)*math.cos(delta_long_rad)

    x = delta_long_rad * math.cos((origin_lat_rad + lat_rad)/2) * R
    y = delta_lat_rad * R

    return x, y



latlon_data = []

path_file = "path.file"
#path_file = "path_square.file"

get_waypoints(path_file)

print("Latlon data: ")
print(latlon_data)

num_targets = len(latlon_data)
target_num = 0

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
p = Pose()
i = 0

print("Num rows: " + str(num_targets))
for row in latlon_data:
    print("lat: " + str(row[0]) + " lon: " + str(row[1]))
    x, y = get_latlon_position(row[0], row[1])

    p.position.x = x
    p.position.y = y
    p.position.z = 0
    # Make sure the quaternion is valid and normalized
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 1.0

    spawn_model_client(
        model_name='wood_cube'+str(i), 
        model_xml=open('../models/wood_cube_10cm/model.sdf', 'r').read(),
        robot_namespace='/oavg',
        initial_pose=p,
        reference_frame='world'
    )

    i = i+1

    #time.sleep(0.1)