#
#  Calculate navigation path for snowplow and simulation
#  By Ayman Hindam   ahindam@hinnovac.com
#
# Calculate navigation path for snowplow based on area define by 4 coordinates points. Using geodesics calculation.
#
#  Install python geographiclib: 
#	$ pip install geographiclib
#
# Calculate path waypoints
#
#  1) Get waypoints every 1 meter (~ snowplow blade width) between point1 and point 4 in wp14 
#  2) Get waypoints every 1 meter (~ snowplow blade width) between point2 and point 3 in wp23 
#  3) define the rover path as explain in documentation: wp14[0] -->wp23[0] -->wp23[1] --> wp14[1] --> wp14[2]...
#

print("Start Waypoints calculation")

# Initialize Geodetic library
from geographiclib.geodesic import Geodesic
import math
geod = Geodesic.WGS84  # define the WGS84 ellipsoid to model the Earth in the UTM coordinate system.


ds = 1 #is the distance between waypoints which correspond to blade width We use 1 meter as the blade is 110cm. 

# Enter  4 boundries points (TICS) 
lat1= 45.407869
lon1= -75.719062

lat2= 45.407841
lon2= -75.719173

lat3= 45.407939
lon3= -75.719234

lat4= 45.407967
lon4= -75.719127

wp14 = []
wp23 = []

# Calculate intermediate points, each one meter(Snowblade length), between 2 points 
def waypoints(wp,lat1,lon1,lat2,lon2):
    l = geod.InverseLine(lat1, lon1, lat2, lon2)
    n = int(math.ceil(l.s13 / ds))
    for i in range(n+1):
        if i == 0:
            print("distance latitude longitude")
        s = min(ds * i, l.s13)
        g = l.Position(s, Geodesic.STANDARD | Geodesic.LONG_UNROLL)
        wp.append([g['lat2'], g['lon2']])
        print("{:.0f} {:.6f} {:.6f}".format(g['s12'], g['lat2'], g['lon2']))
        #print(wp)

# Calculate Waypoints
print("Waypoints 1 - 4")
waypoints(wp14,lat1,lon1,lat4,lon4)
print ("Array Length: {} \n".format(len(wp14)))

print("Waypoints 2 - 3")
waypoints(wp23,lat2,lon2,lat3,lon3)
print ("Array Length: {} \n".format(len(wp23)))


# Create Navigation path coordinates list based on waypoints
print("Navigation path coordinates: ")
count14 = 0
count23 = 0
is14 = True
pathList = []
for i in range(min(len(wp14),len(wp23))):
	if is14:
		if count14 != 0:
			pathList.append(wp14[count14])
			print("WP14-{}  {:.6f} , {:.6f}".format(count14, wp14[count14][0], wp14[count14][1]))
			count14 += 1
 
		pathList.append(wp14[count14])
		print("WP14-{}  {:.6f} , {:.6f}".format(count14, wp14[count14][0], wp14[count14][1]))
		count14 += 1
		is14 = False
		
	else:
		pathList.append(wp23[count23])
		print("WP23-{}  {:.6f} , {:.6f}".format(count23, wp23[count23][0], wp23[count23][1]))
		count23 +=1
		pathList.append(wp23[count23])
		print("WP23-{}  {:.6f} , {:.6f}".format(count23, wp23[count23][0], wp23[count23][1]))
		count23 +=1
		is14 = True


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

write (pathList,"path.file")	# write waypoints path to a file
# newList = read ("path.file")  # read path file 
# print(newList)


