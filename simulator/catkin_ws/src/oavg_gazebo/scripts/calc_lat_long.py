import math
import csv


#R = 6378.1 #Radius of the Earth
R = 6371000 #Radius of the Earth

brng = 1.57 #Bearing is 90 degrees converted to radians.
d = .004 #Distance in km

latlon_pt = {"lat": 0.0, "lon": 0.0}

print(latlon_pt)

#lat2  52.20444 - the lat result I'm hoping for
#lon2  0.36056 - the long result I'm hoping for.

#lat1 = math.radians(52.20472) #Current lat point converted to radians
#lon1 = math.radians(0.14056) #Current long point converted to radians
lat1 = math.radians(45.407869) #Current lat point converted to radians
lon1 = math.radians(-75.719062) #Current long point converted to radians

def calc_lat_lon(brng, d):
  global latlon_pt

  lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
       math.cos(lat1)*math.sin(d/R)*math.cos(brng))

  lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
             math.cos(d/R)-math.sin(lat1)*math.sin(lat2))

  lat2 = math.degrees(lat2)
  lon2 = math.degrees(lon2)

  latlon_pt['lat'] = lat2
  latlon_pt['lon'] = lon2

  print("brng: " + str(brng*180/math.pi) )
  print("lat: " + str(lat2) )
  print("lon: " + str(lon2) )


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

f_read = open('target_xy.csv', 'r')
reader = csv.reader(f_read)
xy_data = list(reader)
print("Num rows: " + str(len(xy_data)))
for row in xy_data:
    print("x: " + str(row[0]) + " y: " + str(row[1]))


f_write = open('target_latlon.csv', 'wb')
writer = csv.writer(f_write)

pathList = []

for i in range(len(xy_data)):
  x = float (xy_data[i][0])
  y = float (xy_data[i][1])
  dist = math.sqrt((x*x)+(y*y))
  bearing = math.atan2(x, y)
  calc_lat_lon( bearing, dist)

  #calc_lat_lon( (i+1)*brng, d)

  writer.writerow([latlon_pt['lat'], latlon_pt['lon']])
  pathList.append([latlon_pt['lat'], latlon_pt['lon']])

write (pathList,"path_square.file")
newList = read ("path_square.file")  # read path file 
print(newList)

f_read.close()
f_write.close()

