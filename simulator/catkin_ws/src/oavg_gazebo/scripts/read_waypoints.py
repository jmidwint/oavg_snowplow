

# test read waypoints

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
newList = read ("path.file")  # read path file 
print(newList)

print("Num waypoints: " + str (len(newList)))

