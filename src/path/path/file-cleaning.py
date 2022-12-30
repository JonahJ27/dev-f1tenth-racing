#!/usr/bin/env python3
import sys
from os.path import expanduser

def main(args=sys.argv):
    home = expanduser("~")
    file = open(home + "/dev_f1tenth_ws/src/path/waypoint-files/" + sys.argv[1], "r")

    #remove_dupes(file, home, sys.argv[2])
    #remove_close_waypoints(file, home, sys.argv[2])
    remove_every_other_waypoint(file, home, sys.argv[2])


def remove_dupes(file, home, out_file):
    line_map = {}

    for line in file.readlines():
        if not line in line_map:
            line_map[line] = True

    f = open(home + "/dev_f1tenth_ws/src/path/waypoint-files/" + out_file, "w")
    for l in line_map.keys():
        f.write(l)

    file.close()
    f.close()

def remove_close_waypoints(file, home, out_file):
    line_map = {}
    waypoints = []

    for line in file.readlines():
        vals = line.split(",")
        if  (len(waypoints) == 0 or
            (abs(waypoints[0][0] - float(vals[0])) > 0.0001 
            and abs(waypoints[0][1] - float(vals[1])) > 0.0001)):
            waypoints.insert(0, [float(vals[0]), float(vals[1])])
            line_map[line] = True

    f = open(home + "/dev_f1tenth_ws/src/path/waypoint-files/" + out_file, "w")
    for l in line_map.keys():
        f.write(l)

    print("Spaced")
    file.close()
    f.close()

def remove_every_other_waypoint(file, home, out_file):
    line_map = {}

    add = True
    for line in file.readlines():
        if add:
            line_map[line] = True
        add = not add

    f = open(home + "/dev_f1tenth_ws/src/path/waypoint-files/" + out_file, "w")
    for l in line_map.keys():
        f.write(l)

    file.close()
    f.close()

        

if __name__ == '__main__':
    main()