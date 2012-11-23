#!/usr/bin/env python
#
# Reads a file of AR tag locations (in the file format specified by the 
# ar_filter software), and produces a list of locations to steer towards.
# Each location is about a meter in front of the relevant tag.  This is 
# pretty klugy, but maybe it will work.  We can also use the tag locations
# to establish repellers at each one to keep the robots from knocking them
# over.
import sys
import math

def read_location_file(locfile):
    with open(locfile, 'r') as locationFile:
        locations = []
        for line in locationFile:
            if line.startswith("#"):
                continue
            tagInfo = line.split()
            if len(tagInfo) != 4:
                continue
            locations.append([float(tagInfo[1]) + math.cos(float(tagInfo[3])),
                              float(tagInfo[2]) + math.sin(float(tagInfo[3])),
                              float(tagInfo[3]) - math.pi])

        return locations

if len(sys.argv) > 1:
    l = read_location_file(sys.argv[1])
    print l
