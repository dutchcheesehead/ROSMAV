#!/usr/bin/env python
import os

#cam_num = 0.0
#cam_coll = 0
#cam_total = 0
#tag_num = 0.0
#tag_coll = 0
#tag_total = 0

#cam_out = open('analyzed_data/cam_total_time.dat','w')
#tag_out = open('analyzed_data/tag_total_time.dat','w')

cam_duration_total = 0.0
cam_collision_total = 0.0
tag_duration_total = 0.0
tag_collision_total = 0.0
cam_durations = []
tag_durations = []
cam_colls = []
tag_colls = []

for root, dirs, files in os.walk('data'):
    for name in files:
        f = open(os.path.join(root, name))
        lines = f.readlines()

        if not lines[len(lines) - 1].startswith('INVALID'):
            # Calculate duration of trial
            start_time = float(lines[0].split('\t')[0])
            end_time = float(lines[len(lines) - 1].split('\t')[0])
            duration = end_time - start_time

            # Calculate number of collisions
            last_coll = 'f'
            coll_count = 0
            for line in lines:
                current_coll = line.split('\t')[5].strip()
                if last_coll == 'f' and current_coll == 't':
                    coll_count += 1
                last_coll = current_coll

            # Assign outcome to proper experimental bin
            if name.find('cam') < 0:
                tag_duration_total += duration
                tag_durations.append(duration)
                tag_collision_total += coll_count
                tag_colls.append(coll_count)
            else:
                cam_duration_total += duration
                cam_durations.append(duration)
                cam_collision_total += coll_count
                cam_colls.append(coll_count)

#            if name.find('cam') < 0:
#                tag_num += duration
#                tag_coll += coll_count
#                tag_total += 1
#                tag_out.write(str(duration) + "\n")
#            else:
#                cam_num += duration
#                cam_coll += coll_count
#                cam_total += 1
#                cam_out.write(str(duration) + "\n")
            
#        f.close()

#cam_out.close()
#tag_out.close()

# Now that we have access to the mean, calculate the variance of the
# list of durations we just saved.

cam_mean = cam_duration_total / len(cam_durations)
tag_mean = tag_duration_total / len(tag_durations)
cam_coll_mean = cam_collision_total / len(cam_colls)
tag_coll_mean = tag_collision_total / len(tag_colls)


def calculate_conf(arr, mean):
    variance_total = 0.0
#    f = open(filename,'r')
#    lines = f.readlines()
    for elt in arr:
#        d = float(line)
        variance_total += (elt - mean)**2
#    f.close()
    std_dev = (variance_total / len(arr))**0.5
    conf = 1.96*(std_dev/(len(arr)**0.5))
    return conf

tag_in = open('analyzed_data/tag_total_time.dat','r')



print("Cam (" + str(len(cam_durations)) + " instances):")
print("\t Time: " + str(cam_mean))
print("\t Time conf: +/-" + str(calculate_conf(cam_durations, cam_mean)))
print("\t Collisions: " + str(cam_coll_mean))
print("\t Collisions conf: +/-" + str(calculate_conf(cam_colls, cam_coll_mean)))
print("Tag:(" + str(len(tag_durations)) + " instances):")
print("\t Time: " + str(tag_mean))
print("\t Time conf: +/-" + str(calculate_conf(tag_durations, tag_mean)))
print("\t Collisions: " + str(tag_coll_mean))
print("\t Collisions conf: +/-" + str(calculate_conf(tag_colls, tag_coll_mean)))

