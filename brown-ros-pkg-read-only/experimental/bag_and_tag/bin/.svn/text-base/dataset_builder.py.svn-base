#!/usr/bin/env python
import os

#Run data_massager.py prior to running this code.  Make sure INVALID
#DATA is removed prior to doing so.

cam_out = open('analyzed_data/cam_data.tab','w')
tag_out = open('analyzed_data/tag_data.tab','w')

cam_out.write('last_tag_seen\ttag_visible\ttag_x_coord\ttag_distance\tbumping\tdrive_command\n')
cam_out.write('d\td\tc\tc\td\td\n')
cam_out.write('\t\t\t\t\tclass\n')
tag_out.write('last_tag_seen\ttag_visible\ttag_x_coord\ttag_distance\tbumping\tdrive_command\n')
tag_out.write('d\td\tc\tc\td\td\n')
tag_out.write('\t\t\t\t\tclass\n')

for root, dirs, files in os.walk('data'):
    for name in files:
        f = open(os.path.join(root, name), 'r')
        for line in f.readlines():
            # Strip off the timestamp
            current_line = line.split('\t')
            towrite = current_line[1] + "\t" + \
                current_line[2] + "\t" + \
                current_line[3] + "\t" + \
                current_line[4] + "\t" + \
                current_line[5] + "\t" + \
                current_line[6]
            if name.find('cam') >= 0:
                cam_out.write(towrite)
            else:
                tag_out.write(towrite)

        f.close()
cam_out.close()
tag_out.close()
