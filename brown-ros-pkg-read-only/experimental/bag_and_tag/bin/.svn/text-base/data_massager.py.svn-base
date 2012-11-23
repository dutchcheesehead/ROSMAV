#!/usr/bin/env python
import os

#INVALID DATA should be removed from the data directory prior to
#running this script

for root, dirs, files in os.walk('data'):
    for name in files:
        f = open(os.path.join(root, name), 'r')
        lines = f.readlines()
        f.close()
        f = open(os.path.join(root, name), 'w')
        for line in lines:
            current_line = line.split('\t')
            current_line[6] = current_line[6].strip()

            towrite = current_line[0] + "\t" + current_line[1] + "\t"

            if current_line[1] == '-':
                towrite += "f\t"
            else:
                towrite += "t\t"

            towrite += current_line[2] + "\t" + current_line[3] + "\t" \
                + current_line[6] + "\t" + current_line[4] + "\t" \
                + current_line[5] + "\n"

            f.write(towrite)

        f.close()
        f = open(os.path.join(root, name), 'r')
        lines = f.readlines()
        f.close()
        for i in range(1, len(lines)):
            previous_line = lines[i-1].split('\t')
            current_line = lines[i].split('\t')
            for j in range(8):
                if current_line[j].strip() == '-':
                    current_line[j] = previous_line[j]
            lines[i] = current_line[0] + "\t" + \
                current_line[1] + "\t" + \
                current_line[2] + "\t" + \
                current_line[3] + "\t" + \
                current_line[4] + "\t" + \
                current_line[5] + "\t" + \
                current_line[6] + "\t" + \
                current_line[7] + "\n"

        for i in range(0, len(lines)):
            current_line = lines[i].split('\t')
            lines[i] = current_line[0] + "\t" + \
                current_line[1] + "\t" + \
                current_line[2] + "\t" + \
                current_line[3] + "\t" + \
                current_line[4] + "\t" + \
                current_line[5] + "\t"
            if current_line[6] == '-0.5':
                lines[i] += 'b\n'
            elif current_line[6] == '0.0' or current_line[6] == '-':
                if current_line[7].strip() == '1.0':
                    lines[i] += 'l\n'
                elif current_line[7].strip() == '-1.0':
                    lines[i] += 'r\n'
                else:
                    lines[i] += 's\n'
            else:
                if current_line[7].strip() == '1.0':
                    lines[i] += 'fl\n'
                elif current_line[7].strip() == '-1.0':
                    lines[i] += 'fr\n'
                else:
                    lines[i] += 'f\n'

        #Take out the stops
        for i in range(len(lines)-2, -1, -1):
            previous_line = lines[i+1].split('\t')
            current_line = lines[i].split('\t')
            if current_line[6].strip() == 's':
#                print "replacing s with " + previous_line[6]
                lines[i] = current_line[0] + "\t" + \
                    current_line[1] + "\t" + \
                    current_line[2] + "\t" + \
                    current_line[3] + "\t" + \
                    current_line[4] + "\t" + \
                    current_line[5] + "\t" + \
                    previous_line[6]

        f = open(os.path.join(root, name), 'w')
        for line in lines:
            f.write(line)
        f.close()
