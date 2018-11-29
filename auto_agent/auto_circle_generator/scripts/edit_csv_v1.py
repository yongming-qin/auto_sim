## For packetgen. Now I have one using rostopic echo /ravenstate
# This v1 works fine by adding the field.current_cmd columns
# v2 is not completed.
from __future__ import print_function
import csv
import numpy as np



## --------------
inFile = open("/home/yq/Simulator/mine_ws/automove_ravenstate3.csv")
reader = csv.reader(x.replace('\0', '') for x in inFile)
headers = reader.next()

outFile = open('/home/yq/Simulator/mine_ws/automove_ravenstate3_sequence.csv','w')
writer = csv.writer(outFile,delimiter=',')
print(headers+['field.current_cmd0'])
for i in range(16):
    cName = "field.current_cmd" + str(i)
    print(cName)
    headers = headers + [cName]
writer.writerow(headers) # column names

# print(headers)
# Find the indices for the variables in the datasheet
runlevel_index = headers.index('field.runlevel')
packet_index = headers.index('field.last_seq')
dori_index = headers.index('field.ori_d0')
dpos_index = headers.index('field.pos_d0')
pos_index = headers.index('field.pos0')
jpos_index = headers.index('field.jpos0')
jvel_index = headers.index('field.jvel0')
dmpos_index = headers.index('field.mpos_d0')
mpos_index = headers.index('field.mpos0')
mvel_index = headers.index('field.mvel0')
grasp_index = headers.index('field.grasp_d0')
enc_index = headers.index('field.encVals0')
# dac_index = headers.index('field.current_cmd0') # not in this one
djpos_index = headers.index('field.jpos_d0')

last_seq = 0
while True:
    try:
        line = reader.next()
    except csv.Error:
        print("Error")
    except StopIteration:
        print("Iteration End")
        break
    last_seq = last_seq + 1
    line[packet_index] = str(last_seq)
    for i in range(16):
        line = line + [str(0)]
    writer.writerow(line)

inFile.close()
outFile.close()

print("csv file is created")









































if False:
    print("jpos: ")
    for i in range(16):
        print(line[jpos_index+i]+",", end='')
    print("\n")

    print("djpos: ")
    for i in range(16):
        print(line[djpos_index+i]+",", end='')
    print("\n")

    print("pos: ")
    for i in range(6):
        print(line[pos_index+i]+",", end='')
    print("\n")

    print("dpos: ")
    for i in range(6):
        print(line[dpos_index+i]+",", end='')
    print("\n")

    print("mpos: ")
    for i in range(16):
        print(line[mpos_index+i]+",", end='')
    print("\n")

    print("dmpos: ")
    for i in range(16):
        print(line[dmpos_index+i]+",", end='')
    print("\n")

    print("dori: ")
    for i in range(18):
        print(line[dori_index+i]+",", end='')
    print("\n")
