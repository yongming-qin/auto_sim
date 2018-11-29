## For packetgen. Now I have one using rostopic echo /ravenstate
from __future__ import print_function
import csv
import numpy as np

## --------------
csvFile = open("/home/yq/new_test_15.csv")
csvReader = csv.reader(x.replace('\0', '') for x in csvFile)
headers = csvReader.next()

inFile = open("/home/yq/Simulator/mine_ws/ravenstate3.csv")
reader = csv.reader(x.replace('\0', '') for x in inFile)
line = reader.next()

outFile = open('/home/yq/Simulator/mine_ws/automove_ravenstate3.csv','w')
writer = csv.writer(outFile,delimiter=',')
writer.writerow(headers) # column names

# print(headers)
# Find the indices for the variables in the datasheet
runlevel_idx = headers.index('field.runlevel')
packet_idx = headers.index('field.last_seq')
dori_idx = headers.index('field.ori_d0')
dpos_idx = headers.index('field.pos_d0')
pos_idx = headers.index('field.pos0')
jpos_idx = headers.index('field.jpos0')
jvel_idx = headers.index('field.jvel0')
dmpos_idx = headers.index('field.mpos_d0')
mpos_idx = headers.index('field.mpos0')
mvel_idx = headers.index('field.mvel0')
grasp_idx = headers.index('field.grasp_d0')
enc_idx = headers.index('field.encVals0')
dac_idx = headers.index('field.current_cmd0')
djpos_idx = headers.index('field.jpos_d0')

fl = csvReader.next()


runlevel_index = line.index('field.runlevel')
packet_index = line.index('field.last_seq')
dori_index = line.index('field.ori_d0')
dpos_index = line.index('field.pos_d0')
pos_index = line.index('field.pos0')
jpos_index = line.index('field.jpos0')
jvel_index = line.index('field.jvel0')
dmpos_index = line.index('field.mpos_d0')
mpos_index = line.index('field.mpos0')
mvel_index = line.index('field.mvel0')
grasp_index = line.index('field.grasp_d0')
enc_index = line.index('field.encVals0')
#dac_index = line.index('field.current_cmd0')  # not in this file
djpos_index = line.index('field.jpos_d0')


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
    fl[packet_index] = str(last_seq)
    runlevel_idx = 
    packet_idx = 
    dori_idx = 
    dpos_idx = 
    pos_idx =
    jpos_idx =
    jvel_idx = 
    dmpos_idx 
    mpos_idx = headers.index('field.mpos0')
    mvel_idx = headers.index('field.mvel0')
    grasp_idx = headers.index('field.grasp_d0')
    enc_idx = headers.index('field.encVals0')
    dac_idx = headers.index('field.current_cmd0')
    djpos_idx = headers.index('field.jpos_d0')
    writer.writerow(fl)

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
