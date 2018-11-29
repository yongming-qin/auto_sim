## man made trajectory
# pos_d are used by r2_control for desired position
# jpos are used to initiate the initial pose
# ori_d is set to be same as the initial pose calculated from jpos

from __future__ import print_function
import csv
import numpy as np




csvfile = open("new_test_15.csv")
reader = csv.reader(x.replace('\0', '') for x in csvfile)
headers = reader.next()

outfile = open('/home/yq/Simulator/raven_homa/raven_2/teleop_data/new_test_25.csv','w')
writer = csv.writer(outfile,delimiter=',')
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
dac_index = headers.index('field.current_cmd0')
djpos_index = headers.index('field.jpos_d0')

line = reader.next()
runlevel = int(line[runlevel_index])
n = 0
while (runlevel != 3):
    n = n + 1
    line = reader.next()
    runlevel = int(line[runlevel_index])
print("The first line which runlevel_index = 3 is: " + str(n))

## ------------------------------------------------------------ ##

firstLineRunlevel3 = line # The first line that runlevel is 3
firstLine = line
posStart = map(int, line[pos_index:pos_index+6])


for i in range(0,8):
    print(line[jpos_index+i]+",", end='')




posContainer = [-126241, -14389, -23611]


test_ori_d0 = [0.916606, 0.342309, 0.206538, 0.282119, -0.187773, -0.940824, -0.283270, 0.920633, -0.268686]
test_ori_d1 = [0.885214, -0.351202, 0.305045, -0.379502, -0.165987, 0.910179, -0.269024, -0.921469, -0.280216]
test_ori = test_ori_d0 + test_ori_d1

## first few lines
last_seq = 0
for i in range(1,10):
    firstLine[runlevel_index] = "3"
    last_seq = last_seq + 1
    firstLine[packet_index] = str(last_seq)
    # filed.pos_d
    # filed.ori_d
    firstLine[dori_index:dori_index+18] = map(str, test_ori)

    # filed.pos
    

    # filed.jpos
    # filed.jvel

    # field.mpos_d
    firstLine[dmpos_index:dmpos_index+5] = [0, 0, 0, 0, 0]
    # field.mpos
    # filed.mvel

    firstLine[grasp_index:grasp_index+2] = [1, 1] # filed.grasp_d
    # filed.envVals
    # filed.current_cmd
    # filed.jpos_d

    writer.writerow(firstLine)

STEPS = 10000

## changing eef positon
posEnd = [-179004, -2788, -14335, -77577, 25926, 13513]
ar_posStart = np.array(posStart)
print(ar_posStart)
ar_posTmp = ar_posStart
ar_posEnd = np.array(posEnd)
ar_posInc = (ar_posEnd - ar_posStart) / STEPS
print(ar_posInc)

newLine = ["0"] * len(firstLine)
for i in range(1,STEPS):
    newLine[runlevel_index] = "3"
    last_seq = last_seq + 1
    newLine[packet_index] = str(last_seq)

    newLine[dpos_index:dpos_index+6] = ar_posTmp.tolist() # filed.pos_d
    ar_posTmp = ar_posTmp + ar_posInc
    #print(ar_posTmp)

    newLine[dori_index:dori_index+18] = map(str, test_ori) # filed.ori_d

    newLine[grasp_index:grasp_index+2] = [1, 1] # firstLineRunlevel3[grasp_index:grasp_index+2] # filed.grasp_d

    

    writer.writerow(newLine)

outfile.close()
csvfile.close()

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
