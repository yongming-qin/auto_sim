## for build moveit path
# the original path is produeced by auto_circle_generater/dsn_path.cpp

from __future__ import print_function
import csv
import numpy as np



## --------------
csvfile = open("new_test_15.csv")
reader = csv.reader(x.replace('\0', '') for x in csvfile)
headers = reader.next()

outfile = open('/home/yq/Simulator/raven_homa/raven_2/teleop_data/new_test_24.csv','w')
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
firstLineRunlevel3 = line # The first line that runlevel is 3


#test_ori_d0 = [0.916606, 0.342309, 0.206538, 0.282119, -0.187773, -0.940824, -0.283270, 0.920633, -0.268686]
#test_ori_d1 = [0.885214, -0.351202, 0.305045, -0.379502, -0.165987, 0.910179, -0.269024, -0.921469, -0.280216]

test_ori_d0 = [-0.939526, -0.209878, 0.270631, -0.303696, 0.145309, -0.941623, 0.158301, -0.966869, -0.200261]
test_ori_d1 = [-0.927714, 0.215162, 0.305045, 0.350481, 0.220764, 0.910179, 0.128493, 0.951298, -0.280216]

## --------------
inFile = open("/home/yq/Simulator/moveit_dsn/raven_devel/moveit_traj_pose.csv")
inReader = csv.reader(x.replace('\0', '') for x in inFile)
inHeaders = inReader.next()
inPosIdx = inHeaders.index('position0')
inOriIdx = inHeaders.index('orientation0')
inLine = inReader.next()

rad2deg = 180 / 3.1415926

## first few lines
newLine = ["0"] * len(firstLineRunlevel3)
last_seq = 0
for i in range(1,10): # [1] initial values
    newLine[runlevel_index] = "3"
    last_seq = last_seq + 1
    newLine[packet_index] = str(last_seq)
    
    newLine[dpos_index:dpos_index+3] = ( np.array([-77875, -25278, 13617])
                                         + np.array( map(float, inLine[inPosIdx:inPosIdx+3]) )*1000*1000 ).tolist() # filed.pos_d
    print(inLine[inPosIdx:inPosIdx+3])
    newLine[dpos_index+3:dpos_index+6] = firstLineRunlevel3[dpos_index+3:dpos_index+6] # filed.pos_d
    
    #newLine[dori_index:dori_index+9] = inLine[inOriIdx:inOriIdx+9] # filed.ori_d
    newLine[dori_index:dori_index+9] = map(str, test_ori_d0)
    newLine[dori_index+9:dori_index+18] = map(str, test_ori_d1) # filed.ori_d

    inLine = inReader.next()

    # filed.pos

    # [144.817537, 89.181395, 0.682912, 0, 127.296729, 2.729005]
    newLine[jpos_index:jpos_index+6] = [0.523*rad2deg, 1.577*rad2deg, 0.4*rad2deg, 0, 0, 0] # filed.jpos # 3rd!!
    newLine[jpos_index+6:jpos_index+8] = firstLineRunlevel3[jpos_index+6:jpos_index+8] # for graspers
    newLine[jpos_index+8:jpos_index+16] = firstLineRunlevel3[jpos_index+8:jpos_index+16] # filed.jpos
    # filed.jvel

    # field.mpos_d
    # field.mpos
    # filed.mvel

    newLine[grasp_index:grasp_index+2] = [1, 1] # filed.grasp_d
    # filed.envVals
    # filed.current_cmd
    # filed.jpos_d

    writer.writerow(newLine)

print("the first line is created")

newLine = ["0"] * len(firstLineRunlevel3)
while True:
    newLine[runlevel_index] = "3"
    last_seq = last_seq + 1
    newLine[packet_index] = str(last_seq)

    newLine[dpos_index:dpos_index+3] = ( np.array([-77875, -25278, 13617])
                                         + np.array( map(float, inLine[inPosIdx:inPosIdx+3]) )*1000*1000 ).tolist() # filed.pos_d
    newLine[dpos_index+3:dpos_index+6] = firstLineRunlevel3[dpos_index+3:dpos_index+6] # filed.pos_d
    
    #newLine[dori_index:dori_index+9] = inLine[inOriIdx:inOriIdx+9] # filed.ori_d
    newLine[dori_index:dori_index+9] = map(str, test_ori_d0)
    newLine[dori_index+9:dori_index+18] = map(str, test_ori_d1) # filed.ori_d


    newLine[grasp_index:grasp_index+2] = [1, 1] # firstLineRunlevel3[grasp_index:grasp_index+2] # filed.grasp_d

    writer.writerow(newLine)
    try:
        inLine = inReader.next()
    except StopIteration:
        print("end of the inFile")
        break

inFile.close()
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
