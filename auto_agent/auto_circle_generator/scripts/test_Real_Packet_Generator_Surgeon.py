'''/* Surgeon Packet Generator - Works with Control software for the Raven II robot
  Input arguments simulator
 * Copyright (C) 2015 University of Illinois Board of Trustees, DEPEND Research Group, Creator: Homa Alemzadeh
 *
 * This file is part of Raven 2 Surgical Simulator.
 *
 * Raven 2 Surgical Simulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Surgical Simulator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */'''

import socket
import struct
import csv
import time
from collections import namedtuple
import threading
import sys
import signal
import time
import math
from sys import argv
import os


def sendPackets():
    csvfile = open('/home/yq/Simulator/mine_ws/automove_ravenstate_1_sequence.csv','r');    
    #outfile = open('./robot_run.csv','w');
    #reader = csv.reader(csvfile)
    reader = csv.reader(x.replace('\0', '') for x in csvfile)
    #writer = csv.writer(outfile,delimiter=',')
    headers = reader.next()
    #writer.writerow(headers)
    # Find the indices for the variables in the datasheet
    runlevel_index = headers.index('field.runlevel');
    packet_index = headers.index('field.last_seq'); 
    dori_index = headers.index('field.ori_d0');
    dpos_index = headers.index('field.pos_d0');
    pos_index = headers.index('field.pos0');
    jpos_index = headers.index('field.jpos0');
    jvel_index = headers.index('field.jvel0');
    dmpos_index = headers.index('field.mpos_d0');
    mpos_index = headers.index('field.mpos0');
    mvel_index = headers.index('field.mvel0');
    grasp_index = headers.index('field.grasp_d0');
    enc_index = headers.index('field.encVals0');
    dac_index = headers.index('field.current_cmd0');
    djpos_index = headers.index('field.jpos_d0');

    # Skip the packets until runlevel 3
    runlevel = 0;	
    packet_num = 111;
    line_no = 0;
    line = [];
    while (runlevel < 3) or (packet_num == 111) or (packet_num == 0):
        try:
            line = reader.next()
        except Exception: 
            print line
        line_no = line_no+1
        runlevel = int(line[runlevel_index])
        packet_num = int(line[packet_index])
    print('Started at Line = '+ str(line_no)+ ' and Packet = '+str(packet_num))
  

    csvfile.close()
    #outfile.close()

sendPackets()



