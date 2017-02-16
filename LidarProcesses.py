#!/usr/bin/env python

import socket
from threading import Thread
from Queue import Queue
from numpy import interp, zeros, savetxt, array, shape
from itertools import chain
import time

from Lidarfunc import *
N_DATA_BLOCKS = 12
#global Data
#Data=[[],[],[],[]] #(x,y,z,intensity)

def pull_data(ip, port, q_data):
    sock_data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_data.bind((ip, port))
    sock_data.settimeout(10) # 10 second timeout, change to input later

    while True:
        try:
            t = time.time()
            data_packet, addr = sock_data.recvfrom(1248)
            hex_data_packet = map(hex,map(ord,data_packet))
            data_packet_parse(hex_data_packet,q_data)
            elapsed = time.time() -t
            #print 'sock create ',elapsed
        except socket.timeout:
            print("Closing data socket")
            sock_data.close()
            break
            

def pull_pos(ip, port, q_pos):
    sock_pos = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_pos.bind((ip, port))

    sock_data.settimeout(2) # 2 second timeout, change to input later

    while True:
        try:
            pos_packet, addr = sock_pos.recvfrom(554)
            hex_pos_packet = map(hex,map(ord, pos_packet))
            q_pos.put_nowait(hex_data_packet)
        except socket.timeout:
            print("Closing GPS socket")
            sock_pos.close()


def data_packet_parse(hex_data_packet,q_data):
    t = time.time()
    hex_time_packet = hex_data_packet[1200:1204]
    
    time_sec = read_timestamp_bytes(hex_time_packet)
    
    #get the factory data bytes
    hex_factory_packet = hex_data_packet[1204:]

    #get hex_data_block
    hex_data_blocks = hex_data_packet[:1200]
    Datablock = {}
    
    for i in xrange(1,N_DATA_BLOCKS+1):
        Datablock[i] = {}
        
        #get the azimuth bytes from each data block
        azimuth_bytes = hex_data_blocks[100*(i-1):100*i][2:4]
        
        # split the entire data block into chunks representing channels
        channel_bytes = list(chunks(hex_data_blocks[100*(i-1):100*i][4:],3)) 

        Datablock[i]['azimuthbytes'] = azimuth_bytes
        Datablock[i]['channelbytes'] = channel_bytes
        
    data_packet_sensor_info = create_packet_table(Datablock)
    q_data.put(data_packet_sensor_info)
    elapsed = time.time() -t
    print 'packet parse ',elapsed


def create_packet_table(datablock):
    data_packet_sensor_info = []
    
    azimuth_angles = [0]*12
    count = 0
    for BLOCK in xrange(1,N_DATA_BLOCKS+1):
        azimuth_bytes = datablock[BLOCK]['azimuthbytes']
        #print azimuth_bytes
        azimuth_angle = read_azimuth_bytes(azimuth_bytes)
        azimuth_angles[count] = azimuth_angle
        count += 1

    #print azimuth_angles
    # interpolate the azimuth angle values
    azimuth_angles_inter = list_mean_circinterp(azimuth_angles)
    #azimuth_angles_inter = interp(azimuth_angles)
    #print azimuth_angles_inter
    
    for BLOCK in xrange(1,N_DATA_BLOCKS+1):
        for CHANNEL_NO in xrange(len(datablock[BLOCK]['channelbytes'])):
            if CHANNEL_NO >=0 and CHANNEL_NO < 16:
                laser_id = CHANNEL_NO
                azimuth = azimuth_angles_inter[BLOCK]
            elif CHANNEL_NO >=16 and CHANNEL_NO < 32:
                laser_id = CHANNEL_NO - 16
                azimuth = azimuth_angles_inter[BLOCK+1]

            range_bytes = datablock[BLOCK]['channelbytes'][CHANNEL_NO][0:2]
            reflec_bytes = datablock[BLOCK]['channelbytes'][CHANNEL_NO][2]
                       
            distance = read_range_bytes(range_bytes)
            reflec = read_reflectivity_bytes(reflec_bytes)
            elevation = get_elevation_angle(laser_id)
            data_packet_sensor_info.append(((distance,azimuth,elevation),reflec))

    return data_packet_sensor_info


def get_elevation_angle(laser_id):
    # Fixed to velodyne VLP16 puck lite for now
    MODEL = 'VLP16PUCK_LITE'
    elevat_dict = {}
    if MODEL == 'VLP16PUCK_LITE':
        elevat_dict[0] = -15.0
        elevat_dict[1] = 1
        elevat_dict[2] = -13.0
        elevat_dict[3] = 3.0
        elevat_dict[4] = -11.0
        elevat_dict[5] = 5.0
        elevat_dict[6] = -9.0
        elevat_dict[7] = 7.0
        elevat_dict[8] = -7.0
        elevat_dict[9] = 9.0
        elevat_dict[10] = -5.0
        elevat_dict[11] = 11.0
        elevat_dict[12] = -3.0
        elevat_dict[13] = 13.0
        elevat_dict[14] = -1.0
        elevat_dict[15] = 15.0
    else:
        print("unknown model type")
        elevat_dict[laser_id] = 0.0
        
    return elevat_dict[laser_id]

def create_frame(q_data, MAX_LIM = 1000):
    Data =[[],[],[],[]] #(x,y,z,intensity)
    #global Data
    if not q_data.empty():
        if q_data.qsize() > MAX_LIM:            
            t = time.time()
            data_packets = queue_get_all(q_data, MAX_LIM)
            q_data.task_done()

            elapsed = time.time() -t
            print 'frame create ',elapsed
            
            for each_packet in data_packets:
                for data in each_packet:
                    my_ret = convert2cart(data[0][0], data[0][1], data[0][2])                    
                    Data[0].append(my_ret[0])
                    Data[1].append(my_ret[1])
                    Data[2].append(my_ret[2])
                    Data[3].append(data[1])
                  
            Data_me = array(Data)
            Data_me = Data_me.T
            Data_me = get_uniques(Data_me)
            return Data_me
        else:
            Data_me = array(Data)
            return Data_me
    else:
        Data_me = array(Data)
        return Data_me
        
    
