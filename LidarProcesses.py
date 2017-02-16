#!/usr/bin/env python

import socket
from threading import Thread
from Queue import Queue
from numpy import interp, zeros, savetxt, array, shape,empty
from numpy import append, concatenate
from itertools import chain
import time

from Lidarfunc import *
N_DATA_BLOCKS = 12


class LIDAR:
    def __init__(self,ip,port):
        self.sock_data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_data.bind((ip, port))
        self.sock_data.settimeout(10) # 10 second timeout, change to input later

        self.dataq = Queue(maxsize=0)
        self.fstore = []
        self.init_frame()

    def init_frame(self):
        self.frame = empty(shape=(0,7), dtype=float)
        
        
    def pull_data(self):
        try:
            data_packet, addr = self.sock_data.recvfrom(1248)
            hex_data_packet = map(hex,map(ord,data_packet))
            data_partial = self.data_packet_parse(hex_data_packet)
            self.dataq.put_nowait(data_partial)
        except socket.timeout:
            print("Closing data socket")
            self.sock_data.close()
            
                
    def data_packet_parse(self,hex_data_packet):
        
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
        return data_packet_sensor_info


    def create_frame(self, MAX_LIM = 50000):        
        F = self.dataq.get_nowait()
        self.dataq.task_done()  
        self.frame = concatenate((self.frame, F), axis=0)
        self.frame = get_uniques(self.frame)

        # need to interpolate between the data along a constant z
        if shape(self.frame)[0] > MAX_LIM:            
            self.fstore.append(self.frame)
            self.init_frame()
        
                                
        

def create_packet_table(datablock):
    #data_packet_sensor_info = []
    #data_packet_sensor_info = empty(shape=(0,4), dtype=float) (x,y,z,reflec)
    data_packet_sensor_info = empty(shape=(0,7), dtype=float) #(x,y,z,r,g,b,a)
    
    azimuth_angles = [0]*12
    count = 0
    for BLOCK in xrange(1,N_DATA_BLOCKS+1):
        azimuth_bytes = datablock[BLOCK]['azimuthbytes']
        #print azimuth_bytes
        azimuth_angle = read_azimuth_bytes(azimuth_bytes)
        azimuth_angles[count] = azimuth_angle
        count += 1

    
    # interpolate the azimuth angle values
    azimuth_angles_inter = list_mean_circinterp(azimuth_angles)
    #azimuth_angles_inter = interp(azimuth_angles)
    
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

            # convert data to cartesian coordinates
            xyz = convert2cart(distance, azimuth, elevation)
            #data_packet_sensor_info.append(((distance,azimuth,elevation),reflec))
            #data_packet_sensor_info.append((xyz[0],xyz[1],xyz[2],reflec))

            #convert to reflectivity to rgba
            reflec_rgba = rgba_creator(reflec)
            
            data_packet_sensor_info = append(data_packet_sensor_info, array([[ xyz[0],xyz[1],xyz[2],reflec_rgba[0],reflec_rgba[1],reflec_rgba[2],reflec_rgba[3] ]]), axis=0)
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


def rgba_creator(reflectivity):
    if reflectivity >= 0 and reflectivity < 50:
        return (0,(1.0/50.0)*reflectivity,1,1)
    elif reflectivity >= 50 and reflectivity < 101:
        return (0,1,-(1.0/50.0)*reflectivity+2,1)
    elif reflectivity >= 101 and reflectivity < 180:
        return (0,(1.0/80.0)*reflectivity-(5.0/4.0),0,1)
    elif reflectivity >= 180 and reflectivity < 256:
        return (0,(255.0-reflectivity)/75.0,0,1)


    
