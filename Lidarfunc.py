#!/usr/bin/env python

import math

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in xrange(0, len(l), n):
        yield l[i:i + n]

def append_hexbytes(a,b):
	head_byte = a[0:2]
	a = a[2:]
	b = b[2:]
	c = head_byte + a + b
	return c

def read_azimuth_bytes(data_bytes):
    if len(data_bytes) == 2:
        data_bytes.reverse() # reverse the list containing the two hex bytes
        data_hex = append_hexbytes(data_bytes[0],data_bytes[1]) #get the complete hex
        data_dec = int(data_hex, 16) #convert to decimal
        azimuth_angle = data_dec/100.0 # divide by 100 to get angle value
    else:
        print("Azimuth bytes should be of length 2")
        sys.exit(-1)
    return azimuth_angle

def read_range_bytes(data_bytes):
    if len(data_bytes) == 2:
        data_bytes.reverse() # reverse the list containing the two hex bytes
        data_hex = append_hexbytes(data_bytes[0],data_bytes[1]) #get the complete hex
        data_dec = int(data_hex, 16) #convert to decimal
        distance_mm = data_dec*2.0 # multiply the decimal value with 2 to obtain range in mm
        distance_m = distance_mm/1000.0 # divide by 1000 to get range in meters
    else:
        print("Range bytes should be of length 2")
        sys.exit(-1)
    return distance_m

def read_reflectivity_bytes(data_bytes):
    #if len(data_bytes) == 1:
    reflectivity = int(data_bytes,16)
    #else:
    #   print("Reflectivity bytes should be of length 1")
    #    sys.exit(-1)
    return reflectivity

def read_timestamp_bytes(data_bytes):
    if len(data_bytes) == 4:
        data_bytes.reverse()
        data_hex = data_bytes[0][0:2]+data_bytes[0][2:]+data_bytes[1][2:]+data_bytes[2][2:]+data_bytes[3][2:] # convert to hex
        data_dec = int(data_hex, 16) #convert to decimal, time obtained in microsec
        time_sec = data_dec/1e6
        time_min = time_sec/60.0;
    else:
        print("Time stamp bytes should be of length 4")
        sys.exit(-1)
    return time_sec,time_min

def queue_get_all(q, maxItemsToRetreive):
    items = []    
    for numOfItemsRetrieved in range(0, maxItemsToRetreive):
        try:
            if numOfItemsRetrieved == maxItemsToRetreive:
                break
            items.append(q.get_nowait())
        except Empty, e:
            break
    return items

def convert2cart(r,a,w):
    a = a*(math.pi/180.0)
    w = w*(math.pi/180.0)
    
    x = r*math.cos(w)*math.sin(a)
    y = r*math.cos(w)*math.cos(a)
    z = r*math.sin(w)
    return x,y,z

def list_mean_circinterp(input_list):
    # add first element to the end of the list
    input_list.append(input_list[0]+360)
    
    desired_list = [0.0]*len(input_list)*2
    count = 0
    for i in input_list:
        desired_list[count] = i
        count += 2

    count_no = 1
    for v, w in zip(input_list[:-1],input_list[1:]):
        desired_list[count_no] = float(sum([v,w])) / max(len([v,w]), 1)
        count_no += 2
        
    for ang_index in xrange(len(desired_list)):
        if desired_list[ang_index] > 360:
            desired_list[ang_index] = desired_list[ang_index] - 360
    desired_list.pop(-1)
    desired_list.pop(-1)
        
    return desired_list
        
        
