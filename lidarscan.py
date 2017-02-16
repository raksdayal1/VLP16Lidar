#!/usr/bin/env python

import sys
import time
import socket
from threading import Thread
from Queue import Queue

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg

from numpy import size,shape,zeros
from LidarProcesses import *

#defines
N_DATA_BLOCKS = 12

def main():
    global lidarscatter, q_frames
    IP = '192.168.1.71' # IP of recving computer
    PORT_DATA = 2368
    PORT_POS = 8308

    FILE_NAME = 'LIDAR_DATA.txt'

    q_data = Queue(maxsize=500)
    #q_pos = Queue(maxsize=500)
    q_frames = Queue(maxsize=2000)
    q_plot = Queue(maxsize=20000)
    
    t_data = Thread(target = pull_data, args=(IP,PORT_DATA, q_data,))
    #t_pos = Thread(target = pull_position, args=(IP, PORT_POS, q_pos,))
    t_parse = Thread(target = data_packet_parse, args=(q_data, q_frames,))
    #t_write = Thread(target = write_to_file, args=(FILE_NAME, q_frames,))

    t_data.daemon = True
    #t_pos.daemon = True
    t_parse.daemon = True
    #t_write.daemon = True
    
    t_data.start()
    #t_pos.start()
    t_parse.start()
    #t_write.start()

    app=QtGui.QApplication([])
    w = gl.GLViewWidget()
    w.opts['distance'] = 20
    w.show()
    w.setWindowTitle('GE-OSU Matrice 600 VLP16 Custom Lidar Scan')

    g = gl.GLGridItem()
    g.setSpacing(0.5,0.5,0.5)
    w.addItem(g)

    axes = gl.GLAxisItem(glOptions='opaque')
    w.addItem(axes)

    pos1 = zeros(shape=(10000,3))
    lidarscatter = gl.GLScatterPlotItem(pos=pos1, color=(0.2,0.4,0.5,0.4),size=2,pxMode=True)
    
    w.addItem(lidarscatter)

    t = QtCore.QTimer()
    t.timeout.connect(update)
    t.start(0)

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


def update():
    global lidarscatter, q_frames
    #t = time.time()
    A = create_frame(q_frames, MAX_LIM = 1000)
    if not size(A) == 0:
        
        pos1 = zeros(shape=(shape(A)[0],3))
        pos1[:,0] = A[:,0]
        pos1[:,1] = A[:,1]
        pos1[:,2] = A[:,2]
        lidarscatter.setData(pos=pos1, color=(0.2,0.4,0.6,0.8), size=2, pxMode=True)
        #elapsed = time.time() -t
        #print elapsed
        
        

if __name__ == '__main__':
    main()

