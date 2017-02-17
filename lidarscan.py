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
    global lidarscatter, lid
    IP = '192.168.1.71' # IP of recving computer
    PORT_DATA = 2368
    PORT_POS = 8308

    #q_data = Queue(maxsize=0)
    #t_data = Thread(target = pull_data, args=(IP,PORT_DATA, q_data,))
    #t_parse = Thread(target = data_packet_parse, args=(q_data, q_frames,))
    #t_write = Thread(target = write_to_file, args=(FILE_NAME, q_frames,))

    #t_data.daemon = True
    #t_parse.daemon = True
    #t_write.daemon = True
    
    #t_data.start()
    #t_parse.start()
    #t_write.start()


    lid = LIDAR(IP,PORT_DATA)
    
    app=QtGui.QApplication([])
    w = gl.GLViewWidget()
    w.opts['distance'] = 20
    layout = QtGui.QGridLayout()
    w.setLayout(layout)
    
    
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
    global lidarscatter, lid
    #FILE_NAME = 'LIDAR_DATA.txt'
    #f_handle = file(FILE_NAME, 'a')
    
    lid.pull_data()
    lid.create_frame(10000)

    if len(lid.fstore) > 0:
        DATA = lid.fstore.pop(0)
        #savetxt(f_handle, DATA)
        pos1 = DATA[:,0:3]
        color1 = DATA[:,3:]
        lidarscatter.setData(pos=pos1, color=color1, size=2.5, pxMode=True)
        #f_handle.close()
    

if __name__ == '__main__':
    main()

