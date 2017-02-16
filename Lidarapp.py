#!/usr/bin/env python

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np

app=QtGui.QApplication([])
w = gl.GLViewWidget()
w.opts['distance'] = 20
w.show()
w.setWindowTitle('pyqtgraph example: GLScatterPlotItem')

g = gl.GLGridItem()
g.setSpacing(0.5,0.5,0.5)
w.addItem(g)

axes = gl.GLAxisItem(glOptions='opaque')
w.addItem(axes)

pos = np.zeros(shape=(100000,3))
lidarscatter = gl.GLScatterPlotItem(pos=pos, color=(1,1,1,1))
w.addItem(lidarscatter)

def update():
    pass

t = QtCore.QTimer()
t.timeout.connect(update)
t.start(50)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
