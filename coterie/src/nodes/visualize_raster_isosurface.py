#!/usr/bin/env python

import rospy
from coterie_msgs.msg import RasterSet
import numpy as np
from mayavi import mlab


def callback(setMsg):
    data = np.array(setMsg.byteArray.data)
    layout = setMsg.byteArray.layout
    block = np.reshape(data, (layout.dim[0].size, layout.dim[1].size, layout.dim[2].size))
    contour.mlab_source.reset(scalars=block)
    fig = mlab.gcf()
    fig.scene.reset_zoom()

def listener():
    rospy.init_node('set_isosurface')
    rospy.Subscriber("se2", RasterSet, callback)
    mlab.show()
    rospy.spin()

if __name__ == '__main__':
    m_data = np.array([[[0,0,0],[0,0,0],[0,0,0]],[[0,0,0],[0,1,0],[0,0,0]],[[0,0,0],[0,0,0],[0,0,0]]])
    contour = mlab.contour3d(m_data, contours=[0.5])
    lastShape = m_data.shape

    listener()