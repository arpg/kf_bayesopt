#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
plt.rc('axes', labelsize=22, linewidth = 3.0)
plt.ion()
params = {'legend.fontsize': 20,
          'legend.handlelength': 2}
plt.rcParams.update(params)
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib import cm
from matplotlib.colors import Normalize

flag = 1

def plot_x(ros_data):
    if ros_data.layout.dim[1].size == 0:
        return
    rospy.loginfo(rospy.get_caller_id() + 'I got the data need to be plotted')
    #print(len(ros_data.data))
    xlen = ros_data.layout.dim[1].size
    if ros_data.layout.dim[0].size == 1 :
        plotNum = 5 #mean, lower/upper bound, acquisition function, x
        _sgAcData   = ros_data.data[0:(plotNum*xlen)] #extrack the mean, bound and acquisition
        _sampleData = ros_data.data[(plotNum*xlen):len(ros_data.data)] #extract the sampled data
    else :
        plotNum = 2 #mean, acquistion function
        _sgAcDataX  = ros_data.data[0:xlen]
        _sgAcDataSg = ros_data.data[xlen : (xlen*xlen+xlen)]
        _sgAcDataAc = ros_data.data[(xlen*xlen+xlen) : (plotNum*xlen*xlen+xlen)]
        _sampleData = ros_data.data[(plotNum*xlen*xlen+xlen):len(ros_data.data)]
    lenSampleData = len(_sampleData)
    _sampleX = []
    _sampleY = []
    if ros_data.layout.dim[0].size == 1 : #one dimension, length must be 2* numbers of sample
        _sampleX    = _sampleData[0:(lenSampleData/2)]
        _sampleY    = _sampleData[(lenSampleData/2):lenSampleData]
        sgAcData = np.asarray(_sgAcData) #from list to array. In fact numpy can directly treat list as numpy element and directly reshape it. I don't need copy the data
        sampleX = np.asarray(_sampleX)
        sampleY = np.asarray(_sampleY)
        reshapeSgAc = np.reshape(sgAcData,(plotNum, ros_data.layout.dim[1].size)) #each row is data such as mean, std deviation

        x   = reshapeSgAc[0,:]
        y   = reshapeSgAc[1,:]
        sl  = reshapeSgAc[2,:]
        su  = reshapeSgAc[3,:]
        c   = reshapeSgAc[4,:]

        plt.clf()
        plt.subplot(2,1,1)
        plt.plot(x, y, 'b-', label = "Mean",linewidth = 3.0) #label means legend here
        plt.plot(x, sl,'g-', label = "Lower and Upper Bound", linewidth = 3.0)
        plt.plot(x, su,'g-',linewidth = 3.0)
        plt.plot(sampleX,sampleY,'r*', markersize = 20.0, label = "Sample Points")
        plt.legend() #after add label, you need this line to show legend
        plt.ylabel('surrogateModel')
        plt.subplot(2,1,2)
        plt.plot(x,c,label = "AcValue", linewidth = 3.0)
        plt.ylabel('acquisitionFunction')
        plt.xlabel('x')
        plt.legend()
        #plt.axis("equal")
        plt.draw()
        plt.pause(0.01) #sholdn't pause too long or we cannot see the data
    else :
        #print(len(_sgAcDataSg))
        #print(len(_sgAcDataAc))
        #print(xlen)
        _sampleX    = _sampleData[0:(2*lenSampleData/3)] #2 dimension, length must be 3* numbers of sample
        _samplez    = _sampleData[(2*lenSampleData/3):lenSampleData]
        sampleX     = np.reshape(_sampleX, (len(_samplez),2))
        samplex     = sampleX[:,0]
        sampley     = sampleX[:,1]
        samplex     = np.reshape(samplex,(1,len(_samplez)))
        sampley     = np.reshape(sampley,(1,len(_samplez)))

        sgAcDataX   = np.asarray(_sgAcDataX)
        sgAcDataSg  = np.asarray(_sgAcDataSg)
        sgAcDataAc  = np.asarray(_sgAcDataAc)
        cooX,cooY   = np.meshgrid(sgAcDataX,sgAcDataX)
        yy          = np.reshape(sgAcDataSg, (xlen,xlen))
        cc          = np.reshape(sgAcDataAc, (xlen,xlen))
        
        global flag
        global ax
        global bx
        if flag == 1 :
            fig = plt.figure()
            ax = fig.add_subplot(211, projection='3d') #if specify ax outside of function as global variable. Error will occur
            bx = fig.add_subplot(212, projection='3d') #make them global in local function
            flag = 0
        #print(len(samplex))
        #print(len(sampley))
        #print(len(_samplez))
        #surf1 = ax.plot_surface(cooX,cooY,yy,cmap=cm.coolwarm, linewidth=0, antialiased=False)
        wire1 = ax.plot_wireframe(cooX,cooY,yy)
        scatter1 = ax.scatter(sampley,samplex,_samplez, c = 'r',s = 40)
        ax.set_xlabel('noise1')
        ax.set_ylabel('noise2')
        ax.set_zlabel('Cost')
        surf2 = bx.plot_surface(cooX,cooY,cc,cmap=cm.coolwarm, linewidth=0, antialiased=False)
        bx.set_xlabel('noise1')
        bx.set_ylabel('noise2')
        bx.set_zlabel('AF Value')
        plt.draw()
        plt.pause(0.02)
        ax.cla()
        bx.cla()


if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("sgAc", Float64MultiArray, plot_x)
    rospy.spin()