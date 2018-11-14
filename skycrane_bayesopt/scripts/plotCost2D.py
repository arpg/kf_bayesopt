#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np

class plotClass:
    his = [] #member, if you need save some data. "plotClass.countHis" to use it
    sizeMsg  = 0
    runTimes = 0
    def __init__(self):
        self.subscriber = rospy.Subscriber("costVsNoise",
            Float64MultiArray, self.callback,  queue_size = 10)
    def callback(self, ros_data):
        rospy.loginfo(rospy.get_caller_id() + 'I got the data need to be plotted')
        #ros_data.data in c++ is vector double, in python should be list
        plotClass.sizeMsg = len(ros_data.data)
        plotClass.runTimes = ros_data.layout.dim[0].size
        plotClass.his.append(ros_data.data) #if append 10 times, then length is 10. No matter what the data is
        if ros_data.layout.dim[0].label == 'startPlot':
            plotClass.start_plot(self)
    
    #python class needs self as parameter
    def start_plot(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        #the very original data is like [1,2,3,4,5,6...], 1 is x, 2 is y, 3 is cost, 4 is x again, 5 is y, 6 is cost....
        allData = np.asarray(plotClass.his) #from list to array
        allData = np.reshape(allData,(len(plotClass.his), plotClass.sizeMsg)) #each row is state at one timestamp
        #after reshape, the data is like x' = [1,4], y' = [2,5], z = [3,6]
        x = allData[:,0] #pnoise[0] or onoise[0]
        y = allData[:,1] #pnoise[1] or onoise[1]
        z = allData[:,2] #cost

        #In fact the data after reshape is like x' = [1,1,1,2,2,2,3,3,3], y' = [1,2,3,1,2,3,1,2,3], z = [1,2,3,4,5,6,7,8,9](see the publisher c++ code plotCost2D data structure)
        #the following code should work the same as 
        # x = np.unique(x), y = np.unique(y), x,y = np.meshgrid(x,y)
        # then z = 1 's coordinate should be (1,1), z = 2 's coordinate should be (1,2)....
        x = np.reshape(x,(plotClass.runTimes,plotClass.runTimes))
        x = np.transpose(x)
        y = np.reshape(y,(plotClass.runTimes,plotClass.runTimes))
        y = np.transpose(y)
        z = np.reshape(z,(plotClass.runTimes,plotClass.runTimes))
        z = np.transpose(z)

        surf = ax.plot_surface(x,y,z,cmap=cm.coolwarm, linewidth=0, antialiased=False)
        # Customize the z axis.
        #ax.set_zlim(0.01, 30)
        #ax.zaxis.set_major_locator(LinearLocator(10))
        #ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
        
        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.5, aspect=5)
        #plot state
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        #plt.title('Test')
        #plt.grid(True)
        #plt.savefig("test.png")
        plt.show()

# def floatCallback(data):
#     rospy.loginfo(rospy.get_caller_id() + 'I heard %f', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    emp1 = plotClass()

    #rospy.Subscriber('floatPub', Float64, floatCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()