#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
plt.rc('axes', labelsize=22)
import numpy as np


class plotClass:
    his = [] #member, if you need save some data. "plotClass.countHis" to use it
    sizeMsg = 0
    def __init__(self):
        self.subscriber = rospy.Subscriber("costVsNoise",
            Float64MultiArray, self.callback,  queue_size = 10)
    def callback(self, ros_data):
        rospy.loginfo(rospy.get_caller_id() + 'I got the data need to be plotted')
        #ros_data.data in c++ is vector double, in python should be list
        plotClass.sizeMsg = len(ros_data.data)
        plotClass.his.append(ros_data.data) #if append 10 times, then length is 10. No matter what the data is
        if ros_data.layout.dim[0].label == 'startPlot':
            plotClass.start_plot(self)
    
    def start_plot(self):
        allData = np.asarray(plotClass.his) #from list to array
        allData = np.reshape(allData,(len(plotClass.his), plotClass.sizeMsg)) #each row is state at one timestamp
        y = allData[:,1]
        x = allData[:,0]
        plt.grid(True)

        #following is to plot histogram
        plt.xlim(1.8,3.0)
        plt.xlabel('cost_NIS')
        plt.ylabel('numbers of each bin')
        plt.hist(y,bins = 'auto',normed = 0) #bins mean how many bins in the plot
        plt.title("histogram of Cost")
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