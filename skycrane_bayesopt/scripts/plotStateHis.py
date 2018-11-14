#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
plt.rc('axes', labelsize=22)
import numpy as np

class plotClass:
    #countHis = [] #member, if you need save some data. "plotClass.countHis" to use it
    def __init__(self):
        self.subscriber = rospy.Subscriber("stateHis",
            Float64MultiArray, self.callback,  queue_size = 10)
    def callback(self, ros_data):
        rospy.loginfo(rospy.get_caller_id() + 'I got the data need to be plotted')
        stateHisData = ros_data.data #ros_data.data in c++ is vector double, in python should be list
        y = np.asarray(stateHisData) #from list to array
        y = np.reshape(y,(ros_data.layout.dim[1].size, ros_data.layout.dim[0].size)) #each row is state at one timestamp
        y = np.transpose(y) 
        x = 0.1*np.arange(0.0,ros_data.layout.dim[1].size,1.0) #dim[1] is EKF iteration number
        #print(len(x))

        #plot state
        #for i in range(0,ros_data.layout.dim[0].size): #dim[0] is number of states
        plt.title('skyCraneState')
        oneState = y[0,:]
        plt.subplot(6,1,1)
        plt.ylabel('xi')
        plt.plot(x, oneState)
        plt.ylim(-1.5,1.5)

        oneState = y[1,:]
        plt.subplot(6,1,2)
        plt.ylabel('xidot')
        plt.plot(x, oneState)
        plt.ylim(-0.5,1.0)

        oneState = y[2,:]
        plt.subplot(6,1,3)
        plt.ylabel('z')
        plt.plot(x, oneState)
        plt.ylim(18,22)

        oneState = y[3,:]
        plt.subplot(6,1,4)
        plt.ylabel('zdot')
        plt.plot(x, oneState)
        plt.ylim(-0.5,0.5)

        oneState = y[4,:]
        plt.subplot(6,1,5)
        plt.ylabel('theta')
        plt.plot(x, oneState)
        plt.ylim(-0.1,0.1)

        oneState = y[5,:]
        plt.subplot(6,1,6)
        plt.ylabel('thetadot')
        plt.plot(x, oneState)
        plt.ylim(-0.1,0.1)
        plt.xlabel('time')
        plt.grid(True)
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