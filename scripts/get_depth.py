#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2 
time_now = 0.
def callback(data):
    global time_now
    time_diff =data.header.stamp - time_now
    rospy.loginfo(1/time_diff.to_sec())
    time_now = data.header.stamp

def listener():
    global time_now
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pt_getter', anonymous=True)
    print('getting IMAGE!')

    rospy.Subscriber("/voxel_grid/output", PointCloud2, callback)
    time_now= rospy.get_rostime()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
