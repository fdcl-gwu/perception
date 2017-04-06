#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image 
time_now = 0.
import numpy as np

class camera_filter(object):
    def __init__(self):
        self.rate = 10
        rospy.set_param('camera_rate', self.rate)
        self.depth_pt = PointCloud2()
        
        self.rate_rgb = 3
        rospy.set_param('camera_rgb_rate', self.rate_rgb)
        self.rgb = Image() 
       
        rospy.init_node('camera_filter')
        self.pub = rospy.Publisher('/camera/depth/points_frate', PointCloud2, queue_size=10)
        self.pub_rgb = rospy.Publisher('/camera/rgb/image_frate', Image, queue_size=10)
        # subscriber name need to changed for xtion pro 
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.updatePCL)
        rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.updateRGB)
        rospy.loginfo('camera_filter node initialized')
    
    def get_frame_rate(self):
        self.rate = rospy.get_param('camera_rate')
    
    def updateRGB(self,msg):
        self.rgb = msg
   
    def updatePCL(self,msg):
        self.depth_pt = msg
  
    def run(self):
        while not rospy.is_shutdown():
            self.get_frame_rate()
            rate = rospy.Rate(self.rate)
            self.pub.publish(self.depth_pt)
            self.pub_rgb.publish(self.rgb)
            rate.sleep()

# def callback(data):
#     global time_now
#     time_diff =data.header.stamp - time_now
#     rospy.loginfo(1/time_diff.to_sec())
#     time_now = data.header.stamp
# 
# def listener():
#     global time_now
#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # node are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('pt_getter', anonymous=True)
#     print('getting IMAGE!')
# 
#     rospy.Subscriber("/voxel_grid/output", PointCloud2, callback)
#     time_now= rospy.get_rostime()
# 
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
# 
# def publisher():
#     pub = rospy.Publisher('test', String, queue_size=10)
#     rospy.init_node('tester')
#     rate = rospy.Rate(10)
#     rate_array = range(1,20) 
#     k = 0
#     while not rospy.is_shutdown():
#         rate = rospy.Rate(rate_array[k])
#         rospy.loginfo('hello '+str(rate_array[k]))
#         pub.publish('rate')
#         if k >= len(rate_array)-1:
#             k = 0
#         else:
#             k+=1
#         rate.sleep()

if __name__ == '__main__':
   #  listener()
   # publisher()
   frate = camera_filter()
   frate.run()
