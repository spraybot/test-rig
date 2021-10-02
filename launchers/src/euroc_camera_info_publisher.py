#v1************************************************************************************
# This is developed at Carnegie Mellon University in collaboration with Autel Robotics 
#                                                                                      
# PI:                                                                                  
# George Kantor                                                                        
#                                                                                      
# Authors:                                                                              
# Weizhao Shao                                                                         
# Cong Li                                                                              
# Srinivasan Vijayarangan                                                              
#                                                                                      
# Please refer to the contract document for details on license/copyright information.  
#**************************************************************************************
#!/usr/bin/env python

import roslib
import rospy
import sys
import pdb
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager

class CameraInfoPublisher:

    def __init__(self):
        self._namespace = rospy.get_namespace()
        rospy.loginfo('Using namespace:'+self._namespace)
        self._calib_filename = rospy.get_param('~calib_filename')
        self._cam_mgr = CameraInfoManager('camera',url=self._calib_filename,
                                          namespace=self._namespace[:-1])
        self._cam_mgr.loadCameraInfo()
        self._cam_info = self._cam_mgr.getCameraInfo()
        self._cam_info.header.frame_id = 'camera'

        self._sub = rospy.Subscriber(self._namespace+'image_raw',Image,self.callback)

        self._pub = rospy.Publisher(self._namespace+'camera_info',CameraInfo,queue_size=100)

    def callback(self,data):
        #copy the timestamp and publish the camera info msg
        rospy.loginfo('Publishing image with stamp:'+str(data.header.stamp))
        self._cam_info.header.stamp = data.header.stamp
        self._pub.publish(self._cam_info)
        
def main(args):

    rospy.init_node('camera_info_pulisher')

    
    caminfopub = CameraInfoPublisher()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
