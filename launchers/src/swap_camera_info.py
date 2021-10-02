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

import rosbag
import os
import sys
import rospy
import pdb
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager

def main(args):

    if len(args) < 2:
        print('Usage: python swap_camera_info.py <path-to-bag-files>')
        sys.exit(-1)


    bagpath = args[1]
    bagfiles = []
    for bag in os.listdir(bagpath):

        if '.bag' in bag:
            bagfiles.append(os.path.join(bagpath,bag))

    #pdb.set_trace()

    for bag in bagfiles:

        print('Processing bag :',bag)
        #open the bag file and read the left and right camera info topics
        inbag = rosbag.Bag(bag,'r')

        right_info = None
        left_info = None

        for topic, msg, t in inbag.read_messages():

            if 'right/camera_info' in topic:
                right_info = msg

            elif 'left/camera_info' in topic:
                left_info = msg

            if right_info is not None and left_info is not None:
                break

        inbag.close()

        #pdb.set_trace()
        
        #reopen it and read the rest of the topics
        inbag = rosbag.Bag(bag,'r')
        #write to a new bag
        outbag = rosbag.Bag(bag[:-6]+'_modified_'+bag[-6:],'w')

        for topic,msg,t in inbag.read_messages():

            if 'right/image_raw' in topic:
                prefix=topic[:-len('right/image_raw')]
                left_info.header.stamp = msg.header.stamp
                outbag.write(prefix+'left/camera_info',left_info,t)
                topic = prefix+'left/image_raw'

            elif 'left/image_raw' in topic:
                prefix=topic[:-len('left/image_raw')]
                right_info.header.stamp = msg.header.stamp
                outbag.write(prefix+'right/camera_info',right_info,t)
                topic = prefix+'right/image_raw'

            if 'camera_info' in topic:
                continue
                
            outbag.write(topic,msg,t)

        outbag.close()
        inbag.close()
        
                            
    rospy.init_node('camera_info_swapper')

    #get
    

if __name__ == '__main__':
    main(sys.argv)
