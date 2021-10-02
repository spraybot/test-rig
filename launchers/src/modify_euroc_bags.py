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
import sys
import pdb

from sensor_msgs.msg import Imu

def main(args):

    inbag = rosbag.Bag(args[1],'r')

    imu_msgs = []

    print('Opening bag to read imu messages...')
    for topic, msg, t in inbag.read_messages():

        if '/imu0' in topic:
            imu_msgs.append(msg)

    inbag.close()
    print('Read ',len(imu_msgs),' imu messages.')

    print('Now writing a new bag with the other messages...')

    inbag = rosbag.Bag(args[1],'r')
    outbag = rosbag.Bag('modified.bag','w')
    
    for topic, msg, t in inbag.read_messages():

        if '/imu0' in topic:
            pass

        else:
            outbag.write(topic,msg,t)

    inbag.close()

    print('Updating the bag with IMU messages...')
    for imu in imu_msgs:
        outbag.write('/imu0',imu,imu.header.stamp)
                
    outbag.close()

    
    
if __name__ == '__main__':
    main(sys.argv)
    
