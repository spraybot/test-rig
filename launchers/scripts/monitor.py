#! /usr/bin/env python
import rospy
import rostopic

def _rostopic_hz(topics, window_size=-1, filter_expr=None, use_wtime=False, tcp_nodelay=False):
    """
    Periodically print the publishing rate of a topic to console until
    shutdown
    :param topics: topic names, ``list`` of ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    :param filter_expr: Python filter expression that is called with m, the message instance
    :param tcp_nodelay: Subscribe with the TCP_NODELAY transport hint if true
    """
    
    rt = rostopic.ROSTopicHz(window_size, filter_expr=filter_expr, use_wtime=use_wtime)
    for topic in topics:
        msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=True) # pause hz until topic is published
        # we use a large buffer size as we don't know what sort of messages we're dealing with.
        # may parameterize this in the future
        if filter_expr is not None:
            # have to subscribe with topic_type
            rospy.Subscriber(real_topic, msg_class, rt.callback_hz, callback_args=topic, tcp_nodelay=tcp_nodelay)
        else:
            rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback_hz, callback_args=topic, tcp_nodelay=tcp_nodelay)
        rospy.loginfo("subscribed to [%s]" % real_topic)
    rospy.loginfo("All topics are publishing")

    while not rospy.is_shutdown():
        rostopic._sleep(2.5)
        for topic in topics:
            hz_stat = rt.get_hz(topic)
            if hz_stat is None:
                rospy.logerr("Topic: " + topic + " isn't publishing any data")

if __name__ == "__main__":
    rostopic._check_master()
    rospy.init_node("monitor")
    _rostopic_hz(["/velodyne_packets", "/xsens/imu/data", "/mapping/right/image_raw", "/mapping/right/camera_info", 
    "/mapping/left/image_raw", "/mapping/left/camera_info"])


    