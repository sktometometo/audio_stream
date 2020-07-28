#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import sys
import time
import wave

from audio_stream_msgs.msg import AudioInfo

def main():

    rospy.init_node( 'audio_info_publisher' )

    # parameters
    loop_rate = rospy.get_param( '~loop_rate', 100 )
    channels = rospy.get_param( '~channels', 1 )
    width = rospy.get_param( '~width', 1 )
    framerate = rospy.get_param( '~sampling_rate', 16000 ) 

    # publisher
    topicname_info = '~info'
    publisher_info = rospy.Publisher( topicname_info, AudioInfo, queue_size=1 )

    # message
    msg_info = AudioInfo()
    msg_info.channels = channels
    msg_info.sampling_rate = framerate
    msg_info.width = width
    msg_info.header.stamp = rospy.Time.now()
    publisher_info.publish( msg_info )

    r = rospy.Rate( loop_rate )
    while not rospy.is_shutdown():
        msg_info.header.stamp = rospy.Time.now()
        r.sleep()
        publisher_info.publish( msg_info )

if __name__=='__main__':
    main()
