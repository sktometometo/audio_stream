#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import wave
import numpy as np

def generateWhiteNoise( std, max_range, f0, duration ):
    return np.clip( np.array( [ random.gauss( 0.0, std ) for i in range( f0, duration ) ] ), -max_range, max_range )

def main():

    rospy.init_node( 'white_noise_publisher' )

    # parameters
    loop_rate = rospy.get_param( '~loop_rate', 100 )
    buffer_size = rospy.get_param( '~buffer_size', 4096 )
    use_audio_common = rospy.get_param( '~use_audio_common', False )
    topicname_data = '~audio'
    topicname_info = '~info'

    # import message modules
    if use_audio_common:
        from audio_common_msgs.msg import AudioData
        from audio_stream_msgs.msg import AudioInfo
    else:
        from audio_stream_msgs.msg import AudioData, AudioInfo

    # publishers
    publisher_data = rospy.Publisher( topicname_data, AudioData, queue_size=1000 )
    publisher_info = rospy.Publisher( topicname_info, AudioInfo, queue_size=10 )

    # gen a default buffer
    msg_info.channels = 1
    msg_info.sampling_rate = 16000
    msg_info.width = 2

    r = rospy.Rate( loop_rate )

    A = 0.1 * ((2**8) ** 2)
    buffer_wave = generateWhiteNoise( 0.8 * A, A, msg_info.sampling_rate, 30 ).astype(np.uint16).tobytes()

    index = 0
    time_played = 0.0 # sec
    time_start = time.time()
    while not rospy.is_shutdown():

        # update msg_info header
        msg_info.header_stamp = rospy.Time.now()

        # fill msg_data.data
        if ( len(bufer_wave) - index ) < buffer_size:
            msg_data.data = buffer_wave[index:] + buffer_wave[:index+buffer_size-len(buffer_wave)]
            time_played += 1.0 * ( len(buffer_wave) - index ) / ( msg_info.width * msg_info.sampling_rate )
            index = index + buffer_size - len(buffer_wave)
        else:
            msg_data.data = buffer_wave[index:index+buffer_size]
            time_played += 1.0 * buffer_size / ( msg_info.width * msg_info.sampling_rate )
            index = index + buffer_size

        time_passed = time.time() - time_start
        while time_played - time_passed > 0:
            time_passed = time.time() - time_start
            if rospy.is_shutdown():
                return
            r.sleep()

        # publish them
        publisher_data.publish( msg_data )
        publisher_info.publish( msg_info )

if __name__=='__main__':
    main()
