#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import wave
import numpy as np
import colorednoise as cn

def generateColoredNoise( beta, max_range, sampling_rate, duration ):

    return np.clip( max_range * cn.powerlaw_psd_gaussian( beta, sampling_rate * duration ), -max_range, max_range )


def main():

    rospy.init_node( 'colored_noise_publisher' )

    # parameters
    loop_rate = int( rospy.get_param( '~loop_rate', 100 ) )
    buffer_size = int( rospy.get_param( '~buffer_size', 4096 ) )
    use_audio_common = bool( rospy.get_param( '~use_audio_common', False ) )
    beta = float( rospy.get_param( '~exponent', 1 ) )
    level = float( rospy.get_param( '~level', 0.5 ) )

    # import message modules
    if use_audio_common:
        from audio_common_msgs.msg import AudioData
        from audio_stream_msgs.msg import AudioInfo
    else:
        from audio_stream_msgs.msg import AudioData, AudioInfo

    # publishers
    publisher_data = rospy.Publisher( '~audio', AudioData, queue_size=1000 )
    publisher_info = rospy.Publisher( '~info', AudioInfo, queue_size=10 )

    # gen a default buffer
    msg_info = AudioInfo()
    msg_data = AudioData()
    msg_info.channels = 1
    msg_info.sampling_rate = 16000
    msg_info.width = 2
    A = level * ((2**8) ** 2)
    buffer_wave = generateColoredNoise( beta, A, msg_info.sampling_rate, 30 ).astype(np.uint16).tobytes()

    r = rospy.Rate( loop_rate )
    index = 0
    time_played = 0.0 # sec
    time_start = time.time()
    while not rospy.is_shutdown():

        rospy.loginfo('publish')

        # update msg_info header
        msg_info.header.stamp = rospy.Time.now()

        # fill msg_data.data
        if ( len(buffer_wave) - index ) < buffer_size:
            msg_data.data = buffer_wave[index:] + buffer_wave[:index+buffer_size-len(buffer_wave)]
            time_played += 1.0 * ( len(buffer_wave) - index ) / ( msg_info.width * msg_info.sampling_rate )
            index = index + buffer_size - len(buffer_wave)
        else:
            msg_data.data = buffer_wave[index:index+buffer_size]
            time_played += 1.0 * buffer_size / ( msg_info.width * msg_info.sampling_rate )
            index = index + buffer_size

        time_passed = time.time() - time_start
        while time_played > time_passed:
            time_passed = time.time() - time_start
            if rospy.is_shutdown():
                return
            r.sleep()

        # publish them
        publisher_data.publish( msg_data )
        publisher_info.publish( msg_info )

if __name__=='__main__':
    main()
