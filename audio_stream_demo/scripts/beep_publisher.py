#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import sys
import time
import wave
import numpy as np

def generateSin( A, f0, fs, duration ):
    return A * np.sin( 2 * np.pi * f0 * np.arange(int(duration*fs)) / fs )

def generateBeepSingle( A, f0, fs, duration, ratio_sleep ):
    duration_sound = duration * ( 1 - ratio_sleep )
    duration_sleep = duration * ratio_sleep
    if type(f0) is not list:
        return np.concatenate([generateSin( A, f0, fs, duration_sound ), np.zeros( int(duration_sleep*fs) )])
    else:
        return np.concatenate([ sum([ generateSin( A, freq, fs, duration_sound ) for freq in f0 ]) / len(f0), \
                                 np.zeros( int(duration_sleep*fs) ) ])

def generateBeep( A, f0, f1, fs, duration, ratio_sleep ):
    duration_single = 1.0 / f1
    beep_times = int( duration / duration_single )
    return np.concatenate([ generateBeepSingle( A, f0, fs, duration_single, ratio_sleep ) for x in range(beep_times) ])

def main():

    rospy.init_node( 'beep_publisher' )

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

    # message
    msg_data = AudioData()
    msg_info = AudioInfo()

    # gen a default buffer
    msg_info.channels = 1
    msg_info.sampling_rate = 16000
    msg_info.width = 2

    frequency_f0 = 440 # 基本周波数
    frequency_beep = 1.0 # beepの周期

    buffer_beep = generateBeep( 0.1*((2**8) ** 2), [ 440, 480, 500, 520, 400, 300, 600, 800 ], frequency_beep, msg_info.sampling_rate, 10.0, 0.5 ).astype(np.uint16).tobytes()

    r = rospy.Rate( loop_rate )
    index = 0
    time_played = 0.0; # sec
    time_start = time.time()
    while not rospy.is_shutdown():
        rospy.loginfo('publish')
        if index > len(buffer_beep):
            index = 0
        msg_info.header.stamp = rospy.Time.now()
        if ( len(buffer_beep) - index) < buffer_size:
            msg_data.data = buffer_beep[index:]
            time_played += 1.0 * ( len(buffer_beep) - index ) / ( msg_info.width * msg_info.sampling_rate )
        else:
            msg_data.data = buffer_beep[index:index+buffer_size]
            time_played += 1.0 * buffer_size / ( msg_info.width * msg_info.sampling_rate )
        time_passed = time.time() - time_start
        while time_played - time_passed > 0:
            time_passed = time.time() - time_start
            if rospy.is_shutdown():
                return
            r.sleep()
        publisher_data.publish( msg_data )
        publisher_info.publish( msg_info )
        index += buffer_size

if __name__=='__main__':
    main()
