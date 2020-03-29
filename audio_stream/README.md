# audio_stream

This package provides audio_stream streaming and playing nodes.

## Dependency

- [PyAudio](https://pypi.org/project/PyAudio/)

## Nodes

### audio_stream_player.py

#### Subscribers

- '~audio' ( audio_stream_msgs/AudioData or audio_common_msgs/AudioData )

audio stream data topic. message type of this topic will be audio_common_msgs/AudioData if '~use_audio_common' parameter is True.

- '~info' ( audio_stream_msgs/AudioInfo )

meta data topic of audio stream. (sampling rate, channels and data width)

#### Parameters

- '~loop_rate' ( int, default: 100 )

main loop rate

- '~use_audio_common' ( bool, default: False )

use audio_common_msgs/AudioData instead of audio_stream_msgs/AudioData

### audio_stream_publisher.py

#### Publishers

- '~audio' ( audio_stream_msgs/AudioData or audio_common_msgs/AudioData )

audio stream data topic. message type of this topic will be audio_common_msgs/AudioData if '~use_audio_common' parameter is True.

- '~info' ( audio_stream_msgs/AudioInfo )

meta data topic of audio stream. (sampling rate, channels and data width)

#### Parameters

- '~loop_rate' ( int, default: 100 )

publishing loop rate

- '~is_loop' ( bool, default: False )

if true, audio stream will be looped.

- '~filename' ( string )

location of a wave file to play

- '~buffer_size' ( int, default: 4096 )

the size of buffer of each message

- '~max_precede' ( float, default 10.0 )

a duration to preceding of publishing time to playing time

- '~use_audio_common' ( bool, default: False )

use audio_common_msgs/AudioData instead of audio_stream_msgs/AudioData
