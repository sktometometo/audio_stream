# audio_stream

These packages provides streaming audio playing and publishing capabilities.

There is already the [audio_common](http://wiki.ros.org/audio_common) packages with similar functions. But their topic message does not contain meta audio information such as sampling rate, data width and channels.
Our package will publish not only audio data itself but also meta audio information.

## Nodes

Please see README.md in the audio_stream package

## Demo

```
$ roslaunch audio_stream_demo audio_stream_demo.launch
```

## LICENSE

This repository is ditributed under [the BSD Clause 3 License](https://opensource.org/licenses/BSD-3-Clause) and All rights reserves at Koki Shinjo except sample audio files under audio_stream_demo/sound/ directory.

The sample audio files are retrieved from [Music Atelier Amacha](https://amachamusic.chagasi.com) and all rights of them belongs to Amacha.
