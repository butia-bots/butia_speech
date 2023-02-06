# butia_speech package

## 1. Description

This package provides some tools to make the robot DoRIS speak and listen. We use ROS packages and Python libraries to be possible to play audios and to recognize of speech (a.k.a speech-to-text and text-to-speech). 

___
## 2. Requirements

You will need:
- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- pip

___
## 3. Download dependencies

To install the dependencies you will need to run the _install.sh_ with super user permission.

```
chmod +x install.sh
sudo ./install.sh
```

The dependencies will be installed.

___
## 4. Nodes

We developed some nodes that will go help to make the speech-to-text (STT) and text-to-speech (TTS). They can be seen in _nodes_ directory.

The **audio_player** node is a ROS Service that receive a audio path and try to reproduce using the PyAudio and Wave libraries (it can be seen in _src/butia_speech/wav_to_mouth.py_). This node is also used to send movements to the jaw used in the mechatronic face used in the robot DoRIS.

The **detector_hotword_node** is a ROS Topic that, when called, listens and waits for a keyword. A possible use to this topic is to make the robot listen a word to make some specific task. We use the _pvporcupine_ Python library to make the recognize (it can be seen in _src/butia_speech/detect_hotword.py_).

The **speech_recognizer** node is a ROS Service that, when called, listens and returns a text of what was spoken.

The **speech_synthesizer** node is a ROS Service and a ROS Topic that receive a text and a language (english by default, other languages aren't implemented yet). We developed the node with service and topic because the ROS Topic is no-blocking, so we can make others tasks while the robot speaks. 

___
## 5. Services and messages

We created some services and messages to make possible the communication between ours packages. 

### 5.1 Services

- AudioPlayer.srv receives a audio path as string and returns a success boolean;
- SpeechToText.srv returns a text as string;
- SynthesizeSpeech.srv receives a text and a language as string and returns a success boolean.

### 5.2. Messages

- SynthesizeSpeechMessage.msg uses the text and a language as string.

___
## 6. Usage

The first use of the nodes may take a while as the models are being downloaded.

To start the nodes you must run:
```
rosrun butia_speech <node_name>.py
```

* <node_name> available are: audio_player, detector_hotword_node, speech_recognizer, speech_synthesizer.

## License & Citation
If you find this package useful, consider citing it using:
```
@misc{butia_speech,
    title={Butia Speech Package},
    author={{ButiaBots}},
    howpublished={\url{https://github.com/butia-bots/butia_speech/}},
    year={2022}
}
```
<p align="center"> 
  <i>If you liked this repository, please don't forget to starred it!</i>
  <img src="https://img.shields.io/github/stars/butia-bots/butia_speech?style=social"/>
</p>
