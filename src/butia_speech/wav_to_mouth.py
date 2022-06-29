import audioop
import os
import wave
import rospy
import rospkg
import pyaudio
import numpy as np
from std_msgs.msg import Int16MultiArray

BUTIA_SPEECH_PKG = rospkg.RosPack().get_path("butia_speech")
AUDIO = os.path.join(BUTIA_SPEECH_PKG, "audios/")

class WavToMouth():

    def __init__(self, filepath="talk.wav"):
        self.filepath = os.path.join(AUDIO, filepath)
        self.audio = None
        self.data = None
        self.chunk = 2048

        self._read_data_of_audio()

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format = self.p.get_format_from_width(self.audio.getsampwidth()),
                                                              channels=self.audio.getnchannels(),
                                                              rate=self.audio.getframerate(),
                                                              output=True)

        self.output = Int16MultiArray()
        self.angle_publisher = rospy.Publisher("mouth", Int16MultiArray, queue_size=1)
        self.rate = rospy.Rate(30) # 100hz

    def _read_data_of_audio(self):
        self.audio = wave.open(self.filepath, "rb")
        self.data = self.audio.readframes(self.chunk)

    def _convert_data_to_angle(self):
        self.output.data = []

        value = audioop.max(self.data, 2) / 100

        value = int(0.3059*value)
        send_value = abs(100 - value)
        new_send_value = int((send_value / (100/13)) + 22.0)
        
        self.output.data = [new_send_value, value]

        self.angle_publisher.publish(self.output)

    def read_audio(self):
        while self.data:
            self.stream.write(self.data)
            self.data = self.audio.readframes(self.chunk)
            self._convert_data_to_angle()
        self.stream.close()
        self.p.terminate()