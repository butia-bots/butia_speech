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

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

class WavToMouth():

    def __init__(self, filepath="talk.wav"):
        self.filepath = os.path.join(AUDIO, filepath)
        self.audio = None
        self.data = None
        self.chunk = 2048

        self.max = 0
        self.min = 0

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

    def _convert_data_to_angle(self, data):
        self.output.data = []

        value = audioop.max(data, 2) / 100

        value = int(0.3059*value)

        value = map_range(value, self.min, self.max, 0, 100)

        send_value = abs(100 - value)
        
        self.output.data = [value, send_value]

        self.angle_publisher.publish(self.output)

    def read_audio(self):
        data_list = []
        while self.data:
            self.data = self.audio.readframes(self.chunk)
            data_list.append(self.data)

        self.max = float('-inf')
        self.min = float('inf')

        for data in data_list:
            value = audioop.max(data, 2) / 100
            value = int(0.3059*value)

            if value > self.max:
                self.max = value
            
            elif value < self.min:
                self.min = value

        for data in data_list:
            self.stream.write(data)
            self._convert_data_to_angle(data)
        self.output.data = [0, 100]
        self.angle_publisher.publish(self.output)
        self.stream.close()
        self.p.terminate()