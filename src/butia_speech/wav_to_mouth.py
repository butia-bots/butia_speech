import audioop
import os
import wave
import rospy
import rospkg
import pyaudio
import numpy as np
import struct
from std_msgs.msg import Int16MultiArray, Int16

BUTIA_SPEECH_PKG = rospkg.RosPack().get_path("butia_speech")
AUDIO = os.path.join(BUTIA_SPEECH_PKG, "audios/")

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
class WavToMouth():

    def __init__(self):
        self.chunk_size = rospy.get_param("chunk_size", 2048)

        self.data = []

        self.max = 0
        self.min = 0

        self.audio = None
        self.audio_info = None

        self.output = Int16MultiArray()
        self.mouth_gain = rospy.get_param("mouth_gain", 2.5)
        self.angle_publisher = rospy.Publisher("mouth", Int16MultiArray, queue_size=1)
        self.mouth_debug_publisher = rospy.Publisher("mouth_debug", Int16, queue_size=1)
    
    def divide_audio_in_chunks(self, audio_data, num_bytes=2):
        chunks = []
        for start in range(0, len(audio_data), self.chunk_size*num_bytes):
            end = min(start + self.chunk_size*num_bytes, len(audio_data))
            chunks.append(audio_data[start:end])
        return chunks

    def _read_data_of_audio(self):
        self.audio = wave.open(self.filepath, "rb")
        audio_data = self.audio.readframes(self.audio.getnframes())
        self.data += self.divide_audio_in_chunks(audio_data, num_bytes=self.audio.getsampwidth())

    def set_filepath(self, filepath):
        self.filepath = os.path.join(AUDIO, filepath)
        self._read_data_of_audio()
        self._open_stream()
    
    def set_data_and_info(self, data, info):
        self.data += self.divide_audio_in_chunks(data)
        self.audio_info = info
        self._open_stream()

    def _open_stream(self):
        self.p = pyaudio.PyAudio()

        if self.audio is not None:
            self.stream = self.p.open(format = self.p.get_format_from_width(self.audio.getsampwidth()),
                                      channels=self.audio.getnchannels(),
                                      rate=self.audio.getframerate(),
                                      output=True)
        elif self.audio_info is not None:
            self.stream = self.p.open(format=int(self.audio_info.sample_format),
                                      channels=self.audio_info.channels,
                                      rate=self.audio_info.sample_rate,
                                      output=True)
        
        self.audio = None
        self.audio_info = None

    def _compute_chunk_rms(self, audio_chunk):
        """Compute the RMS value of an audio chunk."""
        count = len(audio_chunk) // 2
        format = "%dh" % count
        shorts = struct.unpack(format, audio_chunk)
        sum_squares = sum(s ** 2 for s in shorts)
        rms = np.sqrt(sum_squares / count)
        return rms

    def _normalize_rms(self, rms_value, gain=2.5, max_rms=32768):
        """Normalize the RMS value to a range from 0 to 100."""
        rms_value = min(rms_value * gain, max_rms)
        return min(max(int((rms_value / max_rms) * 100), 0), 100)

    def play_chunk(self):
        if len(self.data) > 0:
            data = self.data.pop(0)

            self.stream.write(data)

            rms = self._compute_chunk_rms(data)
            mouth_angle = self._normalize_rms(rms, gain=self.mouth_gain)
            self.output.data = [mouth_angle, abs(100 - mouth_angle)]
            self.angle_publisher.publish(self.output)
            self.mouth_debug_publisher.publish(Int16(mouth_angle))

    def play_all_data(self):

        while len(self.data) > 0:
            self.play_chunk()
        
        self.output.data = [0, 100]
        self.angle_publisher.publish(self.output)
        self.stream.close()
        self.p.terminate()