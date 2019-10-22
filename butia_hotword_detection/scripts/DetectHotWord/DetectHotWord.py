import sys
import os
import struct
import numpy as np
import pyaudio
import rospkg

#PACK_DIR = rospkg.RosPack().get_path('butia_hotword_detection')
#sys.path.append(os.path.join(PACK_DIR, '/include/binding'))

sys.path.append(os.path.join(os.path.dirname(__file__), '../../include/binding/'))

from porcupine import Porcupine

class DetectHotWord():
    def __init__(self, library_path, model_path, keyword_path, sensitivity):
        self.handle = Porcupine(library_path, model_path, keyword_file_path=keyword_path, sensitivity=sensitivity)
        self.mic = None

    def hear(self):
        self.pa = pyaudio.PyAudio()
        audio_stream = self.pa.open(
            rate=self.handle.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.handle.frame_length)
        self.mic = audio_stream

    def process(self):
        if self.mic is not None:
            pcm = self.mic.read(self.handle.frame_length)
            pcm = struct.unpack_from("h" * self.handle.frame_length, pcm)
            recorded_frames = []
            recorded_frames.append(pcm)
            result = self.handle.process(pcm)
            if result:
                return True
        return False

    def __del__(self):
        self.mic.close()
        self.pa.terminate()
        self.handle.delete()
        # In future, save audio file

    
