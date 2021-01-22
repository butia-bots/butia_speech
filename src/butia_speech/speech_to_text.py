import os
import time
import deepspeech
from speech_recognition import Recognizer
import numpy as np

deepspeech_model_dir = os.path.join(os.path.dirname(__file__), '../../include/deepspeech/')

class ButiaSpeechToText(Recognizer):
    def __init__(self):
        super(SpeechToText, self).__init__()
        self.deepspeech_model = deepspeech.Model(deepspeech_model_dir + 'deepspeech-0.9.3-models.pbmm')
        self.deepspeech_model.enableExternalScorer(deepspeech_model_dir + 'deepspeech-0.9.3-models.scorer')

    def recognize_deepspeech(self, audio_data):
        raw_data = audio_data.get_raw_data(convert_rate=16000, convert_width=2)
        buffer = np.frombuffer(raw_data, dtype=np.int16)
        return self.deepspeech_model.stt(buffer)
