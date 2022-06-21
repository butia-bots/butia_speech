#!/usr/bin/env python3

import rospy
from butia_speech.srv import SpeechToText, SpeechToTextResponse
from speech_recognition import Microphone, Recognizer, WaitTimeoutError, AudioData
import os
import numpy as np
from espnet_model_zoo.downloader import ModelDownloader
from espnet2.bin.asr_inference import Speech2Text

tag = 'Shinji Watanabe/spgispeech_asr_train_asr_conformer6_n_fft512_hop_length256_raw_en_unnorm_bpe5000_valid.acc.ave'

def handle_recognition(req):
    with Microphone(sample_rate=16000) as source:
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        rospy.loginfo('Listening ...')
        try:
            audio = recognizer.listen(source, phrase_time_limit=10, timeout=3)
            fs = audio.sample_rate
            audio = np.frombuffer(audio.frame_data, np.int16)
            text, *_ = speech2text(audio)[0]
        except WaitTimeoutError:
            text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    rospy.init_node('speech_recognizer')
    d = ModelDownloader()
    speech2text = Speech2Text(
        **d.download_and_unpack(tag),
        device="cpu", # cuda
        minlenratio=0.0,
        maxlenratio=0.0,
        ctc_weight=0.3,
        beam_size=10,
        batch_size=0,
        nbest=1
    )
    recognizer = Recognizer()
    #deepspeech_model = deepspeech.Model(deepspeech_model_dir + 'deepspeech-0.9.3-models.pbmm')
    #deepspeech_model.enableExternalScorer(deepspeech_model_dir + 'deepspeech-0.9.3-models.scorer')
    #stt = pipeline(task="automatic-speech-recognition", model="facebook/wav2vec2-base-960h")
    recognition_service = rospy.Service('/butia_speech/asr/transcribe', SpeechToText, handle_recognition)
    rospy.spin()
