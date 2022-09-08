#!/usr/bin/env python3

import rospy
from butia_speech.srv import SpeechToText, SpeechToTextResponse
from speech_recognition import Microphone, Recognizer, WaitTimeoutError, AudioData
import os
import numpy as np
from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC
import torch

def handle_recognition(req):
    with Microphone(sample_rate=16000) as source:
        #recognizer.adjust_for_ambient_noise(source, duration=0.5)
        try:
            #audio = recognizer.listen(source, phrase_time_limit=10, timeout=3)
            audio = recognizer.listen(source)
            
            fs = audio.sample_rate
            audio = np.frombuffer(audio.frame_data, np.int16)
            # tokenize
            input_values = processor(audio.reshape((1,-1)), return_tensors="pt", padding="longest").input_values  # Batch size 1
            
            # retrieve logits
            logits = model(input_values[0].double()).logits
            
            # take argmax and decode
            predicted_ids = torch.argmax(logits, dim=-1)
            transcription = processor.batch_decode(predicted_ids)
            text = transcription[0]
        except WaitTimeoutError:
            text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    processor = Wav2Vec2Processor.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    recognizer = Recognizer()
    rospy.init_node('speech_recognizer')
    
    recognizer_service_param = rospy.get_param("/services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")

    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)
    rospy.spin()
