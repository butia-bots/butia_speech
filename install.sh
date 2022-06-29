#!/bin/sh

[ $(which python3-pyaudio) ] || sudo apt install python3-pyaudio

pip install --upgrade numpy=1.22

pip install -r ./requirements.txt