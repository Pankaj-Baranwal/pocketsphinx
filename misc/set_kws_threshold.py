"""
Add feature to say each word individually. THen press a key for next word.
This will help compare start_frame and end_frame and decide if it is 
false positive or missed detection

Calculate frequency of incorrect places each word was found in. And frequency of correct detections.
And missed detections.
"""
import sys, select, os
import termios
import contextlib
import time

import wave
import pyaudio

import numpy as np

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

frequency_threshold = 3
words = []
content = []
test_case = []

def analyse_file(path):
    global words, test_case
    with open(path) as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    words = [x[:x.index('\t')] for x in content]
    print (words)

    
    # threshold = ["/1e-1" for i in range(len(content))]
    frequency = []
    for i in range(len(content)):
        spaces = content[i].count(' ') + 1
        if  spaces <= 3:
            frequency.append(spaces)
        else:
            frequency.append(spaces * 2)
    test_case = []
    test_case = [i for i in words]
    print (test_case)
    np.random.shuffle(test_case)
#     print (test_case)
#     record_audio()
    OUTPUT_FILENAME = 'test_case_audio.wav'
    record(OUTPUT_FILENAME)
    kws_analysis()

@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def record(OUTPUT_FILENAME):

    no_of_frames = []

    _format = pyaudio.paInt16
    _channels = 1
    _rate = 44100
    _chunk = 1024

    audio = pyaudio.PyAudio()

    _rate = int(audio.get_device_info_by_index(0)['defaultSampleRate'])

    # start Recording
    stream = audio.open(format=_format, channels=_channels,
                        rate=_rate, input=True,
                        frames_per_buffer=_chunk)
    frames = []
    i = 0
    print ("-----SAY THE FOLLOWING OUT LOUD AND PRESS ENTER-----")
    print (test_case[i])
    with raw_mode(sys.stdin):
        while True:
            if not sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                data = stream.read(_chunk)
                frames.append(data)
            else:
                a = sys.stdin.read(1)
                if a == '\n':
                    if i == len(test_case)-1:
                        print ("STOPPING RECORDING")
                        # stop Recording
                        stream.stop_stream()
                        stream.close()
                        audio.terminate()

                        _wave_file = wave.open(OUTPUT_FILENAME, 'wb')
                        _wave_file.setnchannels(_channels)
                        _wave_file.setsampwidth(audio.get_sample_size(_format))
                        _wave_file.setframerate(_rate)
                        _wave_file.writeframes(b''.join(frames))
                        _wave_file.close()
                        break
                    else:
                        no_of_frames.append(time.time())
                        i = i+1
                        print ("-----SAY THE FOLLOWING OUT LOUD AND PRESS ENTER-----")
                        print (test_case[i])



def kws_analysis():
    analysis_result = []

    modeldir = "/usr/local/share/pocketsphinx/model/"

    # Create a decoder with certain model
    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
    config.set_string('-dict', 'voice_cmd.dic')
    config.set_string('-kws', 'voice_cmd.kwlist')
    config.set_string('-dither', "no")
    config.set_string('-featparams', os.path.join(os.path.join(modeldir, 'en-us/en-us'), "feat.params"))

    stream = open(os.path.join("test_case_audio.wav"), "rb")

    # Process audio chunk by chunk. On keyphrase detected perform action and restart search
    decoder = Decoder(config)
    decoder.start_utt()
    while True:
        buf = stream.read(1024)
        if buf:
            decoder.process_raw(buf, False, False)
        else:
            break
        
        if decoder.hyp() != None:
            for seg in decoder.seg():
                analysis_result.append([seg.word, seg.prob, seg.start_frame, seg.end_frame])
            # Output: [('forward', -617, 63, 121)]
            print ("Detected keyphrase, restarting search")
            decoder.end_utt()
            decoder.start_utt()
    print (analysis_result)

def process_threshold(analysis_result, test_case):
    for i in range(len(test_case)):
        if analysis_result[i][3] < test_case[i][2] and analysis_result[i][2] > test_case[i][1]:
            if analysis_result[i][0] == test_case[i][0]:
                continue
            else:
                frequency[words.index(analysis_result[i][0])] = frequency[words.index(analysis_result[i][0])] + 2




    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic")
    # kws_analysis()