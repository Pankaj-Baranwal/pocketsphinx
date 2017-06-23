""" sss"""
import sys, select, os
import termios
import contextlib
import time

import numpy as np

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

frequency_threshold = 3
words = []
content = []
test_case = []
no_of_frames = []

def analyse_file(dic_path, kwlist_path):
    global words, test_case
    with open(dic_path) as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    with open(kwlist_path) as f:
        words = f.readlines()
    words = [x.strip()[:x.strip().rfind(' ')] for x in words]

    
    # threshold = ["/1e-1" for i in range(len(content))]
    frequency = []
    for i in range(len(words)):
        if (words[i].index(' '))
        spaces = content[i].count(' ') + 2
        if  spaces <= 3:
            frequency.append(spaces)
        else:
            frequency.append(spaces * 2)
    test_case = []
    test_case = [i for i in words]
    # for i in range(2):
    #     test_case.extend(test_case)
    np.random.shuffle(test_case)
    print ("HERE IS YOUR TRAINING SET")
    print (test_case)
    OUTPUT_FILENAME = 'test_case_audio.wav'
#     print (test_case)
#     record_audio()
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
    global no_of_frames

    # rec -c 1 -r 16000 -b 16 recording.wav
    i = 0
    print ("-----SAY THE FOLLOWING OUT LOUD AND PRESS ENTER-----")
    print (test_case[i])
    os.system('rec -q -c 1 -r 16000 -b 16 test_case_audio.wav &')
    no_of_frames.append(time.time())
    with raw_mode(sys.stdin):
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                a = sys.stdin.read(1)
                if a == '\n':
                    if i == len(test_case)-1:
                        no_of_frames.append(time.time())
                        print ("STOPPING RECORDING")
                        time.sleep(2)
                        # stop Recording
                        os.system('pkill rec')
                        print (no_of_frames)
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
    config.set_string('-featparams', os.path.join(os.path.join(modeldir, 
                    'en-us/en-us'), "feat.params"))

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
    j = 0
    missed = [0 for i in range(len(words))]
    false_alarms = [0 for i in range(len(words))]
    for i in range(len(test_case)):
        if analysis_result[j][3] < test_case[i][2] and analysis_result[i][2] > test_case[i][1]:
            if analysis_result[j][0] == test_case[i][0]:
                continue
            else:
                position = words.find(test_case[i][0])
                missed[position] = missed[position]+1
                frequency[words.index(analysis_result[i][0])] = frequency[words.index(analysis_result[i][0])] + 2
    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", "/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.kwlist")
    # kws_analysis()