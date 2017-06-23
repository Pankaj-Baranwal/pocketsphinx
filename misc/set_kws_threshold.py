""" sss"""
import sys, select, os
import termios
import contextlib
import time
import re

import numpy as np

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

frequency_threshold = 3
words = []
content = []
test_case = []
frequency = []
no_of_frames = []

def analyse_file(dic_path, kwlist_path):
    global words, test_case, frequency
    with open(dic_path) as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    print ('CONTENT LIST')
    print (content)
    with open(kwlist_path) as f:
        words = f.readlines()
    words = [x.strip()[:x.strip().rfind(' ')] for x in words]
    print ('WORDS LIST')

    print (words)

    
    # threshold = ["/1e-1" for i in range(len(content))]

    for i in range(len(words)):
        init_pos = 0
        spaces = 0
        for m in re.finditer(' ', words[i]):
            indices = [j for j, s in enumerate(content) if words[i][init_pos:m.start()]+'\t' in s]
            spaces = content[indices[0]].count(' ') + spaces + 1
            init_pos = m.start()+1
        indices = [j for j, s in enumerate(content) if words[i][init_pos:]+'\t' in s]
        spaces = content[indices[0]].count(' ') + spaces + 1
        if  spaces <= 3:
            frequency.append(spaces)
        else:
            frequency.append(spaces * 2)
    print (frequency)
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
    print ("-----SAY THE FOLLOWING OUT LOUD AND PRESS ENTER-----")
    print (test_case[0])
    os.system('rec -q -c 1 -r 16000 -b 16 test_case_audio.wav &')
    previous = time.time()
    no_of_frames.append(0)
    i = 0
    with raw_mode(sys.stdin):
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                a = sys.stdin.read(1)
                if a == '\n':
                    if i == len(test_case)-1:
                        current = time.time()
                        no_of_frames.append(no_of_frames[i] + (current - previous)*100)
                        previous = current
                        print ("STOPPING RECORDING")
                        time.sleep(2)
                        # stop Recording
                        os.system('pkill rec')
                        print (no_of_frames)
                        break
                    else:
                        current = time.time()
                        no_of_frames.append(no_of_frames[i] + (current - previous)*100)
                        previous = current
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
                analysis_result.append([seg.word.rstrip(), seg.prob, seg.start_frame, seg.end_frame])
            # Output: [('forward', -617, 63, 121)]
            print ("Detected keyphrase, restarting search")
            decoder.end_utt()
            decoder.start_utt()
    print (analysis_result)
    process_threshold(analysis_result)

def process_threshold(analysis_result):
    global frequency
    j = 0
    missed = [0 for i in range(len(words))]
    false_alarms = [0 for i in range(len(words))]
    for i in range(len(test_case)-1):
        if analysis_result[j][3] < no_of_frames[i+1] and analysis_result[j][2] > no_of_frames[i]:
            if analysis_result[j][0] == test_case[i]:
                continue
            else:
                print (words)
                position_original = words.index(test_case[i])
                position_observer = words.index(analysis_result[j][0])
                missed[position_original] = missed[position_original]+1
                false_alarms[position_observer] = false_alarms[position_observer]+2
        else:
            position_original = words.index(test_case[i])
            missed[position_original] = missed[position_original]+1
    for i in range(len(words)):
        print ('Overall missed words for ' + words[i] + str(missed[i]))
        print ('Overall False Alarm for ' + words[i] + str(false_alarms[i]/2))
        if missed[i] > false_alarms[i]/2:
            frequency[i] = frequency[i] + missed[i] - (false_alarms[i]/2)
        elif missed[i] < false_alarms[i]/2:
            frequency[i] = frequency[i] + ((false_alarms[i]/2) - missed)*2    
    f = open('automated.kwlist', 'w')
    for i in range(len(frequency)):
        f.write(words[i] + ' ' + str(frequency[i]) + '\n')
    f.close()  # you can omit in most cases as the destructor will call it

    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", "/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.kwlist")
    # kws_analysis()