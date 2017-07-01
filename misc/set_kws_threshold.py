"""Script for auto tuning keyword thresholds"""
import sys
import select
import os
import termios
import contextlib
import time
import re

import numpy as np

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

WORDS = []
CONTENT = []
TEST_CASE = []
FREQUENCY = []
NO_OF_FRAMES = []
OUTPUT_FILENAME = 'testing_audio.wav'

def analyse_file(dic_path, kwlist_path):

    global WORDS, TEST_CASE, FREQUENCY, OUTPUT_FILENAME
    with open(dic_path) as _f:
        CONTENT = _f.readlines()
    CONTENT = [x.strip() for x in CONTENT]
    with open(kwlist_path) as _f:
        WORDS = _f.readlines()
    WORDS = [x.strip()[:x.strip().rfind(' ')] for x in WORDS]
    print (WORDS)

    for i in range(len(WORDS)):
        init_pos = 0
        spaces = 0
        for _m in re.finditer(' ', WORDS[i]):
            indices = [j for j, s in enumerate(CONTENT) if WORDS[i][init_pos:_m.start()]+'\t' in s]
            spaces = CONTENT[indices[0]].count(' ') + spaces + 1
            init_pos = _m.start()+1
        indices = [j for j, s in enumerate(CONTENT) if WORDS[i][init_pos:]+'\t' in s]
        spaces = CONTENT[indices[0]].count(' ') + spaces + 1
        if  spaces <= 3:
            FREQUENCY.append(spaces)
        else:
            FREQUENCY.append(spaces * 2)
    TEST_CASE = []
    TEST_CASE.extend(WORDS)
    TEST_CASE.extend(WORDS)
    np.random.shuffle(TEST_CASE)
    print ("HERE IS YOUR TRAINING SET")
    print (TEST_CASE)

    record(OUTPUT_FILENAME)
    _f = open(kwlist_path, 'w')
    for i in range(len(FREQUENCY)):
        _f.write(WORDS[i] + ' /1e-' + str(FREQUENCY[i]) + '/\n')
    _f.close()

    # Analysis begins

    missed, fa = process_threshold(kws_analysis(kwlist_path))

    fa.sort(key=lambda x: x[1], reverse=True)
    
    position = 0

    while fa[0][1] > 0:
        print ('Working on FA of ', fa[0][0])
        position = WORDS.index(fa[0][0])
        print (i)
        
        for i in range(FREQUENCY[position],0,-1):
            print (i)
            _f = open(kwlist_path, 'w')
            FREQUENCY[position] -= 2
            for j in range(len(FREQUENCY)):
                _f.write(WORDS[j] + ' /1e-' + str(FREQUENCY[j]) + '/\n')
            _f.close()
            print ('UPDATED FREQUENCY:')
            print (FREQUENCY)
            missed, fa_new = process_threshold(kws_analysis(kwlist_path))
            
            fa_new.sort(key=lambda x: x[1], reverse=True)

            if fa[0][0] != fa_new[0][0] or fa_new[0][1] < 0:
                fa = []
                fa.extend(fa_new)
                print (fa[0][0], ' corrected')
                break
        if fa[0][1] == 0:
            break


    print ("All false alarms removed. New frequency: ")
    print (FREQUENCY)
    time.sleep(1)
    print ('Moving on to missed detections')


    missed, fa = process_threshold(kws_analysis(kwlist_path))

    missed.sort(key=lambda x: x[1], reverse=True)

    inner_list = [e[1] for e in missed]
    if 0 in inner_list:
        smallest_index = inner_list.index(0)-1
    else:
        smallest_index = len(inner_list)-1
    ignore = []

    while smallest_index > -1:
        print (missed[smallest_index][0])
        position = WORDS.index(missed[smallest_index][0])
        print ('Working on Missed detection of: ', missed[smallest_index][0])
        
        for i in range(FREQUENCY[position],50):
            _f = open(kwlist_path, 'w')
            FREQUENCY[position] += 1
            for j in range(len(FREQUENCY)):
                _f.write(WORDS[j] + ' /1e-' + str(FREQUENCY[j]) + '/\n')
            _f.close()
            missed_new, fa = process_threshold(kws_analysis(kwlist_path))
            for _i, _value in enumerate(fa):
                if _value[1] > 0:
                    ignore.append(_value[0])
                    FREQUENCY[_i] -= 1
            _f = open(kwlist_path, 'w')
            for j in range(len(FREQUENCY)):
                _f.write(WORDS[j] + ' /1e-' + str(FREQUENCY[j]) + '/\n')
            _f.close()
            print ('UPDATED FREQUENCY:')
            print (FREQUENCY)
            
            missed_new.sort(key=lambda x: x[1], reverse=True)

            inner_list = [e[1] for e in missed_new]


            smallest_index_new = inner_list.index(0)-1

            while missed_new[smallest_index_new][0] in ignore and smallest_index_new > -1:
                smallest_index_new -= 1
            if smallest_index_new < 0:
                smallest_index = smallest_index_new
                break

            if missed[smallest_index][0] != missed_new[smallest_index_new][0] or missed_new[smallest_index_new][1] <0:
                smallest_index = smallest_index_new
                missed = []
                missed.extend(missed_new)
                print (missed[smallest_index][0], ' corrected')
                break
        if missed[smallest_index][1] == 0 or smallest_index < 0:
            break
    
    print ("Frequency tuned to the best of the script's ability. New frequency: ")
    print (FREQUENCY)

@contextlib.contextmanager
def raw_mode(file):
    """complementing the button click to proceed feature"""
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def record(OUTPUT_FILENAME):
    global NO_OF_FRAMES
    # rec -c 1 -r 16000 -b 16 recording.wav
    print ("-----SAY THE FOLLOWING OUT LOUD AND PRESS ENTER-----")
    print (TEST_CASE[0])
    os.system('rec -q -c 1 -r 16000 -b 16 ' + OUTPUT_FILENAME + ' &')
    previous = time.time()
    NO_OF_FRAMES.append(0)
    i = 0
    with raw_mode(sys.stdin):
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                _a = sys.stdin.read(1)
                if _a == '\n':
                    if i == len(TEST_CASE)-1:
                        current = time.time()
                        NO_OF_FRAMES.append(NO_OF_FRAMES[i] + (current - previous)*100)
                        previous = current
                        print ("STOPPING RECORDING")
                        time.sleep(2)
                        # stop Recording
                        os.system('pkill rec')
                        print (NO_OF_FRAMES)
                        break
                    else:
                        current = time.time()
                        NO_OF_FRAMES.append(NO_OF_FRAMES[i] + (current - previous)*100)
                        previous = current
                        i = i+1
                        print ("-----SAY THE FOLLOWING OUT LOUD AND PRESS ENTER-----")
                        print (TEST_CASE[i])



def kws_analysis(kwlist):
    analysis_result = []

    modeldir = "/usr/local/share/pocketsphinx/model/"

    # Create a decoder with certain model
    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
    config.set_string('-dict', 'voice_cmd.dic')
    config.set_string('-kws', kwlist)
    config.set_string('-dither', "no")
    config.set_string('-logfn', '/dev/null')
    config.set_string('-featparams', os.path.join(os.path.join(modeldir, 'en-us/en-us'), "feat.params"))

    stream = open(OUTPUT_FILENAME, "rb")

    # Process audio chunk by chunk. On keyphrase detected perform action and restart search
    decoder = Decoder(config)
    decoder.start_utt()
    timer = 0
    while True:
        buf = stream.read(1024)
        if buf:
            decoder.process_raw(buf, False, False)
        else:
            break
        if decoder.hyp() != None:
            for seg in decoder.seg():
                pass
            analysis_result.append([seg.word.rstrip(), timer/320])

            decoder.end_utt()
            decoder.start_utt()
        timer += 1024

    # print ("Result on kws analysis:")
    # print (analysis_result)
    return analysis_result

def process_threshold(analysis_result):
    _indices = []

    j = 1
    missed = [[WORDS[i], 0] for i in range(len(WORDS))]
    false_alarms = [[WORDS[i], 0] for i in range(len(WORDS))]
    i = 0

    for i in range(len(analysis_result)):
        _index = min(range(len(NO_OF_FRAMES)), key=lambda l: abs(NO_OF_FRAMES[l] - analysis_result[i][1]))
        _indices.append(_index)
        if TEST_CASE[_index-1] == analysis_result[i][0]:
            print ('DETECTED CORRECTLY', analysis_result[i][0])
        else:
            print ('FA Found', analysis_result[i][0], ' in place of ', TEST_CASE[_index-1])
            position_original = WORDS.index(TEST_CASE[_index-1])
            position_observer = WORDS.index(analysis_result[i][0])
            missed[position_original][1] += 1
            false_alarms[position_observer][1] += 1
    for i in range(len(TEST_CASE)):
        if i+1 not in _indices:
            position_original = WORDS.index(TEST_CASE[i])
            missed[position_original][1] += 1
            print ('Missed ', TEST_CASE[i])
    return missed, false_alarms
    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", "/home/pankaj/catkin_ws/src/pocketsphinx/demo/automated.kwlist")
    # kws_analysis()