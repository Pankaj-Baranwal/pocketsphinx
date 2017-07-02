"""Script for auto tuning keyword thresholds"""
from __future__ import print_function
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
TEST_CASE = []
FREQUENCY = []
NO_OF_FRAMES = []
OUTPUT_FILENAME = 'testing_audio.wav'

def preprocess_files(dic_path, kwlist_path):
    """
    Function to tune threshold according to result of kws mode
    """
    global WORDS, TEST_CASE, FREQUENCY

    _content = []
    with open(dic_path) as _f:
        _content = _f.readlines()
    _content = [x.strip() for x in _content]
    with open(kwlist_path) as _f:
        WORDS = _f.readlines()
    WORDS = [x.strip()[:x.strip().rfind(' ')] for x in WORDS]
    print (WORDS)

    for i, _ in enumerate(WORDS):
        init_pos = 0
        spaces = 0
        for _m in re.finditer(' ', WORDS[i]):
            indices = [j for j, s in enumerate(_content) if WORDS[i][init_pos:_m.start()]+'\t' in s]
            spaces = _content[indices[0]].count(' ') + spaces + 1
            init_pos = _m.start()+1
        indices = [j for j, s in enumerate(_content) if WORDS[i][init_pos:]+'\t' in s]
        spaces = _content[indices[0]].count(' ') + spaces + 1
        if  spaces <= 3:
            FREQUENCY.append(spaces)
        else:
            FREQUENCY.append(spaces * 2)
    TEST_CASE = ['[RANDOM]', '[RANDOM]']
    TEST_CASE.extend(WORDS)
    TEST_CASE.extend(['[RANDOM]', '[RANDOM]'])
    TEST_CASE.extend(WORDS)
    np.random.shuffle(TEST_CASE)
    print ("HERE IS YOUR TRAINING SET")
    print (TEST_CASE)

    record()
    write_frequency_to_file(kwlist_path)

    # Analysis begins
    analyse_fa(dic_path, kwlist_path)
    analyse_missed(dic_path, kwlist_path)
    
    print ("Frequency tuned to the best of the script's ability. New frequency: ")
    print (FREQUENCY)

def write_frequency_to_file(kwlist_path):
    """
    update modified frequencies in kwlist file
    """
    _f = open(kwlist_path, 'w')
    for i, val in enumerate(FREQUENCY):
        _f.write(WORDS[i] + ' /1e-' + str(val) + '/\n')
    _f.close()

@contextlib.contextmanager
def raw_mode(_file):
    """
    Function handle the button press on successful utterance of word by user
    """
    old_attrs = termios.tcgetattr(_file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(_file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(_file.fileno(), termios.TCSADRAIN, old_attrs)

def record():
    """
    Records user's speech with timestamp for each spoken word
    """
    global NO_OF_FRAMES
    # rec -c 1 -r 16000 -b 16 recording.wav
    print ("-----SAY THE FOLLOWING OUT LOUD AND PRESS ENTER-----")
    print (TEST_CASE[0])
    os.system('rec -q -c 1 -r 16000 -b 16 ' + OUTPUT_FILENAME + ' &')
    NO_OF_FRAMES.append(0)
    previous = time.time()
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

def analyse_fa(dic_path, kwlist_path):
    """
    process false alarms to tune thresholds
    """

    _missed, _fa = process_threshold(kws_analysis(dic_path, kwlist_path))
    _fa.sort(key=lambda x: x[1], reverse=True)
    position = 0

    while _fa[0][1] > 0:
        print ('Working on FA of ', _fa[0][0])
        position = WORDS.index(_fa[0][0])
        
        for _ in range(FREQUENCY[position], 1, -1):
            FREQUENCY[position] -= 2
            write_frequency_to_file(kwlist_path)
            print ('UPDATED FREQUENCY:')
            print (FREQUENCY)
            _missed, _fa_new = process_threshold(kws_analysis(dic_path, kwlist_path))
            
            _fa_new.sort(key=lambda x: x[1], reverse=True)

            if _fa[0][0] != _fa_new[0][0] or _fa_new[0][1] < 0:
                _fa = []
                _fa.extend(_fa_new)
                print (_fa[0][0], ' corrected')
                break
        if _fa[0][1] == 0:
            break
    print ("Removed many false alarms. New frequency: ")
    print (FREQUENCY)
    time.sleep(1)

def analyse_missed(dic_path, kwlist_path):
    """
    process missed detections to tune thresholds
    """
    print ('Moving on to missed detections')
    _missed, _fa = process_threshold(kws_analysis(dic_path, kwlist_path))
    _missed.sort(key=lambda x: x[1], reverse=True)
    _inner_list = [e[1] for e in _missed]

    if 0 in _inner_list:
        _smallest_index = _inner_list.index(0)-1
    else:
        _smallest_index = len(_inner_list)-1
    _ignore = []

    while _smallest_index > -1:
        print (_missed[_smallest_index][0])
        _position = WORDS.index(_missed[_smallest_index][0])
        print ('Working on Missed detection of: ', _missed[_smallest_index][0])    
        for _ in range(FREQUENCY[_position], 50):
            FREQUENCY[_position] += 1
            write_frequency_to_file(kwlist_path)
            _missed_new, _fa = process_threshold(kws_analysis(dic_path, kwlist_path))
            for _i, _value in enumerate(_fa):
                if _value[1] > 0:
                    _ignore.append(_value[0])
                    FREQUENCY[_i] -= 1
            write_frequency_to_file(kwlist_path)
            print ('UPDATED FREQUENCY:')
            print (FREQUENCY)
            
            _missed_new.sort(key=lambda x: x[1], reverse=True)
            _inner_list = [e[1] for e in _missed_new]
            _smallest_index_new = _inner_list.index(0)-1

            while _missed_new[_smallest_index_new][0] in _ignore and _smallest_index_new > -1:
                _smallest_index_new -= 1
            if _smallest_index_new < 0:
                _smallest_index = _smallest_index_new
                break
            if (_missed[_smallest_index][0] != _missed_new[_smallest_index_new][0] or 
                    _missed_new[_smallest_index_new][1] < 0):
                _smallest_index = _smallest_index_new
                _missed = []
                _missed.extend(_missed_new)
                print (_missed[_smallest_index][0], ' corrected')
                break
        if _missed[_smallest_index][1] == 0 or _smallest_index < 0:
            break

def kws_analysis(dic, kwlist):
    """
    kws analysis on user speech and updated threshold values
    """
    analysis_result = []

    modeldir = "/usr/local/share/pocketsphinx/model/"

    # Create a decoder with certain model
    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
    config.set_string('-dict', dic)
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
    """
    calculate missed detections and false alarms
    """
    _indices = []

    missed = [[WORDS[i], 0] for i in range(len(WORDS))]
    false_alarms = [[WORDS[i], 0] for i in range(len(WORDS))]
    i = 0

    for i, val in enumerate(analysis_result):
        _index = min(range(len(NO_OF_FRAMES)), key=lambda l: abs(NO_OF_FRAMES[l] - val[1]))
        _indices.append(_index)
        if TEST_CASE[_index-1] == '[RANDOM]':
            position_observer = WORDS.index(val[0])
            false_alarms[position_observer][1] += 1
            print ('FA Found', val[0], ' in place of RANDOM TEXT')
        elif TEST_CASE[_index-1] == val[0]:
            print ('DETECTED CORRECTLY', val[0])
        else:
            print ('FA Found', val[0], ' in place of ', TEST_CASE[_index-1])
            position_original = WORDS.index(TEST_CASE[_index-1])
            position_observer = WORDS.index(val[0])
            missed[position_original][1] += 1
            false_alarms[position_observer][1] += 1
    for i, val in enumerate(TEST_CASE):
        if i+1 not in _indices and val != '[RANDOM]':
            position_original = WORDS.index(val)
            missed[position_original][1] += 1
            print ('Missed ', val)
    return missed, false_alarms
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    preprocess_files("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", 
                     "/home/pankaj/catkin_ws/src/pocketsphinx/demo/automated.kwlist")
