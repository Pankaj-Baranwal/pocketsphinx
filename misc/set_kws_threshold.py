

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
OUTPUT_FILENAME = ''

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
    OUTPUT_FILENAME = 'testing_audio.wav'

    record(OUTPUT_FILENAME)
    _f = open(kwlist_path, 'w')
    for i in range(len(FREQUENCY)):
        _f.write(WORDS[i] + ' /1e-' + str(FREQUENCY[i]) + '/\n')
    _f.close()

    # Analysis begins
    
    missed, fa = process_threshold(kws_analysis(kwlist_path))
    print ('FA')

    fa.sort(key=lambda x: x[1], reverse=True)
    print (fa)

    while fa[0][1] > 0:
        print ('IN WHILE')
        print (fa[0][0], str(fa[0][1]))
        position = words.index(fa[0][0])
        
        for i in range(frequency[position],0,-1):
            _f = open(kwlist_path, 'w')
            frequency[position] -= 2
            for j in range(len(frequency)):
                _f.write(words[j] + ' /1e-' + str(frequency[j]) + '/\n')
            _f.close()
            missed, fa_new = process_threshold(kws_analysis(kwlist_path))
            print ('FA_NEW')
            
            fa_new.sort(key=lambda x: x[1], reverse=True)
            print (fa_new) 
            print (frequency)

            if fa[0][0] == fa_new[0][0] and fa_new[0][1] > 0:
                print ('Still the same')
                print (fa_new[0][0], str(fa_new[0][1]))
                pass
            else:
                fa = []
                fa.extend(fa_new)
                print (fa[0][0], ' reign ended')
                break
        if fa[0][1] == 0:
            break
    print ("All false alarms removed. New frequency: ")
    print (frequency)

    missed, fa = process_threshold(kws_analysis(kwlist_path))

    print ('MISSED')

    missed.sort(key=lambda x: x[1], reverse=True)
    print (missed)

    inner_list = [e[1] for e in missed]
    smallest_index = inner_list.index(0)-1
    ignore = []

    while smallest_index > -1:
        print ('IN MISSED WHILE')
        print (missed[smallest_index][0], str(missed[smallest_index][1]))
        position = words.index(missed[smallest_index][0])
        
        for i in range(frequency[position],50):
            _f = open(kwlist_path, 'w')
            frequency[position] += 1
            for j in range(len(frequency)):
                _f.write(words[j] + ' /1e-' + str(frequency[j]) + '/\n')
            _f.close()
            missed_new, fa = process_threshold(kws_analysis(kwlist_path))
            for _i, _value in enumerate(fa):
                if _value[1] > 0:
                    _f = open(kwlist_path, 'w')
                    ignore.append(_value[0])
                    frequency[_i] -= 1
                    for j in range(len(frequency)):
                        _f.write(words[j] + ' /1e-' + str(frequency[j]) + '/\n')
                    _f.close()

            print ('MISSED_NEW')
            
            missed_new.sort(key=lambda x: x[1], reverse=True)

            inner_list = [e[1] for e in missed_new]


            smallest_index_new = inner_list.index(0)-1

            while missed_new[smallest_index_new][0] in ignore and smallest_index_new > -1:
                smallest_index_new -= 1
            if smallest_index_new < 0:
                smallest_index = smallest_index_new
                print ('ITS OVER!')
                break
            # smallest_index_new = missed.index(0)-1
            print (missed_new)
            print (frequency)

            if missed[smallest_index][0] == missed_new[smallest_index_new][0] and missed_new[smallest_index_new][1] > -1:
                print ('Missed Still the same')
                print (missed_new[smallest_index_new][0], str(missed_new[smallest_index_new][1]))
            else:
                smallest_index = smallest_index_new
                missed = []
                missed.extend(missed_new)
                print (missed[smallest_index][0], ' missed reign ended')
                break
        if missed[smallest_index][1] == 0 or smallest_index < 0:
            break
    print ("All missing detections now detected. New frequency: ")
    print (frequency)    

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
    """
    keyword spotting mode similar to the node
    """
    analysis_result = []

    modeldir = "/usr/local/share/pocketsphinx/model/"

    # Create a decoder with certain model
    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
    config.set_string('-dict', 'voice_cmd.dic')
    config.set_string('-kws', kwlist)
    config.set_string('-dither', "no")
    config.set_string('-logfn', '/dev/null')
    config.set_string('-featparams', os.path.join(os.path.join(modeldir, 
                    'en-us/en-us'), "feat.params"))

    stream = open(os.path.join(OUTPUT_FILENAME), "rb")

    # Process audio chunk by chunk. On keyphrase detected perform action and restart search
    decoder = Decoder(config)
    decoder.start_utt()
    timer = 0
    while True:
        buf = stream.read(1024)
        timer += 1024
        if buf:
            decoder.process_raw(buf, False, False)
        else:
            break
        
        if decoder.hyp() != None:
            # print (timer)
            for seg in decoder.seg():
                analysis_result.append([seg.word.rstrip(), timer])
            # print ("Detected keyphrase, restarting search")
            # print (seg.word.rstrip() + ' ' + str(seg.start_frame) + ' ' + str(seg.end_frame))
            decoder.end_utt()
            decoder.start_utt()
    print (analysis_result)
    process_threshold(analysis_result, kwlist)

def process_threshold(analysis_result, kwlist):
    global FREQUENCY, HISTORY_ERRORS, HISTORY_FREQUENCY
    j = 0
    counter = 0
    missed = [0 for i in range(len(WORDS))]
    false_alarms = [0 for i in range(len(WORDS))]
    for i in range(len(TEST_CASE)-1):
        if analysis_result[j][2] < NO_OF_FRAMES[i+1] and analysis_result[j][1] > NO_OF_FRAMES[i]:
            if analysis_result[j][0] == TEST_CASE[i]:
                j = j + 1
                continue
            else:
                position_original = WORDS.index(TEST_CASE[i])
                position_observer = WORDS.index(analysis_result[j][0])
                missed[position_original] = missed[position_original]+1
                false_alarms[position_observer] = false_alarms[position_observer]+2
                counter = counter + 2
                # print ('adding missed to ' + TEST_CASE[i] + ' which is ' + WORDS[WORDS.index(TEST_CASE[i])])
                # print ('adding false alarm to ' + analysis_result[j][0] + ' which is ' + WORDS[WORDS.index(analysis_result[j][0])])
        else:
            position_original = WORDS.index(TEST_CASE[i])
            missed[position_original] = missed[position_original]+1
            counter = counter + 1
            # print ('adding missed to ' + TEST_CASE[i] + ' which is ' + WORDS[WORDS.index(TEST_CASE[i])]
    for i in range(len(WORDS)):
        if missed[i] > 0:
            print ('Overall missed WORDS for ' + WORDS[i] + str(missed[i]))
        if false_alarms[i]/2 > 0:
            print ('Overall False Alarm for ' + WORDS[i] + str(false_alarms[i]/2))
        if missed[i] > false_alarms[i]/2:
            if not REVERSAL:
                FREQUENCY[i] = FREQUENCY[i] + missed[i] - (false_alarms[i]/2)
            else:
                diff = missed[i] - (false_alarms[i]/2)
                if FREQUENCY[i] > diff:
                    FREQUENCY[i] = FREQUENCY[i] - missed[i] + (false_alarms[i]/2)
        elif missed[i] < false_alarms[i]/2:
            if not REVERSAL:
                diff = (false_alarms[i]/2) - missed[i]
                if FREQUENCY[i] > diff:
                    FREQUENCY[i] = FREQUENCY[i] - (false_alarms[i]/2) + missed[i]
            else:
                FREQUENCY[i] = FREQUENCY[i] + (false_alarms[i]/2) - missed[i]
    HISTORY_FREQUENCY.append(FREQUENCY)
    HISTORY_ERRORS.append(counter)
    print (FREQUENCY)
    _f = open(kwlist, 'w')
    for i in range(len(FREQUENCY)):
        _f.write(WORDS[i] + ' /1e-' + str(FREQUENCY[i]) + '/\n')
    _f.close()
    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", "/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.kwlist")
    # kws_analysis()