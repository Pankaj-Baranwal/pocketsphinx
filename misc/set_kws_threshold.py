"""
training_set
['FORWARD', 'LEFT', 'HALF SPEED', 'BACK', 'BACK', 'LEFT', 'FORWARD', 'MOVE', 'STOP', 'FULL SPEED', 'RIGHT', 'HALF SPEED', 'MOVE', (14)'STOP', 'FULL SPEED', 'RIGHT']
time
[0, 192.20919609069824, 318.2703971862793, 469.4032907485962, 588.911509513855, 706.5659046173096, 831.551194190979, 977.9011964797974, 1116.448712348938, 1258.5033893585205, 1394.4540977478027, 1534.2642068862915, 1674.9691009521484, (13)1807.6836109161377, 1995.4078912734985, 2175.194811820984, 2302.3014068603516]
seg start, end
[['STOP', 101, 127], ['BACK', 1927, 1977], ['FORWARD', 1858, 1914], ['FORWARD', 1785, 1844], ['BACK', 1938, 1986], ['STOP', 2312, 2357]]

"""

"""
training_set
['MOVE', 'HALF SPEED', 'FULL SPEED', 'STOP', 'BACK', 'LEFT', 'FULL SPEED', 'RIGHT', 'FORWARD', 'STOP', 'MOVE', 'HALF SPEED', 'LEFT', 'FORWARD', 'BACK', 'RIGHT']
time
[0, 179.3909788131714, 313.7873888015747, 443.70648860931396, 575.8549928665161, 686.6472005844116, 800.2424001693726, 930.3613901138306, 1030.3193807601929, 1142.4673795700073, 1257.0980787277222, 1362.549877166748, 1493.0691957473755, 1609.5585823059082, 1728.8280963897705, 1843.3436870574951, 1958.2996845245361]
seg start, end
53248
LEFT 137 152
91136
LEFT 254 267
177152
STOP 505 539
221184
BACK 630 675
250880
LEFT 739 768
368640
FORWARD 1306 1359
396288
STOP 1196 1223
443392
LEFT 1354 1369
468992
LEFT 1429 1450
507904
LEFT 1552 1571
556032
FORWARD 1669 1721
579584
BACK 1785 1798
624640
LEFT 1924 1938


"""

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
# KWLIST_FILENAME = 'automated.kwlist'
REVERSAL = False
HISTORY_FREQUENCY = []
HISTORY_ERRORS = []
OUTPUT_FILENAME = ''

def analyse_file(dic_path, kwlist_path):
    """
    create new kwlist, and loop over test cases for autotuning
    """
    global WORDS, TEST_CASE, FREQUENCY, REVERSAL, OUTPUT_FILENAME
    with open(dic_path) as _f:
        CONTENT = _f.readlines()
    CONTENT = [x.strip() for x in CONTENT]
    with open(kwlist_path) as _f:
        WORDS = _f.readlines()
    WORDS = [x.strip()[:x.strip().rfind(' ')] for x in WORDS]
    print (WORDS)

    
    # threshold = ["/1e-1" for i in range(len(CONTENT))]

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
    OUTPUT_FILENAME = 'TEST_CASE_audio02.wav'
#     print (TEST_CASE)
#     record_audio()
    record(OUTPUT_FILENAME)
    _f = open(kwlist_path, 'w')
    for i in range(len(FREQUENCY)):
        _f.write(WORDS[i] + ' /1e-' + str(FREQUENCY[i]) + '/\n')
    _f.close()
    ORIGINAL_FREQUENCY = []
    ORIGINAL_FREQUENCY.extend(FREQUENCY)
    print (ORIGINAL_FREQUENCY)
    for i in range(10):
        kws_analysis('automated.kwlist')
    _f = open(kwlist_path, 'w')
    for i in range(len(FREQUENCY)):
        _f.write(WORDS[i] + ' /1e-' + str(FREQUENCY[i]) + '/\n')
    _f.close()
    REVERSAL = True
    FREQUENCY = []
    FREQUENCY.extend(ORIGINAL_FREQUENCY)
    for i in range(10):
        kws_analysis(kwlist_path)
    print (HISTORY_ERRORS)
    min_ = 0
    for i in range(20):
        if HISTORY_ERRORS[i]<HISTORY_ERRORS[min_]:
            min_ = i
    FREQUENCY = []
    FREQUENCY.extend(HISTORY_FREQUENCY[min_])
    _f = open(kwlist_path, 'w')
    for i in range(len(FREQUENCY)):
        _f.write(WORDS[i] + ' /1e-' + str(FREQUENCY[i]) + '/\n')
    _f.close()

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