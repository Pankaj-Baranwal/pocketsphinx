""" sss"""
import sys, select, os
import termios
import contextlib
import time
import re

import numpy as np

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

words = ['BACK', 'FORWARD', 'FULL SPEED', 'HALF SPEED', 'LEFT', 'MOVE', 'RIGHT', 'STOP']
test_case = ['MOVE', 'HALF SPEED', 'FULL SPEED', 'STOP', 'BACK', 'LEFT', 'FULL SPEED', 'RIGHT', 'FORWARD', 'STOP', 'MOVE', 'HALF SPEED', 'LEFT', 'FORWARD', 'BACK', 'RIGHT']
frequency = [33, 12, 14, 14, 8, 3, 3, 8]
no_of_frames = [0, 179.3909788131714, 313.7873888015747, 443.70648860931396, 575.8549928665161, 686.6472005844116, 800.2424001693726, 930.3613901138306, 1030.3193807601929, 1142.4673795700073, 1257.0980787277222, 1362.549877166748, 1493.0691957473755, 1609.5585823059082, 1728.8280963897705, 1843.3436870574951, 1958.2996845245361]

def analyse_file(dic_path, kwlist_path):
    global frequency
    f = open(kwlist_path, 'w')
    for i in range(len(frequency)):
        f.write(words[i] + ' /1e-' + str(frequency[i]) + '/\n')
    f.close()
    original_frequency = []
    original_frequency.extend(frequency)
    print ('ready')
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
            for j in range(len(frequency)):
                if j == position:
                    frequency[j] -= 2
                    _f.write(words[j] + ' /1e-' + str(frequency[j]) + '/\n')
                else:
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
    
    while missed[0][1] > 0:
        print ('IN MISSED WHILE')
        print (missed[0][0], str(missed[0][1]))
        position = words.index(missed[0][0])
        
        for i in range(frequency[position],50):
            _f = open(kwlist_path, 'w')
            for j in range(len(frequency)):
                if j == position:
                    frequency[j] += 1
                    _f.write(words[j] + ' /1e-' + str(frequency[j]) + '/\n')
                else:
                    _f.write(words[j] + ' /1e-' + str(frequency[j]) + '/\n')
            _f.close()
            missed_new, fa = process_threshold(kws_analysis(kwlist_path))
            print ('MISSED_NEW')
            
            missed_new.sort(key=lambda x: x[1], reverse=True)
            print (missed_new) 
            print (frequency)

            if missed[0][0] == missed_new[0][0] and missed_new[0][1] > 0:
                print ('Missed Still the same')
                print (missed_new[0][0], str(missed_new[0][1]))
                pass
            else:
                missed = []
                missed.extend(missed_new)
                print (missed[0][0], ' missed reign ended')
                break
        if missed[0][1] == 0:
            break
    print ("All missing detections now detected. New frequency: ")
    print (frequency)

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
    print ('INSIDE kws')

    stream = open("TEST_CASE_audio02.wav", "rb")

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
            print ([(seg.word, timer/320)
                               for seg in decoder.seg()])
            analysis_result.append([seg.word.rstrip(), timer/320])

            # Output: [('forward', -617, 63, 121)]
            # print ("Detected keyphrase, restarting search")
            decoder.end_utt()
            decoder.start_utt()
        timer += 1024

    print ("analysis result")
    print (analysis_result)
    return analysis_result

def process_threshold(analysis_result):
    _indices = []
    print ('INSIDE process')

    j = 1
    missed = [[words[i], 0] for i in range(len(words))]
    false_alarms = [[words[i], 0] for i in range(len(words))]
    i = 0

    for i in range(len(analysis_result)):
        _index = min(range(len(no_of_frames)), key=lambda l: abs(no_of_frames[l] - analysis_result[i][1]))
        _indices.append(_index)
        if test_case[_index-1] == analysis_result[i][0]:
            print ('DETECTED CORRECTLY', analysis_result[i][0])
        else:
            print ('FA Found', analysis_result[i][0])
            position_original = words.index(test_case[_index-1])
            position_observer = words.index(analysis_result[i][0])
            missed[position_original][1] += 1
            false_alarms[position_observer][1] += 1
    for i in range(len(test_case)):
        if i not in _indices:
            position_original = words.index(test_case[i])
            missed[position_original][1] += 1
    print ('OUTSIDE')
    return missed, false_alarms
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", "/home/pankaj/catkin_ws/src/pocketsphinx/demo/automated.kwlist")
    # kws_analysis()