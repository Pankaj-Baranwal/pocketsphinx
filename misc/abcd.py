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
words = ['BACK', 'FORWARD', 'FULL SPEED', 'HALF SPEED', 'LEFT', 'MOVE', 'RIGHT', 'STOP']
content = ['BACK\tB AE K', 'FORWARD\tF AO R W ER D', 'FULL\tF UH L', 'HALF\tHH AE F', 'LEFT\tL EH F T', 'MOVE\tM UW V', 'RIGHT\tR AY T', 'SPEED\tS P IY D', 'STOP\tS T AA P']
test_case = ['RIGHT', 'FULL SPEED', 'FORWARD', 'MOVE', 'STOP', 'LEFT', 'HALF SPEED', 'BACK']
frequency = [3, 12, 14, 14, 8, 3, 3, 8]
no_of_frames = [0, 280.32360076904297, 442.45760440826416, 611.5492105484009, 757.0955991744995, 913.1042003631592, 1071.2675094604492, 1245.9512948989868, 1397.8076934814453]
reversal = False
history_frequency = []
history_errors = []

def analyse_file(dic_path, kwlist_path):
    global frequency, reversal
    f = open('automated.kwlist', 'w')
    for i in range(len(frequency)):
        f.write(words[i] + ' /1e-' + str(frequency[i]) + '/\n')
    f.close()
    original_frequency = []
    original_frequency.extend(frequency)
    print (original_frequency)
    for i in range(10):
        kws_analysis('automated.kwlist')
    f = open('automated.kwlist', 'w')
    for i in range(len(frequency)):
        f.write(words[i] + ' /1e-' + str(frequency[i]) + '/\n')
    f.close()
    reversal = True
    frequency = []
    frequency.extend(original_frequency)
    for i in range(10):
        kws_analysis('automated.kwlist')
    print (history_errors)
    min_ = 0
    for i in range(20):
        if history_errors[i]<history_errors[min_]:
            min_ = i
    frequency = []
    frequency.extend(history_frequency[min_])
    f = open('automated.kwlist', 'w')
    for i in range(len(frequency)):
        f.write(words[i] + ' /1e-' + str(frequency[i]) + '/\n')
    f.close()
    


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
                analysis_result.append([seg.word.rstrip(), seg.start_frame, seg.end_frame])

            # Output: [('forward', -617, 63, 121)]
            # print ("Detected keyphrase, restarting search")
            decoder.end_utt()
            decoder.start_utt()
    print (analysis_result)
    process_threshold(analysis_result)

def process_threshold(analysis_result):
    global frequency, history_errors, history_frequency
    marked = []
    _prev = analysis_result[0][2]
    _next = analysis_result[2][1]
    if _prev > analysis_result[1][1]:
        marked.append(0)
    for i in range(len(analysis_result)-2):
        i = i + 1
        if analysis_result[i][1] > _prev and analysis_result[i][2] < _next:
            pass
        else:
            marked.append(i)
        _prev = analysis_result[i][2]
        _next = analysis_result[i+2][1]
    j = 0
    counter = 0
    missed = [0 for i in range(len(words))]
    false_alarms = [0 for i in range(len(words))]
    for i in range(len(test_case)-1):
        if marked.index(j) < 0:
            if analysis_result[j][2] < no_of_frames[i+1] and analysis_result[j][1] > no_of_frames[i]:
                if analysis_result[j][0] == test_case[i]:
                    j = j + 1
                    continue
                else:
                    position_original = words.index(test_case[i])
                    position_observer = words.index(analysis_result[j][0])
                    missed[position_original] = missed[position_original] + 1
                    false_alarms[position_observer] = false_alarms[position_observer] + 1
                    counter = counter + 2
                    # print ('adding missed to ' + test_case[i] + ' which is ' + words[words.index(test_case[i])])
                    # print ('adding false alarm to ' + analysis_result[j][0] + ' which is ' + words[words.index(analysis_result[j][0])])
            else:
                position_original = words.index(test_case[i])
                missed[position_original] = missed[position_original] + 1
                counter = counter + 1
                # print ('adding missed to ' + test_case[i] + ' which is ' + words[words.index(test_case[i])])
                # print (i)
                # print (str(analysis_result[j][3]) + '  ' + str(no_of_frames[i+1]) + '   ' + str(analysis_result[j][2]) + '   ' + str(no_of_frames[i]))
            else:
                print "TIME MISSED"
        else:
            
    for i in range(len(words)):
        if missed[i] > 0:
            print ('Overall missed words for ' + words[i] + str(missed[i]))
        if false_alarms[i]/2 > 0:
            print ('Overall False Alarm for ' + words[i] + str(false_alarms[i]/2))
        if missed[i] > false_alarms[i]/2:
            if not reversal:
                frequency[i] = frequency[i] + missed[i] - (false_alarms[i]/2)
            else:
                diff = missed[i] - (false_alarms[i]/2)
                if frequency[i] > diff:
                    frequency[i] = frequency[i] - missed[i] + (false_alarms[i]/2)
        elif missed[i] < false_alarms[i]/2:
            if not reversal:
                diff = (false_alarms[i]/2) - missed[i]
                if frequency[i] > diff:
                    frequency[i] = frequency[i] - (false_alarms[i]/2) + missed[i]
            else:
                frequency[i] = frequency[i] + (false_alarms[i]/2) - missed[i]
    history_frequency.append(frequency)
    history_errors.append(counter)
    print (frequency)
    f = open('automated.kwlist', 'w')
    for i in range(len(frequency)):
        f.write(words[i] + ' /1e-' + str(frequency[i]) + '/\n')
    f.close()


    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", "/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.kwlist")
    # kws_analysis()