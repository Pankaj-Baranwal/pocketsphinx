""" sss"""
import sys, select, os
import termios
import contextlib
import time
import re

import numpy as np

import Levenshtein

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

frequency_threshold = 3
words = ['BACK', 'FORWARD', 'FULL SPEED', 'HALF SPEED', 'LEFT', 'MOVE', 'RIGHT', 'STOP']
content = ['BACK\tB AE K', 'FORWARD\tF AO R W ER D', 'FULL\tF UH L', 'HALF\tHH AE F', 'LEFT\tL EH F T', 'MOVE\tM UW V', 'RIGHT\tR AY T', 'SPEED\tS P IY D', 'STOP\tS T AA P']
test_case = ['HALF SPEED', 'BACK', 'BACK', 'STOP', 'RIGHT', 'LEFT', 'FULL SPEED', 'HALF SPEED', 'MOVE', 'FORWARD', 'FULL SPEED', 'STOP', 'FORWARD', 'RIGHT', 'MOVE', 'LEFT']
map_words = ['B', 'F', 'U', 'H', 'L', 'M', 'R', 'S']
frequency = [3, 12, 14, 14, 8, 3, 3, 8]
original_frequency = [3, 12, 14, 14, 8, 3, 3, 8]
no_of_frames = [0, 183.32319259643555, 319.18489933013916, 465.0690793991089, 632.1571826934814, 766.5113925933838, 907.1622848510742, 1073.1438875198364, 1232.8045845031738, 1386.5978002548218, 1541.7927980422974, 1713.9843940734863, 1860.0377798080444, 2000.3630876541138, 2148.6729860305786, 2296.087384223938, 2437.8991842269897]
reversal = False
history_frequency = []
history_errors = []

def analyse_file(dic_path, kwlist_path):
    global frequency, reversal
    f = open(kwlist_path, 'w')
    for i in range(len(frequency)):
        f.write(words[i] + ' /1e-' + str(frequency[i]) + '/\n')
    f.close()
    print (original_frequency)
    for i in range(len(words)):
        analysis_result = kws_analysis(kwlist_path)
        print ("In Loop")
        missed, false = process_result(analysis_result)
        if missed[i] + false[i] > 0:
            correct = False
            while not correct:
                if frequency[i] < original_frequency[i] and frequency[i] > 2:
                    frequency[i] = frequency[i] - 3
                    process_threshold(a, kwlist_path)
                elif frequency[i] < 2:
                    frequency[i] = original_frequency[i] + 1
                elif frequency[i] < 100:
                    frequency[i] = frequency[i] + 3
                else:
                    print frequency[i]
                    print ("NOT ABLE TO DETECT WORD")
                    break
            f = open('automated.kwlist', 'w')
            for i in range(len(frequency)):
                if frequency[i] > 50:
                    f.write(words[i] + ' /1e+' + str(frequency[i]-50) + '/\n')
                else:
                    f.write(words[i] + ' /1e-' + str(frequency[i]) + '/\n')
            f.close()
            analysis_result = kws_analysis(kwlist_path)
            missed, false = process_result(analysis_result)
            if missed[i] + false[i] > 0:
                continue
            else:
                break


    


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
    print "KWS EXECUTED"
    return analysis_result
    # process_threshold(analysis_result)

def process_result(analysis_result):
    global frequency, history_errors, history_frequency, original_frequency
    s_observed = ''
    s_original = ''
    for i in analysis_result:
        print (i)
        s_observed = s_observed+map_words[words.index(i[0])]
    print ("ORIGINAL OVER")
    for i in test_case:
        print (i)
        s_original = s_original + map_words[words.index(i)]
    print ("List converted to string")
    print (s_original)
    print (s_observed)
    print (Levenshtein.editops(s_original, s_observed))
    

    


    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic", "/home/pankaj/catkin_ws/src/pocketsphinx/demo/automated.kwlist")
    # kws_analysis()