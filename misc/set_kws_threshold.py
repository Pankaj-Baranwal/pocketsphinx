"""sdff"""
import os
import numpy as np

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

frequency_threshold = 3

def analyse_file(path):
    with open(path) as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    
    threshold = ["/1e-1" for i in range(len(content))]
    frequency = []
    for i in range(len(content)):
        spaces = content[i].count(' ') + 1
        if  spaces <= 3:
            frequency.append("1e-" + str(spaces))
        else:
            frequency.append("1e-" + str((spaces * 5)))
    test_case = []
    for i in range(4):
        test_case.extend(content)
    np.random.shuffle(test_case)
#     print (test_case)
#     record_audio()
    os.system('python record_audio.py test_case_audio')
    kws_analysis(test_case)

def kws_analysis(test_case):
    analysis_result = []

    modeldir = "/usr/local/share/pocketsphinx/model/"

    # Create a decoder with certain model
    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
    config.set_string('-dict', 'voice_cmd.dic')
    config.set_string('-kws', 'voice_cmd.kwlist')

    stream = open(os.path.join("cmds7.wav"), "rb")

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
                analysis_result.append([seg.word, seg.prob])
            # Output: [('forward', -617, 63, 121)]
            print ("Detected keyphrase, restarting search")
            decoder.end_utt()
            decoder.start_utt()
    print (analysis_result)

def process_threshold(analysis_result, test_case):
    




    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic")
    # kws_analysis()