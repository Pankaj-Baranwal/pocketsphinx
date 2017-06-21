import numpy as np
import os

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
    

# def record_audio():
    
    
if __name__ == '__main__':
#     if len(argv) > 1:
#     FILE_NAME = argv[1]
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic")