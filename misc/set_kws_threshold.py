# I think last step for KWS mode would be the script to tune thresholds
# The scenario is as follows
# user defines keyword list
# the script goes through the list and creates a text test set: 
# a set of phrases, where keywords repeat in shuffled order and 
# also add [speak 2-3 random words], [make some noise]. 
# test set should be balanced. Means each keyword should be asked 3-4 times, 
# random words asked at least several times. Same for noise
# The user then speaks the requested text, The tool recognizes and prints accuracy statistics
# Sorry, bfore recognizing, it records to the temporary file
# Then uses this file in a loop decoder with differen thresholds and 
# in the end suggests an optimal set of thresholds for each key phrase
# What do you think?
import numpy as np

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

def record_audio():
    
    
if __name__ == '__main__':
    analyse_file("/home/pankaj/catkin_ws/src/pocketsphinx/demo/voice_cmd.dic")