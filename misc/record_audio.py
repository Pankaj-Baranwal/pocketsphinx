# Taken from https://stackoverflow.com/a/6743593
from sys import byteorder, argv
from array import array
from struct import pack

import wave
import pyaudio

THRESHOLD = 500
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
RATE = 16000

def is_silent(snd_data):
    "Returns 'True' if below the 'silent' threshold"
    return max(snd_data) < THRESHOLD

def normalize(snd_data):
    "Average the volume out"
    MAXIMUM = 16384
    times = float(MAXIMUM)/max(abs(i) for i in snd_data)

    _r = array('h')
    for i in snd_data:
        _r.append(int(i*times))
    return _r

def trim(snd_data):
    "Trim the blank spots at the start and end"
    def _trim(snd_data):
        snd_started = False
        _r = array('h')

        for i in snd_data:
            if not snd_started and abs(i) > THRESHOLD:
                snd_started = True
                _r.append(i)

            elif snd_started:
                _r.append(i)
        return _r

    # Trim to the left
    snd_data = _trim(snd_data)

    # Trim to the right
    snd_data.reverse()
    snd_data = _trim(snd_data)
    snd_data.reverse()
    return snd_data

def add_silence(snd_data, seconds):
    "Add silence to the start and end of 'snd_data' of length 'seconds' (float)"
    _r = array('h', [0 for i in xrange(int(seconds*RATE))])
    _r.extend(snd_data)
    _r.extend([0 for i in xrange(int(seconds*RATE))])
    return _r

def record(path):
    """
    Record a word or words from the microphone and 
    return the data as an array of signed shorts.

    Normalizes the audio, trims silence from the 
    start and end, and pads with 0.5 seconds of 
    blank sound to make sure VLC et al can play 
    it without getting chopped off.
    """
    _p = pyaudio.PyAudio()
    stream = _p.open(format=FORMAT, channels=1, rate=RATE, input=True, 
                     output=True, frames_per_buffer=CHUNK_SIZE)

    num_silent = 0
    snd_started = False

    _r = array('h')

    while 1:
        try:
            # little endian, signed short
            snd_data = array('h', stream.read(CHUNK_SIZE))
            if byteorder == 'big':
                snd_data.byteswap()
            _r.extend(snd_data)

            silent = is_silent(snd_data)

            if silent and snd_started:
                num_silent += 1
                print "SILENCE"
            elif not silent and not snd_started:
                snd_started = True
                print "STARTED. Press Ctrl + C to STOP."
            else:
                print ""
        except KeyboardInterrupt:
            print "RECORDING STOPPED"

            sample_width = _p.get_sample_size(FORMAT)
            stream.stop_stream()
            stream.close()
            _p.terminate()

            _r = normalize(_r)
            _r = trim(_r)
            _r = add_silence(_r, 0.5)
            _r = pack('<' + ('h'*len(_r)), *_r)

            _wf = wave.open(path, 'wb')
            _wf.setnchannels(1)
            _wf.setsampwidth(sample_width)
            _wf.setframerate(RATE)
            _wf.writeframes(_r)
            _wf.close()

if __name__ == '__main__':
    print"please speak a word into the microphone"
    NAME = 'demo.wav'
    if len(argv) > 1:
        NAME = argv[1]
    record(NAME+'.wav')
    print "done - result written to " + NAME + ".wav"
