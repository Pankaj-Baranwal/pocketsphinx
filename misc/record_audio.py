from sys import argv

import wave
import pyaudio

def record(OUTPUT_FILENAME):


    _format = pyaudio.paInt16
    _channels = 1
    _rate = 44100
    _chunk = 1024

    audio = pyaudio.PyAudio()

    _rate = int(audio.get_device_info_by_index(0)['defaultSampleRate'])

    # start Recording
    stream = audio.open(format=_format, channels=_channels,
                        rate=_rate, input=True,
                        frames_per_buffer=_chunk)
    print "STARTED. PRESS CTRL+C TO STOP RECORDING"
    frames = []

    while 1:
        try:
            data = stream.read(_chunk)
            frames.append(data)
        except KeyboardInterrupt:
            print ("STOPPING RECORDING")
            # stop Recording
            stream.stop_stream()
            stream.close()
            audio.terminate()

            _wave_file = wave.open(OUTPUT_FILENAME, 'wb')
            _wave_file.setnchannels(_channels)
            _wave_file.setsampwidth(audio.get_sample_size(_format))
            _wave_file.setframerate(_rate)
            _wave_file.writeframes(b''.join(frames))
            _wave_file.close()

if __name__ == '__main__':
    OUTPUT_FILENAME = 'demo.wav'
    if len(argv) > 1:
        OUTPUT_FILENAME = argv[1] + '.wav'
    record(OUTPUT_FILENAME)
