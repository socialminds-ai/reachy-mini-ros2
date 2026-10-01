#!/usr/bin/env python3
"""Play a WAV file on the robot's speaker through /audio/out (pw_audio_play),
e.g. to check the speaker.

Usage (in a terminal set up as in README section 2.2, or in the robot's container):
    python3 tools/play_audio.py [file.wav | URL | tone]
Default: a short piano sample downloaded from SAMPLE_URL. `tone` plays a 2 s
440 Hz sine instead, without network. The WAV is sent as is (PCM, any rate and
channel count): its format goes on /audio/out/info and PipeWire converts it.
"""
import io
import math
import struct
import sys
import time
import urllib.request
import wave

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from audio_common_msgs.msg import AudioData, AudioInfo

SAMPLE_URL = 'https://www.kozco.com/tech/piano2.wav'
FORMATS = {1: 'U8', 2: 'S16LE', 3: 'S24LE', 4: 'S32LE'}  # WAV sample width -> format
CHUNK_S = 0.02


def load(source):
    """(channels, sample_rate, sample_format, pcm bytes)"""
    if source == 'tone':
        rate = 16000
        pcm = b''.join(struct.pack('<h', int(0.5 * 32767 * math.sin(2 * math.pi * 440 * i / rate)))
                       for i in range(2 * rate))
        return 1, rate, 'S16LE', pcm
    if source.startswith(('http://', 'https://')):
        print(f'downloading {source}')
        source = io.BytesIO(urllib.request.urlopen(source, timeout=30).read())
    with wave.open(source, 'rb') as w:  # raises on non-PCM WAV (e.g. float)
        return (w.getnchannels(), w.getframerate(), FORMATS[w.getsampwidth()],
                w.readframes(w.getnframes()))


channels, rate, fmt, pcm = load(sys.argv[1] if len(sys.argv) > 1 else SAMPLE_URL)
frame = channels * {'U8': 1, 'S16LE': 2, 'S24LE': 3, 'S32LE': 4}[fmt]
duration = len(pcm) / frame / rate

rclpy.init()
node = rclpy.create_node('play_audio')
# as pw_audio_play subscribes: info latched, audio best-effort
info_pub = node.create_publisher(AudioInfo, '/audio/out/info', QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
audio_pub = node.create_publisher(AudioData, '/audio/out', QoSProfile(
    depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))

deadline = time.monotonic() + 10
while audio_pub.get_subscription_count() == 0 or info_pub.get_subscription_count() == 0:
    if time.monotonic() > deadline:
        sys.exit('nobody subscribes to /audio/out: is pw_audio_play running?')
    rclpy.spin_once(node, timeout_sec=0.1)

info_pub.publish(AudioInfo(channels=channels, sample_rate=rate, sample_format=fmt,
                           bitrate=rate * frame * 8, coding_format='WAVE'))
time.sleep(0.3)  # let the info arrive before the first chunk
print(f'playing {duration:.1f} s ({channels} ch, {rate} Hz, {fmt}) on /audio/out')

# paced at real time, a little ahead: pw_audio_play queues, but a burst of
# best-effort messages could be dropped on the way
step = int(CHUNK_S * rate) * frame
start = time.monotonic()
for i, off in enumerate(range(0, len(pcm), step)):
    audio_pub.publish(AudioData(data=pcm[off:off + step]))
    ahead = start + i * CHUNK_S - 0.2 - time.monotonic()
    if ahead > 0:
        time.sleep(ahead)
time.sleep(max(0.0, start + duration - time.monotonic()) + 0.5)
print('done')
node.destroy_node()
rclpy.shutdown()
