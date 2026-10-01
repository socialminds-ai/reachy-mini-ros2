#!/usr/bin/env python3
"""Record an audio_common_msgs/AudioData topic to a WAV file, e.g. to check the
microphones' quality.

Usage (in a terminal set up as in README section 2.2, or in the robot's container):
    python3 tools/record_audio.py [topic] [out.wav] [seconds]
Defaults: /audio/head/raw, head.wav, 10 s. The format (channels, sample rate,
S16LE) is read from the matching .../info topic.
"""
import sys
import wave

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from audio_common_msgs.msg import AudioData, AudioInfo

topic = sys.argv[1] if len(sys.argv) > 1 else '/audio/head/raw'
out = sys.argv[2] if len(sys.argv) > 2 else 'head.wav'
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
info_topic = topic.rsplit('/', 1)[0] + '/info'

rclpy.init()
node = rclpy.create_node('record_wav')
info = None
chunks = []


def on_info(msg):
    global info
    info = msg


def on_audio(msg):
    if info is not None:
        chunks.append(bytes(msg.data))


# /info may be latched (transient local) or not: subscribe both ways
node.create_subscription(AudioInfo, info_topic, on_info, 10)
node.create_subscription(
    AudioInfo, info_topic, on_info,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(AudioData, topic, on_audio, 100)

while rclpy.ok() and info is None:
    rclpy.spin_once(node, timeout_sec=0.1)
if info.sample_format != 'S16LE':
    sys.exit(f'unsupported sample format {info.sample_format}')
print(f'recording {seconds}s of {topic} '
      f'({info.channels} ch, {info.sample_rate} Hz, {info.sample_format}) to {out}')

needed = int(seconds * info.sample_rate * info.channels * 2)
while rclpy.ok() and sum(map(len, chunks)) < needed:
    rclpy.spin_once(node, timeout_sec=0.1)

with wave.open(out, 'wb') as w:
    w.setnchannels(info.channels)
    w.setsampwidth(2)
    w.setframerate(info.sample_rate)
    w.writeframes(b''.join(chunks)[:needed])
print('done')
node.destroy_node()
rclpy.shutdown()
