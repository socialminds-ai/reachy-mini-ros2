# Copyright 2026 SocialMinds Lab, IIIA-CSIC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Tap detection on a mono audio stream, without ROS (tested on recordings).

A tap on the head reaches the microphones through the shell: a loud, sharp
onset that dies out within a few tens of milliseconds. TapDetector finds those
onsets in short frames; TapCounter groups taps that follow each other closely
and reports how many there were once the group is over.

Time is the audio's (samples received / sample rate), so the result does not
depend on how the chunks arrive.
"""

from collections import deque

import numpy as np


class TapDetector:
    """
    Finds taps: a short burst much louder than the recent background, then quiet.

    An onset is a frame `onset_ratio_db` above the median of the last
    `background_s`. The burst's peak is taken over the next `burst_s` and must
    reach `min_level_dbfs`; the following `decay_s` must then be `decay_db`
    below that peak on average. On the Reachy Mini's head microphones a tap is
    a 30-40 ms burst near full scale, 30-40 dB quieter 50 ms later; speech
    and music stay loud.
    """

    def __init__(self, sample_rate, frame_s=0.005, background_s=0.2, onset_ratio_db=20.0,
                 min_level_dbfs=-20.0, burst_s=0.05, decay_s=0.05, decay_db=20.0,
                 refractory_s=0.1):
        self.rate = sample_rate
        self.frame = max(1, int(frame_s * sample_rate))
        self.onset_ratio = 10 ** (onset_ratio_db / 10)       # on energies
        self.min_energy = 10 ** (min_level_dbfs / 10)
        self.burst_frames = max(1, round(burst_s / frame_s))
        self.decay_frames = max(1, round(decay_s / frame_s))
        self.decay_ratio = 10 ** (-decay_db / 10)
        self.refractory = refractory_s
        self.background = deque(maxlen=max(1, round(background_s / frame_s)))
        self.pending = np.zeros(0, dtype=np.float32)    # partial frame
        self.candidate = None       # an onset being followed, see _follow
        self.last_tap = -1e9
        self.frames_seen = 0

    def process(self, samples):
        """Feed float samples in [-1, 1]; return the times (s) of the taps confirmed."""
        taps = []
        buf = np.concatenate([self.pending, np.asarray(samples, dtype=np.float32)])
        n = len(buf) // self.frame
        self.pending = buf[n * self.frame:]
        for frame in buf[:n * self.frame].reshape(n, self.frame):
            energy = float(np.mean(frame * frame))
            t = self.frames_seen * self.frame / self.rate
            self.frames_seen += 1
            if self.candidate is not None:
                tap = self._follow(energy)
                if tap is not None:
                    taps.append(tap)
                continue    # the candidate's burst and tail are not background
            if (len(self.background) == self.background.maxlen
                    and energy > self.onset_ratio * float(np.median(self.background))
                    and t - self.last_tap > self.refractory):
                self.candidate = {'t': t, 'peak': energy, 'frames': 1, 'tail': 0.0}
            else:
                self.background.append(energy)
        return taps

    def _follow(self, energy):
        """Track the burst's peak, then the tail's energy; a tap if loud then quiet."""
        c = self.candidate
        c['frames'] += 1
        if c['frames'] <= self.burst_frames:
            c['peak'] = max(c['peak'], energy)
            return None
        c['tail'] += energy
        if c['frames'] < self.burst_frames + self.decay_frames:
            return None
        self.candidate = None
        tail = c['tail'] / self.decay_frames
        if c['peak'] >= self.min_energy and tail < self.decay_ratio * c['peak']:
            self.last_tap = c['t']
            return c['t']
        return None     # too faint, or still loud: speech, music, a long noise


class TapCounter:
    """Groups taps less than max_gap_s apart; reports a group's size once it is over."""

    def __init__(self, max_gap_s=0.7):
        self.max_gap = max_gap_s
        self.count = 0
        self.last = None

    def add(self, t):
        self.count += 1
        self.last = t

    def poll(self, now):
        """Return the size of the group that ended before `now`, else 0."""
        if self.count and now - self.last > self.max_gap:
            n, self.count = self.count, 0
            return n
        return 0


def to_float(data, sample_format, channels):
    """Raw PCM bytes to mono float samples in [-1, 1] (first channel)."""
    dtypes = {'S16LE': ('<i2', 32768.0), 'S32LE': ('<i4', 2147483648.0),
              'F32LE': ('<f4', 1.0)}
    if sample_format not in dtypes:
        raise ValueError(f'unsupported sample format {sample_format}')
    dtype, scale = dtypes[sample_format]
    x = np.frombuffer(bytes(data), dtype=dtype).astype(np.float32) / scale
    return x[::channels] if channels > 1 else x


def detect_file(path, chunk=160, **params):
    """Run a WAV file through TapDetector and TapCounter: (taps, [(end time, count)])."""
    import wave
    with wave.open(path, 'rb') as w:
        rate, channels = w.getframerate(), w.getnchannels()
        fmt = {2: 'S16LE', 4: 'S32LE'}[w.getsampwidth()]
        x = to_float(w.readframes(w.getnframes()), fmt, channels)
    detector, counter = TapDetector(rate, **params), TapCounter()
    taps, groups = [], []
    for i in range(0, len(x) + chunk, chunk):     # one chunk past the end: closes the last group
        for t in detector.process(x[i:i + chunk]):
            taps.append(t)
            counter.add(t)
        n = counter.poll((i + chunk) / rate)
        if n:
            groups.append((counter.last, n))
    return taps, groups


if __name__ == '__main__':
    import sys
    taps, groups = detect_file(sys.argv[1])
    print('taps:', ' '.join(f'{t:.2f}' for t in taps))
    for end, n in groups:
        print(f'{n} taps, the last at {end:.2f} s')
