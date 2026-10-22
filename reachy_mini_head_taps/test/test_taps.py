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
"""Tap detection on synthetic signals."""

from pathlib import Path

import numpy as np
from reachy_mini_head_taps.taps import detect_file, TapCounter, TapDetector, to_float

RATE = 16000


def signal(duration, taps=(), tones=(), noise_dbfs=-60.0, seed=0):
    """Background noise, decaying clicks at `taps` (s), 300 ms tones at `tones`."""
    rng = np.random.default_rng(seed)
    x = rng.normal(0, 10 ** (noise_dbfs / 20), int(duration * RATE))
    t = np.arange(int(0.03 * RATE)) / RATE
    click = 0.5 * np.exp(-t / 0.005) * rng.normal(0, 1, len(t))
    for start in taps:
        i = int(start * RATE)
        x[i:i + len(click)] += click
    t = np.arange(int(0.3 * RATE)) / RATE
    tone = 0.3 * np.sin(2 * np.pi * 220 * t)
    for start in tones:
        i = int(start * RATE)
        x[i:i + len(tone)] += tone
    return x.astype(np.float32)


def detect(x, chunk=160):
    d = TapDetector(RATE)
    return [t for i in range(0, len(x), chunk) for t in d.process(x[i:i + chunk])]


def counts(x, chunk=160):
    d, c, out = TapDetector(RATE), TapCounter(), []
    for i in range(0, len(x), chunk):
        for t in d.process(x[i:i + chunk]):
            c.add(t)
        n = c.poll((i + chunk) / RATE)
        if n:
            out.append(n)
    return out


def test_taps_found_at_their_time():
    taps = [1.0, 1.3, 1.6]
    found = detect(signal(3.0, taps))
    assert len(found) == 3
    assert np.allclose(found, taps, atol=0.01)


def test_chunk_size_does_not_matter():
    x = signal(3.0, [1.0, 1.3, 1.6])
    assert detect(x, 160) == detect(x, 37)


def test_groups_counted():
    x = signal(6.0, [0.5, 0.8, 1.1] + [3.0, 3.3, 3.6, 3.9])
    assert counts(x) == [3, 4]


def test_long_sounds_are_not_taps():
    assert detect(signal(3.0, tones=[1.0, 2.0])) == []


def test_silence_has_no_taps():
    assert detect(signal(3.0)) == []


def test_to_float_takes_first_channel():
    stereo = np.array([[16384, -1], [-16384, -1]], dtype='<i2').tobytes()
    assert np.allclose(to_float(stereo, 'S16LE', 2), [0.5, -0.5])


def test_recording_from_the_robot():
    """
    On the robot, 45 s from /audio/head/raw (16 kHz mono).

    Taps on the head: 3, 4, 3 (softer), 4 (faster); then speech, claps and
    knocks on the table, which must not make a group of 3 or 4.
    """
    path = Path(__file__).parent / 'data' / 'taps.wav'
    taps, groups = detect_file(str(path))
    assert [n for _, n in groups] == [3, 4, 3, 4]
    assert np.allclose([t for t, _ in groups], [7.93, 14.32, 19.34, 24.16], atol=0.05)
