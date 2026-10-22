# reachy_mini_head_taps: tap on the head to wake the robot up or put it to sleep

`driver.launch.py` (in `reachy_mini`) starts it with the robot (`head_taps`
argument, default true, with the audio): tapping on the head is the easy way to
wake up a robot that starts asleep.

```sh
ros2 run reachy_mini_head_taps head_taps        # or: ros2 launch reachy_mini_head_taps head_taps.launch.py
```

| taps | what the robot does |
|---|---|
| 3 | wakes up: the driver's `/skill/wake_up`, with Pollen's `wake-mini-up` animation the first time, then without |
| 4 | goes to sleep: the driver's `/skill/go_to_sleep` |

The node follows the robot's state on the robot blackboard
(`/robot_blackboard/awake`, which the driver writes): 3 taps on a robot that is
already awake, or 4 on a robot already asleep, do nothing. While the state is
unknown (no blackboard), taps always call the skill.

Taps are counted as one group while less than `max_gap` (0.7 s) apart; the
group's size is published on `~/taps` (`std_msgs/Int32`) when it ends, whatever
it is. While the robot moves, and for `holdoff` (1 s) after, taps are ignored:
the motors would be heard as taps.

On the head microphones, a tap is a burst of 30-40 ms near full scale that is
30-40 dB quieter 50 ms later. The detector looks for an onset `onset_ratio_db`
(20 dB) above the background of the last 200 ms, a peak over the next 50 ms of
at least `min_level_dbfs` (-20 dBFS), then 50 ms `decay_db` (20 dB) below that
peak. Speech, music, claps and knocks on the table do not die out that fast,
or are not that loud: on `test/data/taps.wav` (taps, then speech, claps and
knocks) the four groups of taps are found and nothing else.

To check a recording (made with the driver repository's
`tools/record_audio.py`):

```sh
python3 -m reachy_mini_head_taps.taps my_recording.wav
```

| parameter | default | |
|---|---|---|
| `audio_topic` | `/audio/head/raw` | `audio_common_msgs/AudioData`; its format is read from `<topic>/../info` |
| `wake_taps`, `sleep_taps` | 3, 4 | |
| `wake_motion` | `wake-mini-up` | the animation of `/skill/wake_up` the first time the robot wakes up (by taps or not): any move `/skill/list_motions` lists, or empty for a plain goto to the rest pose |
| `rewake_motion` | empty | the same, the following times: by default a plain 2 s goto to the rest pose |
| `skills_namespace` | `/skill` | where the driver's skills are |
| `awake_parameter` | `/robot_blackboard/awake` | the robot's state, as `/<node>/<parameter>`; empty: not followed |
| `max_gap` | 0.7 | seconds between two taps of a group |
| `holdoff` | 1.0 | seconds deaf after a motion |
| `onset_ratio_db`, `min_level_dbfs`, `decay_db`, `refractory` | 20, -20, 20, 0.1 | the detector, see above |

The detection (`reachy_mini_head_taps/taps.py`) has no ROS dependency;
`colcon test` runs it on synthetic signals and on the recording.
