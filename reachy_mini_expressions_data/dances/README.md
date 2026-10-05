---
license: apache-2.0
task_categories:
  - robotics
language:
  - en
tags:
  - reachy_mini_community_moves
pretty_name: Reachy Mini Dances Library
---

# Reachy Mini Dances Library

Curated dance moves for the Reachy Mini robot, maintained by Pollen
Robotics. Each move is a JSON trajectory (head pose, antennas, body
yaw, sampled over time). Motion-only — no audio tracks in this set.

## File layout

Files live at the root of the dataset, named `<dance>.json`.

## How to use

**Python** — via the `reachy_mini` package:

```python
from reachy_mini import ReachyMini
from reachy_mini.motion.recorded_move import RecordedMoves

library = RecordedMoves("pollen-robotics/reachy-mini-dances-library")
with ReachyMini() as reachy:
    reachy.play_move(library.get("chicken_peck"))
```

**Marionette web app** — discoverable in the Community tab at
[huggingface.co/spaces/RemiFabre/marionette](https://huggingface.co/spaces/RemiFabre/marionette).

## Reuse

Keep the `reachy_mini_community_moves` tag when sharing derivatives so
the community can discover related sets.
