# reachy_mini_expressions_data

Pollen Robotics' expressive moves and sounds for the Reachy Mini, as a ROS package:
installed with the rest of the stack, versioned with it, and usable without network
access or a HuggingFace cache. Data only, no code.

| directory | content | source |
|---|---|---|
| `emotions/` | 85 recorded moves (`<name>.json`) and their sounds (`<name>.ogg`, mono Opus, 16 kHz; `waiting` has none), `metadata.jsonl` (a description of each move) | [reachy-mini-emotions-library](https://huggingface.co/datasets/pollen-robotics/reachy-mini-emotions-library) @ `873ae49` |
| `dances/` | 19 recorded dance moves (`<name>.json`), no sound | [reachy-mini-dances-library](https://huggingface.co/datasets/pollen-robotics/reachy-mini-dances-library) @ `3564295` |
| `sounds/` | the SDK's sound files: `wake_up`, `go_sleep`, `dance1`, `confused1`, `impatient1`, `count` (WAV, stereo, 44.1 kHz) | [reachy-mini SDK 1.10.0](https://pypi.org/project/reachy-mini/1.10.0/), `reachy_mini/assets/` |

The moves are the datasets' JSON files: head pose, antennas and body yaw sampled at
50 Hz. Each dataset keeps its card (`README.md`).

## Use

Everything is installed under the package's share directory; find it through the ament
index, never with a hard-coded path. The driver takes
`package://reachy_mini_expressions_data/<directory>` as a move library, and uses these by
default:

```yaml
# reachy_mini/config/driver.yaml
dataset: package://reachy_mini_expressions_data/emotions
extra_datasets: ['package://reachy_mini_expressions_data/dances']
```

From code: `ament_index_cpp::get_package_share_path("reachy_mini_expressions_data")`
(C++) or `ament_index_python.packages.get_package_share_directory(...)` (Python), then
`emotions/`, `dances/` or `sounds/`.

## Licence

Apache-2.0, as published by Pollen Robotics; see `LICENSE`, and `NOTICE` for the
attribution and the exact sources. The files are unmodified.

## Updating

Pin the new revisions, then copy the files over and update the revisions in `NOTICE`
and above:

```bash
python3 - <<'PY'
from huggingface_hub import snapshot_download
for name, sub in (("reachy-mini-emotions-library", "emotions"),
                  ("reachy-mini-dances-library", "dances")):
    snapshot_download(f"pollen-robotics/{name}", repo_type="dataset",
                      local_dir=sub, ignore_patterns=[".gitattributes"])
PY
rm -rf emotions/.cache dances/.cache   # huggingface_hub's bookkeeping
```

The `sounds/` come from the installed SDK: `python3 -c "import reachy_mini, os;
print(os.path.dirname(reachy_mini.__file__))"`, then `assets/*.wav`.
