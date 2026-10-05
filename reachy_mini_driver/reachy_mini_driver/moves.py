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
Recorded moves: loading and evaluation.

A move is a JSON file (Pollen's format: `time` + `set_target_data`, 50 Hz)
giving a head pose, antenna positions and a body yaw over time. Libraries can
be a directory of an installed package (package://<package>/<directory>,
found through the ament index), a local directory, or a HuggingFace dataset
(downloaded once, cached).
"""

import glob
import json
import os
from pathlib import Path
from typing import Dict, Optional

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from reachy_mini.motion.recorded_move import RecordedMove, RecordedMoves

# Pollen's emotions library, as installed by reachy_mini_expressions_data
DEFAULT_DATASET = 'package://reachy_mini_expressions_data/emotions'
PACKAGE_SCHEME = 'package://'


def resolve_package_dataset(dataset: str) -> str:
    """Return the directory of package://<package>/<directory>, through the ament index."""
    package, _, sub = dataset[len(PACKAGE_SCHEME):].partition('/')
    try:
        share = get_package_share_directory(package)
    except PackageNotFoundError:
        raise KeyError(f"dataset '{dataset}': package '{package}' is not installed") from None
    path = os.path.join(share, sub)
    if not os.path.isdir(path):
        raise KeyError(f"dataset '{dataset}': no directory {path}")
    return path


class MoveLibrary:
    """Loads moves by name from a dataset name or a directory, with a cache."""

    def __init__(self, default_dataset: str = DEFAULT_DATASET) -> None:
        """Create an empty library; datasets are loaded on first use."""
        self.default_dataset = default_dataset
        self._datasets: Dict[str, Dict[str, RecordedMove]] = {}

    def _load(self, dataset: str) -> Dict[str, RecordedMove]:
        if dataset.startswith(PACKAGE_SCHEME):
            dataset = resolve_package_dataset(dataset)
        if os.path.isdir(dataset):
            moves = {}
            for path in glob.glob(os.path.join(dataset, '*.json')):
                with open(path, encoding='utf-8') as f:
                    moves[Path(path).stem] = RecordedMove(json.load(f))
            return moves
        lib = RecordedMoves(dataset)
        return {name: lib.get(name) for name in lib.list_moves()}

    def get(self, move: str, dataset: Optional[str] = None) -> RecordedMove:
        """Return the move `move` from `dataset` (default library if empty)."""
        dataset = dataset or self.default_dataset
        if dataset not in self._datasets:
            self._datasets[dataset] = self._load(dataset)
        try:
            return self._datasets[dataset][move]
        except KeyError:
            raise KeyError(f"move '{move}' not found in {dataset}") from None

    def names(self, dataset: Optional[str] = None):
        """List the move names of a dataset."""
        dataset = dataset or self.default_dataset
        if dataset not in self._datasets:
            self._datasets[dataset] = self._load(dataset)
        return sorted(self._datasets[dataset])
