"""Locate vendored reBotArm_control_py and add it to sys.path."""

from __future__ import annotations

import sys
from pathlib import Path


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def sdk_candidates() -> list[Path]:
    workspace = workspace_root()
    return [
        workspace / "third_party" / "reBotArm_control_py",
        workspace / "sdk" / "reBotArm_control_py",
        Path.home() / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
    ]


def ensure_rebot_sdk_in_syspath() -> Path:
    for root in sdk_candidates():
        if (root / "reBotArm_control_py").is_dir():
            root_str = str(root)
            if root_str not in sys.path:
                sys.path.insert(0, root_str)
            return root
    candidates = "\n".join(f"  - {path}" for path in sdk_candidates())
    raise FileNotFoundError(f"Cannot find reBotArm_control_py. Clone it into one of:\n{candidates}")
