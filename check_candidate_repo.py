"""Search a candidate policy repository before any Piper execution.

The report lists file clues for arm count, state and action size, action
meaning, normalization, cameras, and an existing Piper driver. A clue is not
a conclusion. Do not enable joints from this report.
"""

import argparse
import re
from pathlib import Path

SKIP_DIRS = {".git", ".venv", "venv", "__pycache__", "node_modules", "dist", "build"}
TEXT_SUFFIXES = {".py", ".md", ".yml", ".yaml", ".json", ".toml", ".txt", ".cfg", ".sh"}
MAX_BYTES = 512_000
MAX_HITS = 8

CHECKS = [
    (
        "Config: one arm or two, and the state and action dimensions",
        [
            r"bimanual|dual[_\- ]?arm|bi[_\- ]?arm|two[_\- ]?arm|left_arm|right_arm|num_arms",
            r"single[_\- ]?arm|one[_\- ]?arm",
            r"action_dim|state_dim|OBS_STATE_DIM|ACTION_DIM|action_horizon|proprio",
        ],
    ),
    (
        "Action: joint angles or end-effector pose, absolute or delta, and the units",
        [
            r"joint[_\- ]?(position|angle|delta)|eef|end[_\- ]?effector|tcp|cartesian|pose",
            r"\bdelta\b|absolute|incremental|relative",
            r"radian|degree|millimeter|\bmm\b|\bmeter",
        ],
    ),
    (
        "Preprocessing and postprocessing: normalization stats or processor files used in training",
        [
            r"norm_stats|dataset_stats|quantile|unnormal|denormal|processor|normalizer",
        ],
    ),
    (
        "Cameras: required views, field names, and resolution",
        [
            r"observation/image|wrist_image|exterior|camera_names|image_keys",
            r"\b224\b|\b256\b|resize_with_pad|resolution",
        ],
    ),
    (
        "Deployment: a Piper driver, gripper mapping, and control rate",
        [
            r"\bpiper\b|agilex|0x159|0x155",
            r"gripper",
            r"control_freq|control_frequency|fps|hz|action_dt|replan",
        ],
    ),
]


def iter_text_files(root):
    for path in root.rglob("*"):
        if path.name == "check_candidate_repo.py":
            continue
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        if path.stat().st_size > MAX_BYTES:
            continue
        yield path


def search(root):
    compiled = [(title, [re.compile(pattern, re.IGNORECASE) for pattern in patterns]) for title, patterns in CHECKS]
    found = {title: [] for title, _ in CHECKS}
    names = []
    for path in iter_text_files(root):
        relative = path.relative_to(root).as_posix()
        lowered = relative.lower()
        if any(token in lowered for token in ("norm_stat", "dataset_stat", "processor", "quantile")):
            names.append(relative)
        try:
            lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
        except OSError:
            continue
        for title, patterns in compiled:
            if len(found[title]) >= MAX_HITS:
                continue
            for number, line in enumerate(lines, start=1):
                if any(pattern.search(line) for pattern in patterns):
                    snippet = " ".join(line.strip().split())
                    if len(snippet) > 180:
                        snippet = snippet[:177] + "..."
                    found[title].append(f"{relative}:{number}: {snippet}")
                    if len(found[title]) >= MAX_HITS:
                        break
    return found, names[:MAX_HITS]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", help="path to the candidate repository")
    args = parser.parse_args()
    root = Path(args.repo).resolve()
    if not root.is_dir():
        raise SystemExit(f"not a directory: {root}")
    found, names = search(root)
    print(f"repository={root}")
    print("These hits only show that the words appear in the repository. Open the files to confirm units and whether values are absolute or deltas. Do not enable joints or execute actions until that is confirmed.")
    for title, _ in CHECKS:
        print(f"\n## {title}")
        hits = found[title]
        if hits:
            print("\n".join(hits))
        else:
            print("No hits.")
    print("\n## Possible stats or processor files")
    print("\n".join(names) if names else "No filename hits.")


if __name__ == "__main__":
    main()
