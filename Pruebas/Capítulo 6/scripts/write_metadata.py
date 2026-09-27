#!/usr/bin/env python3
"""Escribe metadata reproducible de una ejecucion experimental."""

import argparse
import datetime as dt
import json
import platform
import subprocess
from pathlib import Path


def run(*command):
    return subprocess.check_output(command, text=True).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--mission-profile", required=True)
    parser.add_argument("--drones", required=True, type=int)
    parser.add_argument("--command", required=True)
    parser.add_argument("--instrumentation", required=True)
    parser.add_argument("--duration-s", required=True, type=float)
    parser.add_argument("--configuration", required=True)
    args = parser.parse_args()

    metadata = {
        "commit": run("git", "rev-parse", "HEAD"),
        "worktree_porcelain": run("git", "status", "--porcelain"),
        "timestamp": dt.datetime.now().astimezone().isoformat(),
        "hostname": platform.node(),
        "scenario": args.scenario,
        "mission_profile": args.mission_profile,
        "drone_count": args.drones,
        "command": args.command,
        "instrumentation": args.instrumentation,
        "duration_s": args.duration_s,
        "configuration": json.loads(args.configuration),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
