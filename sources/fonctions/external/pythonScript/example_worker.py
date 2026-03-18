#!/usr/bin/env python3
"""
Exemple de script Python utilisable depuis C++.

Modes :
1) one-shot par arguments CLI
   python3 -u example_worker.py add 4 5

2) mode pipe JSON ligne par ligne
   python3 -u example_worker.py --pipe
"""

from __future__ import annotations

import argparse
import math
import sys
from catj_py_helper import emit_json, serve_json_loop


def cli_mode(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["add", "norm2", "echo"])
    parser.add_argument("values", nargs="*")
    args = parser.parse_args(argv)

    if args.command == "add":
        vals = [float(v) for v in args.values]
        emit_json({"ok": True, "command": "add", "result": sum(vals)})
        return 0

    if args.command == "norm2":
        vals = [float(v) for v in args.values]
        emit_json({"ok": True, "command": "norm2", "result": math.sqrt(sum(v * v for v in vals))})
        return 0

    emit_json({"ok": True, "command": "echo", "result": args.values})
    return 0


def pipe_handler(req: dict) -> dict:
    cmd = req.get("cmd", "")

    if cmd == "ping":
        return {"ok": True, "reply": "pong"}

    if cmd == "add":
        values = req.get("values", [])
        return {"ok": True, "result": sum(float(v) for v in values)}

    if cmd == "norm2":
        values = req.get("values", [])
        return {"ok": True, "result": math.sqrt(sum(float(v) * float(v) for v in values))}

    if cmd == "detect_mock":
        return {
            "ok": True,
            "detected": True,
            "label": "ball",
            "confidence": 0.92,
            "cx": 318,
            "cy": 201,
        }

    return {"ok": False, "error": f"unknown_cmd: {cmd}"}


def main() -> int:
    if "--pipe" in sys.argv:
        serve_json_loop(pipe_handler)
        return 0
    return cli_mode(sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
