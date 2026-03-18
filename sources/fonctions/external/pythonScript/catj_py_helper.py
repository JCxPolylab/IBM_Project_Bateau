#!/usr/bin/env python3
"""
helpers Python pour dialoguer avec la librairie C++ du projet.
Convention messages JSON ligne par ligne sur stdin/stdout.
"""

from __future__ import annotations

import json
import sys
from typing import Any, Callable, Dict


def emit_text(text: str, end: str = "\n") -> None:
    sys.stdout.write(text + end)
    sys.stdout.flush()


def emit_json(obj: Any) -> None:
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def read_line() -> str | None:
    line = sys.stdin.readline()
    if line == "":
        return None
    return line.rstrip("\r\n")


def read_json() -> Dict[str, Any] | None:
    line = read_line()
    if line is None:
        return None
    if line == "":
        return {}
    return json.loads(line)


def serve_json_loop(handler: Callable[[Dict[str, Any]], Dict[str, Any] | None]) -> None:
    """
    - lit une ligne JSON
    - appelle handler(dict)
    - renvoie la réponse en JSON sur stdout
    Commande réservée : {"cmd":"quit"}
    """
    while True:
        try:
            req = read_json()
        except Exception as exc:
            emit_json({"ok": False, "error": f"invalid_json: {exc}"})
            continue

        if req is None:
            break

        if req.get("cmd") == "quit":
            emit_json({"ok": True, "bye": True})
            break

        try:
            resp = handler(req)
            if resp is not None:
                if "ok" not in resp:
                    resp = {"ok": True, **resp}
                emit_json(resp)
        except Exception as exc:
            emit_json({"ok": False, "error": str(exc)})
