"""JSONL capture format.

Line 1 is a header record:
    {"type": "header", "version": 1, "wall": "...", "mode": "bridge", ...}
Every following line is one of:
    {"t": <ns since start>, "d": "c2p" | "p2c" | "h2p", "x": "<hex>"}   data
    {"t": <ns since start>, "d": "mark", "note": "..."}                 annotation

"c2p" is camera -> PTZ board, "p2c" is PTZ board -> camera, "h2p" is a frame
the host itself sent to the PTZ board while bridging (a probe), and "c2m" is
camera output that was muted (logged, never sent to the board). One data record
holds whatever a single read() returned, so record boundaries are USB/driver
artefacts, not frame boundaries; framing.py reassembles frames.
"""

from __future__ import annotations

import datetime as _dt
import gzip
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Iterator

FORMAT_VERSION = 1
C2P = "c2p"
P2C = "p2c"
H2P = "h2p"  # injected by the host into the PTZ side during a bridge run
C2M = "c2m"  # camera bytes logged but NOT forwarded (bridge --mute-cam)
MARK = "mark"


@dataclass
class Record:
    t: int  # ns since capture start
    d: str  # C2P, P2C or MARK
    data: bytes = b""
    note: str = ""


def git_revision() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=2,
            cwd=Path(__file__).resolve().parent,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


class LogWriter:
    def __init__(self, fp: IO[str], header: dict):
        self.fp = fp
        hdr = {
            "type": "header",
            "version": FORMAT_VERSION,
            "wall": _dt.datetime.now(_dt.timezone.utc).isoformat(),
            "git": git_revision(),
        }
        hdr.update(header)
        self._line(hdr)

    def _line(self, obj: dict) -> None:
        self.fp.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def data(self, t: int, d: str, data: bytes) -> None:
        self._line({"t": t, "d": d, "x": data.hex()})

    def mark(self, t: int, note: str) -> None:
        self._line({"t": t, "d": MARK, "note": note})

    def flush(self) -> None:
        self.fp.flush()


def read_log(path: str | Path) -> tuple[dict, list[Record]]:
    header: dict = {}
    records: list[Record] = []
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as fp:
        for n, line in enumerate(fp, 1):
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("type") == "header":
                header = obj
                continue
            if obj["d"] == MARK:
                records.append(Record(obj["t"], MARK, note=obj.get("note", "")))
            else:
                records.append(Record(obj["t"], obj["d"], bytes.fromhex(obj["x"])))
    return header, records


def iter_direction(records: list[Record], d: str) -> Iterator[Record]:
    return (r for r in records if r.d == d)
