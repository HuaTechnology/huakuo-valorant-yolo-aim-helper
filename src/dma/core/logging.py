"""Structured logging: JSON Lines on disk, human readable on the console.

Every log record carries the stage that produced it plus arbitrary structured
fields, so a support bundle is ``out/logs/*.jsonl`` and nothing else.  Rotation
is size based and never blocks the writer for long.
"""
from __future__ import annotations

import json
import logging
import os
import sys
import threading
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Dict

_CONFIGURED = False
_LOCK = threading.Lock()

_RESERVED = {
    "name", "msg", "args", "levelname", "levelno", "pathname", "filename", "module",
    "exc_info", "exc_text", "stack_info", "lineno", "funcName", "created", "msecs",
    "relativeCreated", "thread", "threadName", "processName", "process", "message",
    "taskName",
}


class JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload: Dict[str, Any] = {
            "ts": round(record.created, 6),
            "t": time.strftime("%H:%M:%S", time.localtime(record.created)),
            "lvl": record.levelname,
            "src": record.name,
            "msg": record.getMessage(),
        }
        for key, value in record.__dict__.items():
            if key in _RESERVED or key.startswith("_"):
                continue
            try:
                json.dumps(value)
                payload[key] = value
            except TypeError:
                payload[key] = repr(value)
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


class ConsoleFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        stamp = time.strftime("%H:%M:%S", time.localtime(record.created))
        extra = {
            k: v
            for k, v in record.__dict__.items()
            if k not in _RESERVED and not k.startswith("_")
        }
        tail = " " + " ".join(f"{k}={v}" for k, v in extra.items()) if extra else ""
        return f"{stamp} {record.levelname[0]} {record.name:<22} {record.getMessage()}{tail}"


def setup(
    level: str = "INFO",
    log_dir: str | os.PathLike | None = "out/logs",
    *,
    console: bool = True,
    filename: str = "aim.jsonl",
    max_bytes: int = 8 * 1024 * 1024,
    backups: int = 3,
) -> Path | None:
    """Idempotent logging bootstrap.  Returns the log file path (if any)."""
    global _CONFIGURED
    with _LOCK:
        root = logging.getLogger()
        if _CONFIGURED:
            root.setLevel(getattr(logging, str(level).upper(), logging.INFO))
            return None
        root.setLevel(getattr(logging, str(level).upper(), logging.INFO))
        for h in list(root.handlers):
            root.removeHandler(h)

        if console:
            ch = logging.StreamHandler(stream=sys.stdout)
            ch.setFormatter(ConsoleFormatter())
            root.addHandler(ch)

        path = None
        if log_dir:
            directory = Path(log_dir)
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / filename
            fh = RotatingFileHandler(path, maxBytes=max_bytes, backupCount=backups, encoding="utf-8")
            fh.setFormatter(JsonFormatter())
            root.addHandler(fh)

        # onnxruntime / urllib3 are noisy at DEBUG
        for noisy in ("onnxruntime", "urllib3", "matplotlib"):
            logging.getLogger(noisy).setLevel(logging.WARNING)
        _CONFIGURED = True
        return path


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)


class StageLogger:
    """Logger wrapper that stamps a stage name on every record (rate limited)."""

    __slots__ = ("_log", "_min_interval", "_last")

    def __init__(self, name: str, min_interval_s: float = 0.0):
        self._log = get_logger(name)
        self._min_interval = min_interval_s
        self._last: Dict[str, float] = {}

    def _allowed(self, key: str) -> bool:
        if self._min_interval <= 0:
            return True
        now = time.monotonic()
        last = self._last.get(key, 0.0)
        if now - last < self._min_interval:
            return False
        self._last[key] = now
        return True

    def info(self, msg: str, **fields: Any) -> None:
        self._log.info(msg, extra=fields)

    def warn(self, msg: str, **fields: Any) -> None:
        self._log.warning(msg, extra=fields)

    def debug(self, msg: str, **fields: Any) -> None:
        self._log.debug(msg, extra=fields)

    def error(self, msg: str, **fields: Any) -> None:
        self._log.error(msg, extra=fields)

    def throttled(self, key: str, level: str, msg: str, **fields: Any) -> None:
        if not self._allowed(key):
            return
        getattr(self._log, level, self._log.info)(msg, extra=fields)
