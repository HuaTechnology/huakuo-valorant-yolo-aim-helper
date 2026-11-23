from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dma.core.config import Settings, load, save  # noqa: E402


@pytest.fixture()
def settings() -> Settings:
    return Settings()


@pytest.fixture()
def tmp_settings(tmp_path):
    def _make(**patches):
        from dma.core.config import set_fields

        cfg = set_fields(Settings(), patches) if patches else Settings()
        path = tmp_path / "settings.json"
        save(cfg, path)
        return load(path)

    return _make
