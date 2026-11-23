from __future__ import annotations

import json

from dma.cli import main


def test_version(capsys):
    assert main(["version"]) == 0
    assert "aim2" in capsys.readouterr().out


def test_config_validate(tmp_path, capsys):
    assert main(["--settings", str(tmp_path / "s.json"), "config", "validate"]) == 0
    assert "valid" in capsys.readouterr().out


def test_config_set_and_get(tmp_path, capsys):
    path = str(tmp_path / "s.json")
    assert main(["--settings", path, "config", "set", "aim.kp_far", "0.66"]) == 0
    capsys.readouterr()
    assert main(["--settings", path, "config", "get", "aim.kp_far"]) == 0
    assert "0.66" in capsys.readouterr().out


def test_config_rejects_bad_value(tmp_path):
    path = str(tmp_path / "s.json")
    assert main(["--settings", path, "config", "set", "aim.kp_far", "9"]) == 2


def test_sim_json_output(capsys):
    assert main(["--json", "sim", "--scenario", "micro_drift", "--controller", "predictive"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload and payload[0]["scenario"] == "micro_drift"


def test_bench_gate_passes(capsys):
    assert main(["bench"]) == 0
    assert "pass" in capsys.readouterr().out
