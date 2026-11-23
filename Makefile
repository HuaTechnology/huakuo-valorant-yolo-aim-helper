PY ?= .venv/bin/python
export PYTHONPATH := src

.PHONY: help test bench sim doctor run autotune fmt clean

help:
	@echo "make test      运行测试套件"
	@echo "make bench     跑仿真回归门禁（新控制律 vs v1）"
	@echo "make sim       打印全场景对比表"
	@echo "make doctor    环境/模型体检"
	@echo "make run       合成画面演示（无需硬件）"
	@echo "make autotune  离线自动调参"

test:
	$(PY) -m pytest

bench:
	$(PY) -m dma.cli bench

sim:
	$(PY) -m dma.cli sim --compare

doctor:
	$(PY) -m dma.cli doctor --probe --dry-run

run:
	$(PY) -m dma.cli run --sim --aim --dry-run

autotune:
	$(PY) -m dma.cli autotune --passes 2

clean:
	find . -name __pycache__ -prune -exec rm -rf {} + 2>/dev/null || true
