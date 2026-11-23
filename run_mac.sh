#!/bin/bash
# 在 macOS 上启动（可被 SSH 远程调用）。所有路径相对仓库根，可直接 clone 使用。
#   ./run_mac.sh run        真机运行（开启瞄准）
#   ./run_mac.sh run-dry    真机运行但不写串口
#   ./run_mac.sh gui        桌面控制台
#   ./run_mac.sh sim        合成画面演示（无需硬件）
#   ./run_mac.sh doctor     体检：模型/后端/串口/推理基准
#   ./run_mac.sh selftest   8 秒闭环自检：直接说明为什么不动/不开火
#   ./run_mac.sh bench      仿真回归门禁
set -u
PROJ="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJ" || exit 1
export PYTHONPATH="$PROJ/src"
export ORT_DISABLE_TELEMETRY=1
PY="${PYTHON:-$PROJ/.venv/bin/python}"
[ -x "$PY" ] || PY="$(command -v python3)"
mkdir -p out/logs
MODE="${1:-run}"; shift 2>/dev/null || true

case "$MODE" in
  run)       exec "$PY" -m dma.cli run --aim "$@" ;;
  run-dry)   exec "$PY" -m dma.cli run --aim --dry-run "$@" ;;
  gui)       exec "$PY" -m dma.cli gui --aim "$@" ;;
  sim)       exec "$PY" -m dma.cli run --sim --aim --dry-run "$@" ;;
  doctor)    exec "$PY" -m dma.cli doctor --probe "$@" ;;
  selftest)  exec "$PY" -m dma.cli selftest "$@" ;;
  bench)     exec "$PY" -m dma.cli bench "$@" ;;
  *) echo "用法: $0 [run|run-dry|gui|sim|doctor|selftest|bench]"; exit 64 ;;
esac
