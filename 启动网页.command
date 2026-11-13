#!/bin/bash
# GLUON 机械臂网页启动脚本
# 双击本文件，或在终端运行：  ./启动网页.command
# 模拟模式：              ./启动网页.command --mock
# 指定端口：              ./启动网页.command 8771
set -u
cd "$(dirname "$0")/arm-control" || exit 1

MOCK=()
MODE="真机"
if [ "${1:-}" = "--mock" ]; then
  MOCK=(--mock)
  MODE="模拟（--mock）"
  shift
fi
PORT="${1:-8770}"

if command -v lsof >/dev/null 2>&1 && lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "端口 $PORT 已被占用。先停掉占用进程，或换端口："
  echo "  ./启动网页.command 8771"
  echo
  lsof -nP -iTCP:"$PORT" -sTCP:LISTEN
  echo
  read -n 1 -s -r -p "按任意键关闭..."
  exit 1
fi

echo "启动 GLUON 网页执行器"
echo "  目录: $(pwd)"
echo "  模式: $MODE"
echo "  端口: $PORT"
echo "  地址: http://127.0.0.1:$PORT/"
echo
echo "停止：Ctrl+C"
echo

python3 -m arm.executor "${MOCK[@]}" --port "$PORT"
RC=$?
echo
echo "执行器已退出（code=$RC）。"
read -n 1 -s -r -p "按任意键关闭..."
exit $RC
