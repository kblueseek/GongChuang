#!/usr/bin/env bash
# 旧名字，保留只是为了不打断习惯。现在 PC 和树莓派共用 run_gui.sh，
# 这个文件只是把参数原样转过去：
#     bash scripts/run_gui_pc.sh 10.23.113.193
# 等价于
#     bash scripts/run_gui.sh 10.23.113.193
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_gui.sh" "$@"
