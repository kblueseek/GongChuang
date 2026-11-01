#!/usr/bin/env bash
# 修复 ROS2 Lyrical 的可执行文件解析。
#
# 背景：本环境的 colcon-ros 把 ament_python 包的 console_scripts 装到
#   install/<pkg>/bin/，但 Lyrical 的 ros2 run/launch 只在 install/<pkg>/lib/<pkg>/
#   （libexec 目录）里找可执行文件，找不到就报：
#     "libexec directory '.../lib/<pkg>' does not exist"
# 本脚本给每个包的 bin 脚本在 lib/<pkg>/ 下建软链接，让 ros2 launch 能解析到。
#
# 用法：colcon build 之后跑一次 `bash scripts/mk_libexec.sh`（幂等，可重复跑）。
# run_car.sh 里已经内置调用，平时无需手动跑。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="$REPO_ROOT/ros2_ws/install"

[ -d "$INSTALL_DIR" ] || { echo "未找到 install 目录，先 colcon build" >&2; exit 1; }

for pkg_dir in "$INSTALL_DIR"/*/; do
    pkg="$(basename "$pkg_dir")"
    bin_dir="$pkg_dir/bin"
    [ -d "$bin_dir" ] || continue
    libexec_dir="$pkg_dir/lib/$pkg"
    mkdir -p "$libexec_dir"
    for script in "$bin_dir"/*; do
        [ -f "$script" ] && [ -L "$script" ] && continue   # 跳过 python 解释器软链接
        [ -f "$script" ] || continue
        name="$(basename "$script")"
        case "$name" in
            python*|python3*) continue ;;                   # 跳过解释器相关文件
        esac
        ln -sf "../../bin/$name" "$libexec_dir/$name"
    done
done

echo "libexec 软链接已就绪（install/*/lib/<pkg>/ 指向 bin/ 脚本）"
