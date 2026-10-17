#!/usr/bin/env bash
# Regenerates the README media in docs/media/ (one script per image in tools/media/).
#
#   tools/make_media.sh            offline figures (patterns, behavior tree, results) and the
#                                  sim images assembled from existing snapshot runs
#   tools/make_media.sh --sim      first re-run the four snapshot pallets (headless Gazebo,
#                                  ~1 h wall): sku_runs --snapshot tools/media/_work/runs
#   tools/make_media.sh --rviz     also capture RViz (needs a display; opens an RViz window):
#                                  docs/media/rviz_full.png
#   tools/make_media.sh --gif      also record the README GIF (headless Gazebo, warehouse world,
#                                  ~15 min wall): docs/media/warehouse_demo.gif
#   tools/make_media.sh --all      --sim + --rviz + --gif
#
# Environment: ROS, then the project venv, then the workspace overlay (sourced here).
# Needs graphviz (dot). Uses pngquant / optipng when installed, else PIL quantization.
set -eo pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS/.venv/bin/activate"
source "$WS/install/setup.bash"
set -u
M=$WS/tools/media
RUNS=$M/_work/runs
SIM=0; RVIZ=0; GIF=0
for a in "$@"; do
  case $a in
    --sim) SIM=1 ;;
    --rviz) RVIZ=1 ;;
    --gif) GIF=1 ;;
    --all) SIM=1; RVIZ=1; GIF=1 ;;
    *) echo "usage: $0 [--sim] [--rviz] [--gif] [--all]" >&2; exit 2 ;;
  esac
done

if [[ $SIM == 1 ]]; then
  # SKU:placed boxes before the mid-run shot. Each run: one full pallet, the exchange and the
  # next pallet's first box (CSVs stay in $RUNS/results, results/ is not touched).
  mkdir -p "$RUNS"
  for spec in 600x400x300:6 520x320x250:8 450x350x300:6 380x280x220:18; do
    ros2 run palletizer_orchestrator sku_runs --skus "${spec%%:*}" --snapshot "$RUNS" \
      --hero-after "${spec##*:}" --timeout 5400
  done
fi
if [[ $RVIZ == 1 ]]; then
  python3 "$M/rviz_full.py"
fi
if [[ $GIF == 1 ]]; then
  python3 "$M/record_gif.py"
fi

python3 "$M/patterns.py"
python3 "$M/behavior_tree.py"
python3 "$M/results_chart.py"
python3 "$M/hero_cell.py"
python3 "$M/perception_images.py"
python3 "$M/pallets_gallery.py"
python3 "$M/exchange_sequence.py"

du -ch "$WS"/docs/media/* | tail -1
