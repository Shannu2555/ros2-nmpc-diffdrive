#!/bin/bash
# Runs all 4 combos in fresh containers, then makes plots. Needs docker.
set -e
cd "$(dirname "$0")/.."
OUT=${1:-$PWD/runs}; mkdir -p "$OUT"
docker build -t nmpc:humble docker/
for c in mpc pure_pursuit; do for p in figure_eight sharp_corners; do
  docker run --rm -v "$PWD":/repo -v "$OUT":/out nmpc:humble /repo/docker/run_in_docker.sh $c $p /out
done; done
docker run --rm -v "$PWD":/repo -v "$OUT":/out nmpc:humble python3 /repo/tools/analyze.py /out /out/plots
