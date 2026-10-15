#!/bin/bash
# SIMULATION, the stored baselines. Runs the published exploration methods in Isaac Sim on
# the full stack (each car with its own SLAM and Nav2), several full sessions per method,
# and keeps them as the fixed reference every policy is compared with (summary page
# baselines.html). Each session is ./run_isaac_explore.sh --planner METHOD --headless.
# Stopping and starting again continues: finished sessions are kept and counted. Needs the
# ma_explore repository and its venv ($MA_EXPLORE_DIR, default ~/ma_explore). One session
# in the lab takes about 22 minutes; do not train or run other Isaac sessions meanwhile.
#
#   ./run_baselines.sh [--mmpf] [--mtsp] [--burgard] [--alc] [--all] [--sessions N]
#                      [--env big|room] [--time-limit S] [--restart] [--summary]
#
#   --mmpf        MMPF: multi-robot multi-target potential field (SMMR-Explore, Yu et al. 2021)
#   --mtsp        MTSP: frontier clusters as a multiple travelling-salesman problem (Faigl et al. 2012)
#   --burgard     Burgard et al. 2005: coordinated frontier assignment with utility discounting
#   --alc         active loop closing (Stachniss et al. 2004): Burgard plus loop-closing detours
#   --all         all four (the default when no method is given)
#   --sessions N  finished sessions per method (default 4)
#   --env         big (default): the 22 x 16 m lab, 1200 s missions,
#                 results in ~/result/ma_explore/baselines_big/; room: the 6 x 6 m room,
#                 300 s, in ~/result/ma_explore/baselines/
#   --time-limit  another mission length in seconds (default: the environment's)
#   --restart     move the existing results of this environment aside (to <folder>_old_DATE)
#                 and start a new set, e.g. after the world changed
#   --summary     only rewrite the summary page from the stored sessions
#   A single session of any planner (also the rule baselines coordinated, nearest, utility,
#   random) without storing it: ./run_isaac_explore.sh --planner NAME
#
# Results: <folder>/<method>/<session>/ (report.html, explore.html), <folder>/baselines.html
# and summary.json; log in <folder>/bank_run.log.
#
# Examples:
#   ./run_baselines.sh                         all four methods, 4 sessions each, in the lab
#   ./run_baselines.sh --mmpf --alc --sessions 2
#   ./run_baselines.sh --restart               a new set after the lab changed
#   ./run_baselines.sh --env room --summary
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
MAX="${MA_EXPLORE_DIR:-$HOME/ma_explore}"
PY="$HOME/venvs/ma_explore/bin/python"
METHODS=(); SESSIONS=4; ENV=big; LIMIT=(); RESTART=false; SUMMARY=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mmpf|--mtsp|--burgard|--alc) METHODS+=("${1#--}"); shift ;;
        --all) METHODS=(mmpf mtsp burgard alc); shift ;;
        --sessions) SESSIONS="$2"; shift 2 ;;
        --env) ENV="$2"; shift 2 ;;
        --time-limit) LIMIT=(--time-limit "$2"); shift 2 ;;
        --restart) RESTART=true; shift ;;
        --summary) SUMMARY=true; shift ;;
        *) echo "Unknown option: $1   (./run_baselines.sh --help lists the options)"; exit 1 ;;
    esac
done
[ ${#METHODS[@]} -eq 0 ] && METHODS=(mmpf mtsp burgard alc)
case "$ENV" in big) ROOT="$HOME/result/ma_explore/baselines_big" ;; room) ROOT="$HOME/result/ma_explore/baselines" ;;
    *) echo "--env big or room"; exit 1 ;; esac
cd "$MAX" || { echo "no ma_explore at $MAX"; exit 1; }
if $SUMMARY; then exec "$PY" -m ma_explore.baseline_bank summary --env "$ENV"; fi
if pgrep -af "[r]un_isaac.py|[m]a_explore.train" > /dev/null; then
    echo "Isaac or a training run is active; run the baselines when the GPU is free."; exit 1
fi
if $RESTART && [ -d "$ROOT" ]; then
    OLD="${ROOT}_old_$(date +%Y%m%d_%H%M)"; mv "$ROOT" "$OLD"; echo "previous results moved to $OLD"
fi
mkdir -p "$ROOT"
echo "baselines ${METHODS[*]}, $SESSIONS sessions each, env $ENV -> $ROOT (log: $ROOT/bank_run.log)"
"$PY" -m ma_explore.baseline_bank run --env "$ENV" --methods "${METHODS[@]}" --sessions "$SESSIONS" "${LIMIT[@]}" \
    2>&1 | tee -a "$ROOT/bank_run.log"
