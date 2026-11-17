#!/usr/bin/env bash
# run_many.sh <algorithm> <first_seed> <last_seed>
# Runs run_trial.sh once per seed, one after the other.
# Example:  bash run_many.sh random_turn 3 12

ALGO="$1"
FIRST="$2"
LAST="$3"
if [ -z "$ALGO" ] || [ -z "$FIRST" ] || [ -z "$LAST" ]; then
  echo "usage: run_many.sh <algorithm> <first_seed> <last_seed>"
  exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
for SEED in $(seq "$FIRST" "$LAST"); do
  bash "$HERE/run_trial.sh" "$ALGO" "$SEED" || echo "trial $ALGO seed $SEED did not finish"
done
echo "all done"
