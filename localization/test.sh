#!/usr/bin/env bash
set -euo pipefail

# Generate cases if missing
[[ -d testcases ]] || python3 gen_test.py

# Compile if missing
[[ -x ./lane_align ]] || g++ lane_align_task.cpp -O2 -std=gnu++17 $(pkg-config --cflags --libs opencv4) -I/usr/include/eigen3 -o lane_align

pass=0
fail=0

run_case() {
  local id="$1" roi="$2" horizon="$3" expect="$4"
  local debug_arg=()
  [[ "$expect" == "pass" ]] && debug_arg=(--debug "testcases/$id/debug.png")

  echo "== $id (roi=$roi, horizon=$horizon; expect $expect) =="

  local out
  if ! out=$(./lane_align "testcases/$id/mask.png" "testcases/$id/prior.csv" --roi "$roi" --horizon_frac "$horizon" "${debug_arg[@]}" 2>&1); then
    echo "  program error"
    ((fail++))
    return
  fi
  echo "  $out"

  local has_nan_off=0
  echo "$out" | grep -q 'offset_m=nan' && has_nan_off=1
  local r2
  r2=$(echo "$out" | sed -n 's/.*r2=\(.*\)$/\1/p')
  [[ -z "${r2:-}" ]] && {
    echo "  could not parse r2"
    ((fail++))
    return
  }

  if [[ "$expect" == "pass" ]]; then
    if [[ "$r2" == "nan" || $has_nan_off -eq 1 ]]; then
      echo "  ❌ expected a fit but got NaNs"
      ((fail++))
    elif awk -v v="$r2" 'BEGIN{exit !(v>=0.90)}'; then
      echo "  ✅ pass"
      ((pass++))
    else
      echo "  ❌ r2 too low ($r2)"
      ((fail++))
    fi
  else
    # Expect failure
    if [[ "$r2" == "nan" || $has_nan_off -eq 1 || ! $(awk -v v="$r2" 'BEGIN{print (v>=0.90) ? "1" : "0"}') -eq 1 ]]; then
      echo "  ✅ expected failure"
      ((pass++))
    else
      echo "  ❌ unexpectedly succeeded (r2=$r2)"
      ((fail++))
    fi
  fi
}

run_case 01_good_line 10 0.67 pass
run_case 02_steeper_line 10 0.67 pass
run_case 03_few_points_horizon 10 0.98 fail
run_case 04_vertical_line 10 0.67 fail
run_case 05_noisy_low_r2 10 0.67 fail
run_case 06_needs_dilation 3 0.67 fail
run_case 06_needs_dilation 10 0.67 pass
run_case 07_wrong_prior 20 0.67 fail
run_case 08_curved_lane 10 0.67 fail

echo
echo "Summary: $pass passed, $fail failed"
exit $((fail > 0))
