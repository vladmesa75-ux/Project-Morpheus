#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")/../MorpheusHelicopter"
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
flags=(-std=c++11 -Wall -Wextra -Werror -pedantic -I.)
tabs=(RotorState.cpp RpmGovernor.cpp YawControl.cpp Pidf.cpp HeliMode.cpp Collective.cpp Cyclic.cpp OutputMixer.cpp)
g++ "${flags[@]}" ../tests/control_test.cpp "${tabs[@]}" -o "$build_dir/control"
"$build_dir/control"
g++ "${flags[@]}" -I../tests/host ../tests/flight_loop_test.cpp FlightComputer.cpp "${tabs[@]}" -o "$build_dir/flight"
for profile in flight rpm yaw; do "$build_dir/flight" "$profile"; done
