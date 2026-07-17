#!/usr/bin/env bash

set -euo pipefail

exec ros2 bag record \
    /tf \
    /tf_static \
    /talos/mapping/torpedo \
    /talos/mapping/table \
    /talos/odometry/filtered \
    /talos/ivc/pinger/selected_freq_amp_stream
