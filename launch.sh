#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

CAMERA_NAME=${CAMERA_NAME:-foscam_r2}

ARGS=("camera_name:=${CAMERA_NAME}")

if [[ -v CAMERA_PARAM_FILE ]]; then
  ARGS+=("camera_param_file:=${CAMERA_PARAM_FILE}")
fi

if [[ -v CROP_PARAM_FILE ]]; then
  ARGS+=("crop_param_file:=${CROP_PARAM_FILE}")
fi

if [[ -v CONFIG_DIR ]]; then
  ARGS+=("config_dir:=${CONFIG_DIR}")
fi

if [[ -v PARAM_FILENAME ]]; then
  ARGS+=("param_file_name:=${PARAM_FILENAME}")
fi

roslaunch foscam_ros foscam_ros.launch ${ARGS[@]}
