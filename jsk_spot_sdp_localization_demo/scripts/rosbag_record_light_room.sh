#!/bin/bash

# This script plays a rosbag file in a loop

function abs_path() {
    echo "$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
}

ROSBAG_FILES=($@)

if [ ${#ROSBAG_FILES[@]} -eq 0 ]; then
    echo "Usage: $0 <rosbag_file>"
    exit 1
fi

ABSPATH_ROSBAG_FILES=""
for ROSBAG_FILE in ${ROSBAG_FILES[@]}; do
    echo "ROSBAG_FILE: ${ROSBAG_FILE}"
    ABSPATH_ROSBAG_FILES+="$(abs_path ${ROSBAG_FILE}) "
done

echo "ROSBAG_FILES: ${ROSBAG_FILES[@]}"
echo "ABSPATH_ROSBAG_FILES: ${ABSPATH_ROSBAG_FILES}"

roslaunch jsk_spot_startup rosbag_record.launch additional_topics:="/light_room_demo/debug" rosbag:="${ABSPATH_ROSBAG_FILES}"
