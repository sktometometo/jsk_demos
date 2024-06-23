#!/bin/bash

# This script plays a rosbag file in a loop

function abs_path() {
    echo "$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
}

function usage() {
    echo "Usage: $0 <rosbag_file1> <rosbag_file2> ... [-s START_TIME]"
    exit 1
}

START_TIME=""

while getopts ":s:" opt; do
    case ${opt} in
    s)
        START_TIME=$OPTARG
        ;;
    \?)
        echo "Invalid option: -$OPTARG" >&2
        usage
        ;;
    :)
        echo "Option -$OPTARG requires an argument." >&2
        usage
        ;;
    esac
done
shift $((OPTIND - 1))

# Check if at least one rosbag file is provided
if [ $# -lt 1 ]; then
    usage
fi

ROSBAG_FILES=()
while [[ $# -gt 0 ]]; do
    case $1 in
    -s)
        if [ -n "$2" ]; then
            START_TIME=$2
            shift 2
        else
            echo "Option -s requires an argument." >&2
            usage
        fi
        ;;
    *)
        ROSBAG_FILES+=("$1")
        shift
        ;;
    esac
done

echo "ROSBAG_FILES: ${ROSBAG_FILES[@]}"
echo "START_TIME: ${START_TIME}"

if [ ${#ROSBAG_FILES[@]} -eq 0 ]; then
    usage
fi

ABSPATH_ROSBAG_FILES=""
for ROSBAG_FILE in ${ROSBAG_FILES[@]}; do
    echo "ROSBAG_FILE: ${ROSBAG_FILE}"
    ABSPATH_ROSBAG_FILES+="$(abs_path ${ROSBAG_FILE}) "
done

echo "ROSBAG_FILES: ${ROSBAG_FILES[@]}"
echo "ABSPATH_ROSBAG_FILES: ${ABSPATH_ROSBAG_FILES}"

roslaunch jsk_spot_sdp_localization_demo rosbag_play_fd_demo.launch rosbag:="${ABSPATH_ROSBAG_FILES}" start_time:=${START_TIME}
