#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
DIV_DIR=${SCRIPT_DIR%/*}
SRC_DIR=${DIV_DIR%/*}
BASE_DIR=${SRC_DIR%/*}

DIV_CORE=$SCRIPT_DIR"/divider_core.sh"

# Show usage
function usage() {
cat <<_EOT_
Usage:
  $0 <INPUT_PCD_DIR> <OUTPUT_PCD_DIR> <PREFIX> <CONFIG_FILE>

Description:
  Dividing and downsampling PCD files into XY 2D rectangle grids.

Options:
  None

_EOT_
}

# Parse options
if [ "$OPTIND" = 1 ]; then
  while getopts h OPT
  do
    case $OPT in
      h)
        usage
        exit 0 ;;
      \?)
        echo "Undefined option $OPT"
        usage
        exit 1 ;;
    esac
  done
else
  echo "No installed getopts-command." 1>&2
  exit 1
fi
shift $(($OPTIND - 1))

INPUT_DIR=$1
OUTPUT_DIR=$2
PREFIX=$3
CONFIG_FILE=$4

PCD_FILES=$(find $INPUT_DIR -name "*.pcd" -printf "%p ")

$DIV_CORE $PCD_FILES $OUTPUT_DIR $PREFIX $CONFIG_FILE
