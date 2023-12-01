#!/bin/bash

# Obtain the path to the application
SCRIPT_DIR=$(cd $(dirname $0); pwd)
DIV_DIR=${SCRIPT_DIR%/*}
PCD_DIV=$DIV_DIR"/build/pointcloud_divider"

# Show usage
function usage() {
cat <<_EOT_
Usage:
  $0 <PCD_0> ... <PCD_N> <OUTPUT_PCD_DIR> <PREFIX> <CONFIG_FILE>

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

# Check the number of runtime arguments
if [ "$#" -lt 4 ]; then
  echo "Error: divider_core.sh requires 4 or more arguments."
  usage
  exit 1
fi

# Input arguments
ARGC=$#
ARGV=("$@")

# Total number of input PCD files
N_PCD=$(($ARGC-3))

# Prepare PCD file names
PCD_FILES=""
for (( i=0; i<N_PCD; i++ ))
do
  PCD_FILES+="${ARGV[i]}"" "
done

# Remove trailing space if any
PCD_FILES="$(echo -e "${PCD_FILES}" | sed -e 's/[[:space:]]*$//')"

# Prepare other file pathes
OUTPUT_DIR=${ARGV[$(($ARGC-3))]}"/"
PREFIX=${ARGV[$(($ARGC-2))]}
CONFIG_FILE=${ARGV[$(($ARGC-1))]}

# Call the pointcloud_divider
$PCD_DIV $N_PCD $PCD_FILES $OUTPUT_DIR $PREFIX $CONFIG_FILE
