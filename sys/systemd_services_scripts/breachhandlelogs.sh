#!/bin/bash

# needed amout of space in kB
NEEDED=10000000
MAX_SIZE_BAG=20000000
MAX_SIZE_LOG=500000
# log dir
LOG_DIR="/home/breach/.ros/log/"
BAG_DIR="/home/breach/.ros/bag/breach/"


# check size
DF_REC=$(df . | tail -1)
DEVICE=$(echo $DF_REC | awk '{ print $1 }')
TOTAL=$(echo $DF_REC | awk '{ print $2 }')
FREE=$(echo $DF_REC | awk '{ print $4 }')

echo "Current state of the log dir $LOG_DIR: "
echo "   on device $DEVICE, capacity $TOTAL kB, free $FREE kB"

# BAG DIR

# check dir
if ! cd $BAG_DIR
then
  echo "Cannot switch to backup directory $BAG_DIR"
  exit 1
fi

echo "clearing bag directory $BAG_DIR"

# clear
FILES=$(ls -p -r --sort=time)
for f in $FILES
do
    # check the available space
  CHECK=`du -s ./`
  BAG_SIZE=$(echo $CHECK | awk '{ print $1 }')

  echo "bag folder size $BAG_SIZE kB, max_size $MAX_SIZE_BAG kB"

    if [ "$BAG_SIZE" -lt "$MAX_SIZE_BAG" ]; then
        echo "Log space in bag folder OK, finished"
        break
    fi

  # remove the oldest log directory
    echo "Log space not sufficient, removing folder '$LOG_DIR$f'..."
    rm -r $BAG_DIR$f
done

# LOG DIR

# check dir
if ! cd $LOG_DIR
then
  echo "Cannot switch to backup directory $LOG_DIR"
  exit 1
fi

echo "clearing log directory $LOG_DIR"

# clear
FILES=$(ls -p -r --sort=time)
for f in $FILES
do
    # check the available space
  CHECK=`du -s ./`
  LOG_SIZE=$(echo $CHECK | awk '{ print $1 }')

  echo "log folder size $LOG_SIZE kB, max_size $MAX_SIZE_LOG kB"

    if [ "$LOG_SIZE" -lt "$MAX_SIZE_LOG" ]; then
        echo "Log space in log folder OK, finished"
        break
    fi

  # remove the oldest log directory
    echo "Log space not sufficient, removing folder '$LOG_DIR$f'..."
    rm -r $LOG_DIR$f
done


