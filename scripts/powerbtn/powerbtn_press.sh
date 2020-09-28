#!/usr/bin/env bash

TIMESTAMP=$(date +%s%N | cut -b1-13)
PRESS_COUNTER=""
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
RESOURCE_DIR=$SCRIPT_DIR/resources
RESOURCE_FILE=$RESOURCE_DIR/powerbtn.event
#only if the press command is registerd
if [[ $2 == LNXPWRBN:00 ]]; then
    PRESS_COUNTER=$4
    /usr/bin/logger "ACPI_POWER_BTTN_EVENT:[$TIMESTAMP]$1:$PRESS_COUNTER"
    echo "[$TIMESTAMP]$1:$PRESS_COUNTER" >> $RESOURCE_FILE
    chmod 777 $RESOURCE_FILE
fi