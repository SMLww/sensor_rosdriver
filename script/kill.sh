#!/bin/bash

LAUNCH_FILES=("mpmgs201.launch" "mprid1356.launch")

for launch in "${LAUNCH_FILES[@]}"; do
    echo "Find and kill $launch node..."
    
    PIDS=$(ps -ef | grep "$launch" | grep -v grep | awk '{print $2}')
    
    if [ -n "$PIDS" ]; then
        kill -SIGINT $PIDS
        echo "Successfully send stop signal to the process: $PIDS"
        
        sleep 2
        
        PIDS_REMAIN=$(ps -ef | grep "$launch" | grep -v grep | awk '{print $2}')
        if [ -n "$PIDS_REMAIN" ]; then
            kill -9 $PIDS_REMAIN
            echo "Residual processes have been forcibly terminated: $PIDS_REMAIN"
        fi
    else
        echo "No find $launch processes"
    fi
done

echo "Operation complete !!!"
    