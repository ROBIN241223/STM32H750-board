#!/bin/bash
# Run STM32H750 simulation in Renode
# Usage: ./renode/run.sh [script.resc]

SCRIPT="${1:-renode/simulate.resc}"

# Clear snap env vars to avoid libpthread conflict
env -i PATH="/usr/bin:/bin:/usr/local/bin" \
    HOME="$HOME" \
    TERM="$TERM" \
    LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu" \
    dotnet /opt/renode/bin/Renode.dll -e "include @$SCRIPT" 2>&1
