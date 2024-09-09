#!/bin/bash
### BEGIN INIT INFO
# Provides:          run_mode_switch
# Required-Start:    $network $local_fs
# Required-Stop:     $network $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Run Mode Switch
# Description:       This script activates the Conda environment and runs a command.
### END INIT INFO

# Define paths
CONDA_PATH="/home/gr1p24ap0058/miniconda3/etc/profile.d/conda.sh"
CONDA_ENV="grx-env"
COMMAND="python mode_listener.py"
WORKING_DIR="/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1"

#tmux new-session -d -s mysession "source $CONDA_PATH; conda activate $CONDA_ENV; cd '$WORKING_DIR'; $COMMAND; exec bash "
#gnome-terminal -- bash -c "source $CONDA_PATH; conda activate $CONDA_ENV; cd '$WORKING_DIR'; $COMMAND; exec bash "
echo "start mode switch..."
. $CONDA_PATH
conda activate $CONDA_ENV
cd "$WORKING_DIR" || exit 1
$COMMAND &
echo "mode swith service started."

exit 0
