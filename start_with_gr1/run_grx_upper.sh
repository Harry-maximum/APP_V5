#!/bin/bash
### BEGIN INIT INFO
# Provides:          run_grx_service
# Required-Start:    $network $local_fs
# Required-Stop:     $network $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Run GRX service
# Description:       This script activates the Conda environment and runs a command.
### END INIT INFO

# Define paths
CONDA_PATH="/home/gr1p24ap0058/miniconda3/etc/profile.d/conda.sh"
CONDA_ENV="grx-env"
COMMAND1="grx run ./config/config_GR1_T2.yaml --urdf-path ./urdf"
WORKING_DIR1="/home/gr1p24ap0058/wiki-grx-deploy"

COMMAND2="python remote_control.py"
WORKING_DIR2="/home/gr1p24ap0058/remote_control_upper/APP_V5"

case "$1" in
  start_service)
    echo "Starting GRX service"
    # Source the Conda environment and run the first command
    . $CONDA_PATH
    conda activate $CONDA_ENV
    cd $WORKING_DIR1
    $COMMAND1 &

    ;;
  
  start_algorithm)
  
    echo "Starting GRX remote control algorithm"
    # Run the second command
    cd $WORKING_DIR2
    $COMMAND2 &
    ;;
    
  stop_algorithm)
    echo "Stopping remote control algorithm"

    sudo pkill -f "remote_control.py"
    ;;

  stop_service)
    echo "Stopping GRX service "

    sudo pkill -f "grx"
    ;;
    
  restart)
    $0 stop
    $0 start
    ;;

  *)
    echo "Usage: /etc/init.d/run_grx_service {start_service|start_algorithm|stop_algorithm|stop_service}"
    exit 1
    ;;
esac

exit 0
exit 0
