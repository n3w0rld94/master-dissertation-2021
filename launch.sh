###################### Ilyasse Fakhreddine #######################


#!/bin/bash

trap "kill 0" SIGINT

# Initiating Compilation.

echo "Initiating Compilation..."

(cd ./icub-control && cmake ./)
(cd ./icub-control && make)

echo "iCub Controller... Ok"

echo "All modules compiled successfully."
sleep 1

echo "Starting roscore, yarpserver and iCub Simulation nodes..."

roscore >/dev/null &
sleep 2
sudo yarpserver >/dev/null &
sleep 2
iCub_SIM >/dev/null &

echo "roscore... Ok"
echo "yarpserver... Ok"
echo "icubsim... Ok"

sleep 2

echo "Starting robot controller..."

gnome-terminal -e "`pwd`/icub-control/icub-control" &
echo "Node icub-controller... Ok"

sleep 2

echo "setup completed... Press any key to start the RAT Model simulation."

# Wait for a keypress
read -n 1 -s

(cd ./rate-model && gnome-terminal -e "nrniv -python main.py" &

echo "Running RAT model... Ok".

wait 
