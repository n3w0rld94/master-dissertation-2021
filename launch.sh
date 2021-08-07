###################### Ilyasse Fakhreddine #######################

#!/bin/bash

if [[ $EUID -eq 0 ]]; then
    echo "Please do not run this as root."
    exit
fi

trap "kill 0" SIGINT

# Initiating Compilation.

echo "Initiating Compilation..."

(cd ./icub-control && cmake ./)
(cd ./icub-control && make)

echo "iCub Controller... Ok"

echo "All modules compiled successfully."
sleep 1

echo "Starting roscore, yarpserver and iCub Simulation nodes..."

gnome-terminal --tab -- "roscore"
sleep 2
gnome-terminal --tab -- bash -c "echo '123' | sudo -S yarpserver"
gnome-terminal --tab -- "iCub_SIM"
sleep 2
gnome-terminal -- "`pwd`/icub-control/ice_controller"
gnome-terminal --tab -- bash -c "echo 'Go back to the first tab.'; read -n 1 -s"

echo "roscore... Ok"
echo "icubsim... Ok"
echo "Starting robot controller..."
echo "Node ice_controller... Ok"

sleep 2

echo "========================================================================="
echo "||                                                                     ||"
echo "|| Setup completed... Press any key to start the RAT Model simulation. ||"
echo "||                                                                     ||"
echo "========================================================================="

# Wait for a keypress
read -n 1 -s

gnome-terminal --working-directory "/home/ilyasse/Desktop/rat-model-launch" -- bash -c "nrniv -python main.py;"

echo "Running RAT model... Ok".

wait 
