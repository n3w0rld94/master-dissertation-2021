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
sleep 3
gnome-terminal --tab -- bash -c "echo '123' | sudo -S yarpserver"
sleep 2
gnome-terminal --tab -- "iCub_SIM"

echo "roscore... Ok"
echo "yarpserver... Ok"
echo "icubsim... Ok"

sleep 2

echo "========================================================================="
echo "||                                                                     ||"
echo "|| Setup completed... Press any key to start the RAT Model simulation. ||"
echo "||                                                                     ||"
echo "========================================================================="

wait 
