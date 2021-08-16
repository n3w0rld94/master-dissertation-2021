# master-dissertation-2021

This repository contains threee main folders:

1. The RAT model: This is the modified version of the RAT model to transmit data via ROS to the iCub controller. To run it, open the folder in your terminal and execute the following command: `nrniv -python main.py`.
2. The iCub controller: This is the controller operating the iCub robot and corresponds to prototype 1 and 3. To run it, open the folder, open the build folder in your terminal and execute the following commands 
  a. `cmake ..;`
  b. `make`
  c. `./ice_controller`
3. The Neuro Stabiliser: This is the spiking neural network developed as part of prototype 4 and 5. It is in its early stages and performs classification on pre-recorded spiking data. To run it, open the folder in your terminal and simply execute the following command: `neuro-stabilizer.py`.


In the RAT Model folder there is a folder called logs:
1. Each file represents a different initialisation seed for the RAT model, and presents a matrix of ISI values.
2. The first column represents the stimulation applied to NetPyNE
3. The dataset is divided in Prkinsonian and non-parkinsonian for ease of analysis

Running the solution described in prototype 2 and 3 requires the setup of YARP to use ROS (see official docs fora guide on how to setup your machine).
Moreover, before testing, `roscore`, `sudo yarpserver` and `iCub_SIM` need to be run pre-emptively.
