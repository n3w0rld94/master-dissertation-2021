Last version: 26 May 2010, Fernando

RUNNING:

If everything about the requirements is working properly, just:

> roslaunch webots_ros complete_test.launch

...to open Webots in a NAO simple world. On the left sidebar of the simulator select: NAO -> controller -> click in the "Select..." button in a small windows below. Switch the controller to 'nao_controller'.

Check in the 'Console window' of the simulator (located in the down side) if no error message appears. If it's ok, make a stimulation occurs on neuron(s) through a sensor reading of NAO.

Inside the file nao_controller.py there are plenty of comments and docstrings about the code. You can read them in order to understand its operation. To see which kind of NAO sensors are enabled in the code, check the methods findAndEnableDevices() and run() from the class FireNao. If you want to use other sensors, look the available ones from the NAO model of Webots:
   - https://cyberbotics.com/doc/guide/nao
   - https://cyberbotics.com/doc/guide/sensors

Requirements to run:
   - 1 - Webots - 14 Jan 2020 - R2020a-rev1, https://cyberbotics.com/doc/guide/installation-procedure
       - Python 2.7, 3.7 (Linux, Windows, macOS)
       - Python 3.6 (also for Ubuntu 18.04 and 16.04)
       - Python 3.8 (also for macOS, Ubuntu 18.04 and 16.04)
   
   - 2 - ROS - ROS Melodic Morenia (it works with Python 2.7, but let's hope it doesn't break this code written in Python 3. Amem.). Install the full version: http://wiki.ros.org/melodic/Installation
   
   - 3 - IT IS RECOMMENDED TO INSTALL NEURON and NETPYNE on ANACONDA. First install it: https://www.anaconda.com/products/individual
       - Then them:
           - Neuron - 7.7.2 (Python 3), https://www.neuron.yale.edu/neuron/
           - Netpyne (without GUI) - May 2020 (Python 2.7, 3.6 and 3.7), http://www.netpyne.org/install.html   
   
   - 4 - Finish by installing and configuring the merger between webots and ros, from step 2: https://github.com/jhielson/neurorobotics_computational_environment
   
   - 5 - Read the documentation of Webots: https://cyberbotics.com/doc/guide/
      - A digest:
		- https://cyberbotics.com/doc/guide/introduction-to-webots
		- https://cyberbotics.com/doc/guide/introduction
		- https://cyberbotics.com/doc/guide/controller-start-up
		- https://cyberbotics.com/doc/guide/webots-built-in-editor
		- https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project
		- https://cyberbotics.com/doc/guide/controller-programming
		- https://cyberbotics.com/doc/guide/cpp-java-python
