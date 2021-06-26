"""Run simulation."""

###############################################################################
# SIMULATION PARAMETERS
###############################################################################

import rospy
import numpy as np

from netpyne import sim
from std_msgs.msg import Float64, String


class Mixin7:

    def simulate(self, dt=0.1, lfp=False, seeds=None):

        self.buildSimConfig(dt=dt, lfp=lfp, seeds=seeds)

        sim.create(netParams=self.netParams, simConfig=self.simConfig)

        self.sub = rospy.Subscriber('icub_sensory_data', String, self.process_input_data)
        self.pub = rospy.Publisher('rat_control_commands', Float64)
        rospy.init_node('neural_model_rat', anonymous=False)

        sim.runSimWithIntervalFunc(1000, self.send_data)


    def process_input_data(network, data):

      print('Input data received' + str(data))


    def send_data(self, time):
        # Check time
        self.current_interval = time

        # Collect Data 
        sim.gatherData()

        # Variables to calculate ISIs(InterSpikes Interval)
        one_second_data = [[],[],[],[],[],[],[],[],[],[]]

        # Count spikes: cortex neurons        
        countSpikes = 0
        for i in range(self.previous_interval,len(sim.allSimData['spkid'])):
            spike_id = sim.allSimData['spkid'][i]
            if spike_id >= 50 and spike_id < 60:
                countSpikes += 1
                pos = int(spike_id%50)
                one_second_data[pos].append(sim.allSimData['spkt'][i])

        # Save position for the next second          
        self.previous_interval = len(sim.allSimData['spkid'])

        # Calculate the average FR (firing rate) of Cortex neurons
        mean_FR = countSpikes/10.0

        # Calculate ISIs
        isis_mean = np.zeros(10)
        isis_std  = np.zeros(10)
        isis = [[],[],[],[],[],[],[],[],[],[]]
        for i in range(0,10):
            for t in range(0,len(one_second_data[i])):
                if t>0:
                    diff = one_second_data[i][t]-one_second_data[i][t-1]
                    isis[i].append(diff)
            if len(isis[i]) > 0:
                isis_mean[i] = np.mean(isis[i])
                isis_std[i] = np.std(isis[i])
            else:
                isis_mean[i] = 0
                isis_std[i] = 0

        # Calculate the mean values of cortex neurons
        isis_mean_ctx = np.mean(isis_mean)
        # isis_std_ctx = np.mean(isis_std)


        print('Data sent: ' + str(isis_mean_ctx))
        self.pub.publish(Float64(data=isis_mean_ctx))
        print('SENDING DONE')
