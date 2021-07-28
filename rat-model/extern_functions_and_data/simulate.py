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

        self.last_index_read = 0
        self.num_of_cortex_cells = 10
        self.sub = rospy.Subscriber('icub_sensory_data', Float64, self.process_input_data)
        self.pub = rospy.Publisher('rat_control_commands', Float64)
        rospy.init_node('neural_model_rat', anonymous=False)

        sim.runSimWithIntervalFunc(800, self.send_data)


    def process_input_data(network, data):
        sensory_data = data.data
        stimulation = {
                'conds':{'source':'Input_th'},
                'cellConds':{'pop':'TH'},
                'amp':0.0012
            }
        print('Input data received' + str(sensory_data))

        if (sensory_data > 0.5):
            stimulation['amp'] = 0.0020
            sim.net.modifyStims(stimulation)
        else:
            sim.net.modifyStims(stimulation)


    def send_data(self, time):

        cortex_spikes = [[],[],[],[],[],[],[],[],[],[]]
        sim.gatherData()
    
        num_spikes, cortex_spikes = self.count_and_store_cortex_spikes(cortex_spikes)

        self.last_index_read = len(sim.allSimData['spkid'])
        mean_firing_rate = num_spikes/self.num_of_cortex_cells

        spike_variations_mean_across_cells = self.get_mean_spikes_variation(cortex_spikes)

        print('Data sent: ' + str(spike_variations_mean_across_cells))
        self.pub.publish(Float64(data=spike_variations_mean_across_cells))


    def count_and_store_cortex_spikes(self, one_second_data):
        spike_ids = sim.allSimData['spkid']
        spike_values = sim.allSimData['spkt']
        num_spikes = 0
        last_index_read = self.last_index_read
        
        for i in range(last_index_read, len(spike_ids)):
            spike_id = spike_ids[i]

            if self.is_spike_from_cortex(spike_id):
                num_spikes += 1
                spike_index = int(spike_id % 50)
                one_second_data[spike_index].append(spike_values[i])
        
        return num_spikes, one_second_data

    def get_mean_spikes_variation(self, cortex_spikes):
        spike_variations_means = np.zeros(10)
        spike_variations = [[],[],[],[],[],[],[],[],[],[]]
        
        for i in range(0, self.num_of_cortex_cells):
            cell_spikes = cortex_spikes[i]
            cell_spike_variations = spike_variations[i]

            for j in range(1, len(cell_spikes)):
                spike_variation = cell_spikes[j] - cell_spikes[j - 1]
                cell_spike_variations.append(spike_variation)
            
            if len(cell_spike_variations) > 0:
                spike_variations_means[i] = np.mean(cell_spike_variations)
            else:
                spike_variations_means[i] = 0

        # Calculate the mean values of cortex neurons
        return np.mean(spike_variations_means)

    def is_spike_from_cortex(self, spike_id):

        return spike_id >= 50 and spike_id < 60
