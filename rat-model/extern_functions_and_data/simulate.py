"""Run simulation."""

###############################################################################
# SIMULATION PARAMETERS
###############################################################################

import rospy
import numpy as np

from netpyne import sim
from std_msgs.msg import Float64, String


class Mixin7:

    # SEED 1: {'conn': 1111, 'stim': 1111, 'loc': 1111}

    def simulate(self, dt=0.1, lfp=False, seeds=None): 
        step_size = 0.0002
        steps = 31
        current_stimulation = 0

        stimulation = {
            'conds':{'source':'Input_th'},
            'cellConds':{'pop':'TH'},
            'amp':0
        }

        self.buildSimConfig(dt=dt, lfp=lfp, seeds={'conn': 1111, 'stim': 1111, 'loc': 1111})
        sim.create(netParams=self.netParams, simConfig=self.simConfig)

        self.last_index_read = 0
        self.num_of_cortex_cells = 10
        # self.sub = rospy.Subscriber('icub_sensory_data', Float64, self.process_input_data)
        # self.pub = rospy.Publisher('rat_control_commands', Float64)
        # rospy.init_node('neural_model_rat', anonymous=False)
        advertiser = "Stimulation logs\n\n"
        print(advertiser)

        filename = "logs/stimulation-logs-parkinsonian.txt"
        self.file_object = open(filename, 'a')
        self.file_object.write(advertiser)

        for i in range(steps):
            stimulation['amp'] = current_stimulation
            sim.net.modifyStims(stimulation)
            stim_label = format(current_stimulation, '.4f')
            self.file_object.write('\n' + stim_label + ', ')

            sim.runSimWithIntervalFunc(800, self.send_data)
            
            current_stimulation += step_size

        self.file_object.close()


    def process_input_data(self, network, data):
        sensory_data = data.data
        stimulation = {
                'conds':{'source':'Input_th'},
                'cellConds':{'pop':'TH'},
                'amp':0.0012
            }
        print('Input data received' + str(sensory_data))

        if (sensory_data > 0.5):
            stimulation['amp'] = 0.0016
            sim.net.modifyStims(stimulation)
        else:
            sim.net.modifyStims(stimulation)


    def send_data(self, time):

        cortex_spikes = [[],[],[],[],[],[],[],[],[],[]]
        sim.gatherData(False)
    
        num_spikes, cortex_spikes = self.count_and_store_cortex_spikes(cortex_spikes)

        self.last_index_read = len(sim.allSimData['spkid'])
        mean_firing_rate = num_spikes/self.num_of_cortex_cells

        mean_inter_spike_interval_across_cells = self.get_mean_inter_spikes_interval(cortex_spikes)

        print('Data sent: ' + str(mean_inter_spike_interval_across_cells))
        # self.pub.publish(Float64(data=mean_inter_spike_interval_across_cells))

        self.file_object.write(format(mean_inter_spike_interval_across_cells, '.2f') + ", ")


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

    def get_mean_inter_spikes_interval(self, cortex_spikes):
        inter_spike_interval_means = np.zeros(10)
        inter_spike_intervals = [[],[],[],[],[],[],[],[],[],[]]
        
        for i in range(0, self.num_of_cortex_cells):
            cell_spikes = cortex_spikes[i]
            cell_inter_spike_variations = inter_spike_intervals[i]

            for j in range(1, len(cell_spikes)):
                spike_variation = cell_spikes[j] - cell_spikes[j - 1]
                cell_inter_spike_variations.append(spike_variation)
            
            if len(cell_inter_spike_variations) > 0:
                inter_spike_interval_means[i] = np.mean(cell_inter_spike_variations)
            else:
                inter_spike_interval_means[i] = 0

        # Calculate the mean values of cortex neurons
        return np.mean(inter_spike_interval_means)

    def is_spike_from_cortex(self, spike_id):

        return spike_id >= 50 and spike_id < 60
