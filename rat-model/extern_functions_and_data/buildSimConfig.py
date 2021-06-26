"""Função usada na simulação."""

###############################################################################
# SIMULATION PARAMETERS
###############################################################################

from extern_functions_and_data import data

class Mixin6:

    def buildSimConfig(self, dt=0.1, lfp=False, seeds=None):
        """Define tempos, intervalos de... Configuração da simulação"""

        # Simulation parameters
        # simConfig = specs.SimConfig() I put it in the constructor of the class Network
        self.simConfig.duration = self.t_sim  # Duration of the simulation, in ms
        self.simConfig.dt = dt  # Internal integration timestep to use
        self.simConfig.verbose = True  # Show detailed messages
        self.simConfig.printPopAvgRates = True

        if seeds is not None:
            self.simConfig.seeds = seeds

        # Recording
        
        # Dict with traces to record
        self.simConfig.recordTraces = {'V_soma':{'sec':'soma','loc':0.5,'var':'v'}}
        self.simConfig.recordStep = 1  # Step size in ms to save data (eg. V traces, LFP, etc)
        self.simConfig.recordCells = ['allCells']
        self.simConfig.recordStim = True  # record spikes of cell stims
        self.simConfig.recordSpikesGids = True

        # Saving
        estado = 'PARKINSONIANO_' if self.pd else 'SAUDÁVEL_'
        self.simConfig.filename = estado # Set file output name
        # self.simConfig.filename = 'output' # Set file output name
        self.simConfig.saveFileStep = self.t_sim # step size in ms to save data to disk
        self.simConfig.savePickle = False # Whether or not to write spikes etc. to a .mat file

        #self.simConfig.analysis['plotRaster'] = {'saveFig': 'raster_80_neurons.png'} # True  # Plot raster
        #self.simConfig.analysis['plotRaster'] = {'saveFig': True} # True  # Plot raster
        
        # potencial de membrana para um neurônio arbitrário
        #self.simConfig.analysis['plotTraces'] = {'include': ['CTX_RS'], 'overlay': True, 'oneFigPer': 'trace', 'saveFig': True}
        #self.simConfig.analysis['plotTraces'] = {'include': ['CTX_RS'], 'overlay': False, 'oneFigPer': 'trace', 'saveFig': True}

        #self.simConfig.analysis['plotRaster'] = {'saveFig': True} # True  # Plot raster
        
        # plot lfp: 'timeSeries', 'locations', 'PSD', 'spectogram'
        # if lfp:
        #    self.simConfig.recordLFP = data.electrodesPos
        #    # self.simConfig.saveLFPCells = True
        #    self.simConfig.analysis['plotLFP'] = {'electrodes': ['all'],
        #       'includeAxon': False, 'timeRange': [0, self.t_sim], # before: [0, 2000],
        #       'plots': ['timeSeries', 'PSD'],
        #       #'plots': ['timeSeries', 'locations', 'PSD', 'spectrogram'],
        #       'showFig': True, 'saveFig': True}

