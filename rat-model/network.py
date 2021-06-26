"""..."""

# Last edition: 23 November 2020, Fernando Zuher
# Refactored from: GenericBg.py, 06 May 2020
 
from netpyne import specs, sim
import rospy

from std_msgs.msg import Float64

# functions used in the class Network
from extern_functions_and_data import buildPopulationParameters, buildCellRules
from extern_functions_and_data import buildSynMechParams, buildCellConnRules
from extern_functions_and_data import buildStimParams, buildSimConfig, simulate

# About functions of class into multiple files:
#    http://www.qtrac.eu/pyclassmulti.html
#    https://stackoverflow.com/questions/47561840/python-how-can-i-separate-functions-of-class-into-multiple-files

class Network(buildPopulationParameters.Mixin1, buildCellRules.Mixin2,
  buildSynMechParams.Mixin3, buildStimParams.Mixin4, buildCellConnRules.Mixin5,
  buildSimConfig.Mixin6, simulate.Mixin7):
    """???"""

    def __init__(self, has_pd=0, it_num=1, dbs=0, t_sim=1000, seed=None):

        self.pd = has_pd # *** se tem doenca de parkinson. 0 = false, 1 = true
        self.it_num = it_num # *** ??? 

        # deep brain stimulation: caso queira simular o dbs.
        # Nao estah sendo usado isso no momento.
        self.dbs = dbs
        self.t_sim = t_sim # simulation time, self.timeStep from Webots

        #######################################################################  
        # netParams is a dict containing a set of network parameters using a
        # standardized structure.
        # simConfig is a dict containing a set of simulation configurations
        # using a standardized structure.

        # object of class NetParams to store the network parameters
        self.netParams = specs.NetParams()
        
        # object of class SimConfig to store the simulation configuration
        self.simConfig = specs.SimConfig()
        
        # specs is imported in modules: from netpyne import specs
        #######################################################################

        ###################################
        # NETWORK PARAMETERS
        self.buildPopulationParameters()
        self.buildCellRules()
        self.buildSynMechParams()
        self.buildStimParams()
        self.buildCellConnRules()
        ###################################
