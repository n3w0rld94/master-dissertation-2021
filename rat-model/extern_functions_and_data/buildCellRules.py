"""Set cell rules for all the desired regions."""

###############################################################################
# NETWORK PARAMETERS
###############################################################################

import random

class Mixin2:

    def buildCellRules(self, seed=None):
        """Call methods that set cell rules for each type/region."""

        random.seed(seed)
        self._rsCellRules()
        self._fsiCellRules()
        self._strD1CellRules()
        self._strD2CellRules()
        self._thCellRules()
        self._gpiCellRules()
        self._gpeCellRules()
        self._stnCellRules()

    def _rsCellRules(self):
        """Define CTX_RS cell properties."""

        cellRule = {'conds': {'cellModel': 'CTX_RS', 'cellType': 'CTX_RS'},
                    'secs': {}} # cell rule dict

        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1, 'cm': 1}

        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
            'a': 0.02, 'b': 0.2, 'c': -65, 'd': 8, 'f': 5, 'g': 140, 'thresh': 30} 

        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30
        
        # add dict to list of cell params
        self.netParams.cellParams['CTX_RS'] = cellRule 

    def _fsiCellRules(self):
        """Define CTX_FSI cell properties."""

        cellRule = {'conds': {'cellModel': 'CTX_FSI', 'cellType': 'CTX_FSI'},
                    'secs': {}}

        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1, 'cm': 1}

        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
            'a': 0.1, 'b': 0.2, 'c': -65, 'd': 2, 'f': 5, 'g': 140, 'thresh': 30} 

        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30

        self.netParams.cellParams['CTX_FSI'] = cellRule

    def _strD1CellRules(self):
        """Define StrD1 cell properties."""

        cellRule = {'conds': {'cellModel': 'StrD1', 'cellType': 'StrD1'},
                    'secs': {}}

        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1}

        cellRule['secs']['soma']['mechs']['Str'] = {
                                             'gmbar': (2.6e-3 - self.pd * 1.1e-3)}

        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10

        self.netParams.cellParams['StrD1'] = cellRule

    def _strD2CellRules(self):
        """Define StrD2 cell properties."""

        cellRule = {'conds': {'cellModel': 'StrD2', 'cellType': 'StrD2'},
                    'secs': {}}

        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1}

        cellRule['secs']['soma']['mechs']['Str'] = {
                                             'gmbar': (2.6e-3 - self.pd * 1.1e-3)}

        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10

        self.netParams.cellParams['StrD2'] = cellRule

    def _thCellRules(self):
        """Define Thal cell properties."""

        cellRule = {'conds': {'cellModel': 'TH', 'cellType': 'Thal'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1}

        cellRule['secs']['soma']['mechs']['thalamus'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10

        self.netParams.cellParams['TH'] = cellRule

    def _gpiCellRules(self, gahp=10e-3):
        """Define GPi cell properties."""

        cellRule = {'conds': {'cellModel': 'GPi', 'cellType': 'GPi'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1}

        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPi']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10

        self.netParams.cellParams['GPi'] = cellRule

    def _gpeCellRules(self, gahp=10e-3):
        """Define GPe cell properties."""

        cellRule = {'conds': {'cellModel': 'GPe', 'cellType': 'GPe'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1}

        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPe']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10

        self.netParams.cellParams['GPe'] = cellRule

    def _stnCellRules(self, gkcabar=5e-3):
        """Define STN cell properties."""

        cellRule = {'conds': {'cellModel': 'STN', 'cellType': 'STN'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642, 'L': 5.642, 'Ra': 1,
                                            'nseg': 1}

        cellRule['secs']['soma']['mechs']['STN'] = {'dbs': self.dbs, 
                                                    'gkcabar': gkcabar}

        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10

        self.netParams.cellParams['STN'] = cellRule
