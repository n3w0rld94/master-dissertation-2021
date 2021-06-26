"""Connectivity parameters."""

###############################################################################
# NETWORK PARAMETERS
###############################################################################

import random

class Mixin5:

    def buildCellConnRules(self, seed=None):
        """Regras de conectividade."""

        random.seed(seed)
        self._thConnRules()
        self._gpeConnRules()
        self._gpiConnRules()
        self._stnConnRules()
        self._strConnRules()
        self._ctxConnRules()


    def _thConnRules(self):
        """???."""

        # GPi-> Th connections
        n_th = self.netParams.popParams['TH']['numCells']
        n_gpi = self.netParams.popParams['GPi']['numCells']
        n_neurons = max(n_th, n_gpi)

        self.netParams.connParams['GPi->th'] = {
            'preConds': {'pop': 'GPi'}, 'postConds': {'pop': 'TH'},  # GPi-> th
            'connList': [[i%n_gpi, i%n_th] for i in range(n_neurons)],
            'weight': 0.0336e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igith'}  # target synaptic mechanism

    ################################################################################

    def _gpeConnRules(self, stn_gpe=2, gpe_gpe=2):
        """???."""

        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        n_stn = self.netParams.popParams['STN']['numCells']
        n_gpe = self.netParams.popParams['GPe']['numCells']
        n_neurons = max(n_stn, n_gpe)

        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[(x - c)%n_stn, x%n_gpe] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]

        self.netParams.connParams['STN->GPe'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPe'},  # STN-> GPe
            'connList': connList,  # AMPA
            'weight': weight,  # synaptic weight (conductance)
            'delay': 2,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Insge,ampa'}  # target synaptic mechanism

        ############################################################################

        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[(x - c)%n_stn, x%n_gpe] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.002) * 0.43e-3 for k in range(len(connList))]

        self.netParams.connParams['STN->GPe2'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPe'},  # STN-> GPe
            'connList': connList,  # NMDA
            'weight': weight,  # synaptic weight (conductance)
            'delay': 2,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Insge,nmda'}  # target synaptic mechanism

        ############################################################################

        # GPe-< GPe connections
        n_neurons = self.netParams.popParams['GPe']['numCells']

        connList = [[(idx + ncn) % n_neurons, idx] for ncn in range(1, gpe_gpe+1, 2)
                    for idx in range(n_neurons)] + \
                   [[idx, (idx + ncn) % n_neurons] for ncn in range(2, gpe_gpe+1, 2)
                    for idx in range(n_neurons)]
        # connList = [[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[0,9],[1,0],
        #            [8,0],[9,1],[0,2],[1,3],[2,4],[3,5],[4,6],[5,7],[6,8],[7,9]]

        weight = [(0.25 + 0.75 * self.pd) * random.uniform(0, 1) * 0.3e-3 \
                  for k in range(len(connList))]

        self.netParams.connParams['GPe->GPe'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'GPe'},  # GPe-< GPe
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igege'}  # target synaptic mechanism

        ############################################################################

        # StrD2>GPe connections
        n_strd2 = self.netParams.popParams['StrD2']['numCells']
        n_gpe = self.netParams.popParams['GPe']['numCells']

        self.netParams.connParams['StrD2->GPe'] = {
            'preConds': {'pop': 'StrD2'}, 'postConds': {'pop': 'GPe'}, # StrD2-> GPe
            'connList': [[j, i] for i in range(n_gpe)
                                for j in range(n_strd2)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Istrgpe'}  # target synaptic mechanism

    ################################################################################

    def _gpiConnRules(self, stn_gpi=5, gpe_gpi=2):
        """???."""

        # STN-> GPi connections
        # Five aleatory GPi cells (index i) receive synapse from cells i and i - 1
        n_stn = self.netParams.popParams['STN']['numCells']
        n_gpi = self.netParams.popParams['GPi']['numCells']
        n_neurons = max(n_stn, n_gpi)

        aux = random.sample(range(n_neurons), stn_gpi)
        connList = [[(x - c)%n_stn, x%n_gpi] for x in aux for c in [1, 0]]

        self.netParams.connParams['STN->GPi'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPi'},
            'connList': connList,
            'weight': 0.0645e-3,  # synaptic weight (conductance)
            'delay': 1.5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Isngi'}  # target synaptic mechanism

        ############################################################################

        # GPe-< GPi connections 
        n_gpe = self.netParams.popParams['GPe']['numCells']
        n_gpi = self.netParams.popParams['GPi']['numCells']
        n_neurons = max(n_gpe, n_gpi)

        self.netParams.connParams['GPe->GPi'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'GPi'},
            'connList':
                [[idx%n_gpe, (idx + ncn) % n_gpi] for ncn in range(2, gpe_gpi+1, 2)
                 for idx in range(n_neurons)] + \
                [[(idx + ncn) % n_gpe, idx%n_gpi] for ncn in range(1, gpe_gpi+1, 2)
                 for idx in range(n_neurons)],
            # [ [ idx, (idx+2) % n_neurons ] for idx in range( n_neurons ) ] + \
            # [ [ (idx+1) % n_neurons, idx ] for idx in range( n_neurons ) ],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 3,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igegi'}  # target synaptic mechanism

        ############################################################################

        # StrD1>GPi connections
        n_strd1 = self.netParams.popParams['StrD1']['numCells']
        n_gpi = self.netParams.popParams['GPi']['numCells']

        self.netParams.connParams['StrD1->GPe'] = {
            'preConds': {'pop': 'StrD1'}, 'postConds': {'pop': 'GPi'}, # StrD1-> GPi
            'connList': [[j, i] for i in range(n_gpi)
                         for j in range(n_strd1)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 4,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Istrgpi'}  # target synaptic mechanism

    ################################################################################

    def _stnConnRules(self):
        """???."""

        # GPe-> STN connections 
        n_gpe = self.netParams.popParams['GPe']['numCells']
        n_stn = self.netParams.popParams['STN']['numCells']
        n_neurons = max(n_gpe, n_stn)

        self.netParams.connParams['GPe->STN'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'STN'},  # GPe-< STN
            'connList': [[(i+c) % n_gpe, i%n_stn] for c in [1, 0] for i in range(n_neurons)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 4,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igesn'}  # target synaptic mechanism

        ############################################################################

        # CTX-> STN connections
        n_ctxrs = self.netParams.popParams['CTX_RS']['numCells']
        n_stn = self.netParams.popParams['STN']['numCells']
        n_neurons = max(n_ctxrs, n_stn)

        connList = [[(i+c) % n_ctxrs, i%n_stn] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]

        self.netParams.connParams['CTX->STN'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'STN'},  # CTX-> STN
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 5.9,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icosn,ampa'}  # target synaptic mechanism

        ############################################################################

        # CTX-> STN2 
        connList = [[(i+c) % n_ctxrs, i%n_stn] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.003) * 0.43e-3 for k in range(len(connList))]

        self.netParams.connParams['CTX->STN2'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'STN'},  # CTX-> STN
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 5.9,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icosn,nmda'}  # target synaptic mechanism

    ################################################################################

    def _strConnRules(self, strd2_strd2=4, strd1_strd1=3, gsynmod=1):
        """???."""

        # StrD2-< StrD2 connections
        # Each StrD2 cell receive synapse from 4 aleatory StrD2 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD2']['numCells']

        connList = [[x, i] for i in range(n_neurons)
                           for x in random.sample(
                           [k for k in range(n_neurons) if k != i], strd2_strd2)]

        self.netParams.connParams['StrD2-> StrD2'] = {
            'preConds': {'pop': 'StrD2'}, 'postConds': {'pop': 'StrD2'},  # StrD2-< StrD2
            'connList': connList,
            'weight': 0.1 / 4 * 0.5e-3,  # synaptic weight (conductance) -> mudar essa maluquisse
            'delay': 0,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igabaindr'}  # target synaptic mechanism

        ############################################################################

        # StrD1-< StrD1 connections
        # Each StrD1 cell receive synapse from 3 aleatory StrD1 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD1']['numCells']
        
        connList = [[x, i] for i in range(n_neurons)
                           for x in random.sample(
                           [k for k in range(n_neurons) if k != i], strd1_strd1)]

        self.netParams.connParams['StrD1-> StrD1'] = {
            'preConds': {'pop': 'StrD1'}, 'postConds': {'pop': 'StrD1'},  # StrD1-< StrD1
            'connList': connList,
            'weight': 0.1 / 3 * 0.5e-3,  # synaptic weight (conductance) -> mudar aqui tb
            'delay': 0,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igabadr'}  # target synaptic mechanism

        ############################################################################

        # RS-> StrD1 connections 
        n_ctxrs = self.netParams.popParams['CTX_RS']['numCells']
        n_strd1 = self.netParams.popParams['StrD1']['numCells']
        n_neurons = max(n_ctxrs, n_strd1)

        self.netParams.connParams['RS->StrD1'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'StrD1'},  # RS-> StrD1
            'connList': [[i%n_ctxrs, i%n_strd1] for i in range(n_neurons)],
            'weight': (0.07 - 0.044 * self.pd) * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
            'delay': 5.1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icostr'}  # target synaptic mechanism

        ############################################################################

        # RS-> StrD2 connections 
        n_ctxrs = self.netParams.popParams['CTX_RS']['numCells']
        n_strd2 = self.netParams.popParams['StrD2']['numCells']
        n_neurons = max(n_ctxrs, n_strd2)

        self.netParams.connParams['RS->StrD2'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'StrD2'},  # RS-> StrD2 
            'connList': [[i%n_ctxrs, i%n_strd2] for i in range(n_neurons)],
            'weight': 0.07 * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
            'delay': 5.1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icostr'}  # target synaptic mechanism

    ################################################################################

    def _ctxConnRules(self, rs_fsi=4, fsi_rs=4):
        """???."""

        # RS -> FSI connections
        # Each FSI cell receive synapse from 4 aleatory RS cells
        n_rs = self.netParams.popParams['CTX_RS']['numCells']
        n_fsi = self.netParams.popParams['CTX_FSI']['numCells']

        connList = [[x, i] for i in range(n_fsi)
                    for x in random.sample([k for k in range(n_rs) if k != i],
                                           rs_fsi)]

        self.netParams.connParams['ctx_rs->ctx_fsi'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'CTX_FSI'},  # ctx_rs -> ctx_fsi
            'connList': connList,
            'weight': 0.043e-3,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Iei'}  # target synaptic mechanism

        ############################################################################

        # FSI -> RS connections
        # Each RS cell receive synapse from 4 aleatory FSI cells
        connList = [[x, i] for i in range(n_rs)
                    for x in random.sample([k for k in range(n_fsi) if k != i],
                                           fsi_rs)]

        self.netParams.connParams['ctx_fsi->ctx_rs'] = {
            'preConds': {'pop': 'CTX_FSI'}, 'postConds': {'pop': 'CTX_RS'},  # ctx_fsi -< ctx_rs
            'connList': connList,
            'weight': 0.083e-3,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Iie'}  # target synaptic mechanism

        ############################################################################

        # Th -> RS connections
        n_th = self.netParams.popParams['TH']['numCells']
        n_ctxrs = self.netParams.popParams['CTX_RS']['numCells']
        n_neurons = max(n_th, n_ctxrs)

        self.netParams.connParams['th->ctx_rs'] = {
            'preConds': {'pop': 'TH'}, 'postConds': {'pop': 'CTX_RS'},  # th -> ctx_rs
            'connList': [[i%n_th, i%n_ctxrs] for i in range(n_neurons)],
            'weight': 0.0645e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Ithco'}  # target synaptic mechanism
        
        ############################################################################
