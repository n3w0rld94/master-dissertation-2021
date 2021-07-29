"""Stimulation parameters."""

###############################################################################
# NETWORK PARAMETERS
###############################################################################

import random

class Mixin4:

    def buildStimParams(self, amp_th=1.2e-3, amp_gpe=3e-3, amp_gpi=3e-3,
                    amp_stn=0, amp_fs=0, amp_rs=0, amp_dstr=0, amp_istr=0):
        """Parâmetros da corrente elétrica externa aplicada à cada neurônio da
                                                                    região."""

        # flags de silenciamento
        bin_fs=bin_rs=bin_gpe=bin_gpi=bin_stn=bin_dstr=bin_istr=bin_th=0

        ############################################################################

        # FS receives a constant 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_FS'] = {
            'type': 'IClamp', 'delay': 0, 'dur': self.t_sim, 'amp': bin_fs * -1}
        
        self.netParams.stimTargetParams['Input_FS->FS'] = {'source': 'Input_FS',
            'conds': {'pop': 'CTX_FSI'}, 'sec': 'fsi', 'loc': 0}
        
        ############################################################################
        
        # RS receives a constant 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_RS'] = {'type': 'IClamp',
            'delay': 0, 'dur': self.t_sim, 'amp': bin_rs * -1 + amp_rs}

        self.netParams.stimTargetParams['Input_RS->RS'] = {'source': 'Input_RS',
            'conds': {'pop': 'CTX_RS'}, 'sec': 'rs', 'loc': 0}
        
        ############################################################################
        
        # GPe receives a constant 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_GPe'] = {'type': 'IClamp',
            'delay': 0, 'dur': self.t_sim, 'amp': bin_gpe * -1 + amp_gpe}

        self.netParams.stimTargetParams['Input_GPe->GPe'] = {'source': 'Input_GPe',
            'conds': {'pop': 'GPe'}, 'sec': 'GPe', 'loc': 0}

        ############################################################################

        # GPi receives a constant 3 density current
        self.netParams.stimSourceParams['Input_GPi'] = {'type': 'IClamp',
            'delay': 0, 'dur': self.t_sim, 'amp': bin_gpi * -1 + amp_gpi}

        self.netParams.stimTargetParams['Input_GPi->GPi'] = {'source': 'Input_GPi',
            'conds': {'pop': 'GPi'}, 'sec': 'GPi', 'loc': 0}

        ############################################################################

        # STN receives a constant 3 density current or 1 during cortical stimulation
        
        self.netParams.stimSourceParams['Input_STN'] = {'type': 'IClamp',
            'delay': 0, 'dur': self.t_sim, 'amp': bin_stn * -1 + amp_stn}
        
        self.netParams.stimTargetParams['Input_STN->STN'] = {'source': 'Input_STN',
            'conds': {'pop': 'STN'}, 'sec': 'STN', 'loc': 0}

        ############################################################################

        # dStr receives a constant 3 density current
        self.netParams.stimSourceParams['Input_StrD1'] = {'type': 'IClamp',
            'delay': 0, 'dur': self.t_sim, 'amp': bin_dstr * -1 + amp_dstr}

        self.netParams.stimTargetParams['Input_StrD1->StrD1'] = {
            'source': 'Input_StrD1', 'conds': {'pop': 'StrD1'}, 'sec': 'StrD1',
            'loc': 0}

        ############################################################################

        # iStr receives a constant 3 density current
        self.netParams.stimSourceParams['Input_StrD2'] = {'type': 'IClamp', 
            'delay': 0, 'dur': self.t_sim, 'amp': bin_istr * -1 + amp_istr}

        self.netParams.stimTargetParams['Input_StrD2->StrD2'] = {
            'source': 'Input_StrD2', 'conds': {'pop': 'StrD2'}, 'sec': 'StrD2',
            'loc': 0}

        ############################################################################

        # Thalamus receives a constant 1.2 density current
        self.netParams.stimSourceParams['Input_th'] = {'type': 'IClamp',
            'delay': 0, 'dur': self.t_sim, 'amp': bin_th * -1 + amp_th}

        self.netParams.stimTargetParams['Input_th->TH'] = {'source': 'Input_th',
            'conds': {'pop': 'TH'}, 'sec': 'th', 'loc': 0}

        ############################################################################
