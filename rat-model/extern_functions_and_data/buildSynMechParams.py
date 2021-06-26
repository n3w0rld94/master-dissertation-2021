"""Synaptic mechanism parameters."""

###############################################################################
# NETWORK PARAMETERS
###############################################################################

class Mixin3:

    def buildSynMechParams(self):
        """Configura mecanismos das sinapses."""

        # TH
        self.netParams.synMechParams['Igith'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': -85}  # gpi -<th
        
        # GPe #####################################################################
        self.netParams.synMechParams['Insge,ampa'] = {
            'mod': 'Exp2Syn', 'tau1': 0.4, 'tau2': 2.5, 'e': 0}  # stn -> gpe
        
        self.netParams.synMechParams['Insge,nmda'] = {
            'mod': 'Exp2Syn', 'tau1': 2, 'tau2': 67, 'e': 0}  # stn -> gpe
        
        self.netParams.synMechParams['Igege'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': -85}  # gpe -< gpe

        self.netParams.synMechParams['Istrgpe'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': -85}  # D2 -> gpe
        ###########################################################################

        # GPi #####################################################################
        self.netParams.synMechParams['Igegi'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': -85}  # gpe -< gp

        self.netParams.synMechParams['Isngi'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': 0}  # stn -> gpi

        self.netParams.synMechParams['Istrgpi'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': -85}  # D1 -> gpi
        ###########################################################################

        # STN #####################################################################
        self.netParams.synMechParams['Igesn'] = {
            'mod': 'Exp2Syn', 'tau1': 0.4, 'tau2': 7.7, 'e': -85}  # gpe -< stn

        self.netParams.synMechParams['Icosn,ampa'] = {
            'mod': 'Exp2Syn', 'tau1': 0.5, 'tau2': 2.49, 'e': 0}  # ctx -> gpe

        self.netParams.synMechParams['Icosn,nmda'] = {
            'mod': 'Exp2Syn', 'tau1': 2, 'tau2': 90, 'e': 0}  # ctx -> gpe
        ###########################################################################

        # Str #####################################################################
        self.netParams.synMechParams['Igabadr'] = {
            'mod': 'Exp2Syn', 'tau1': 0.1, 'tau2': 13, 'e': -80}  # str -< str

        self.netParams.synMechParams['Igabaindr'] = {
            'mod': 'Exp2Syn', 'tau1': 0.1, 'tau2': 13, 'e': -80}  # str -< str

        self.netParams.synMechParams['Icostr'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': 0}  # ctx -> str
        ###########################################################################
        
        # CTX #####################################################################
        self.netParams.synMechParams['Iei'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': 0}  # rs->fsi

        self.netParams.synMechParams['Iie'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': -85}  # fsi<-rs

        self.netParams.synMechParams['Ithco'] = {
            'mod': 'Exp2Syn', 'tau1': 5, 'tau2': 5, 'e': 0}  # th->rs