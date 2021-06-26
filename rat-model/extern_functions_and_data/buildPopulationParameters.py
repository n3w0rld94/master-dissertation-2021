"""Population parameters."""

###############################################################################
# NETWORK PARAMETERS
###############################################################################

class Mixin1:

    def buildPopulationParameters(self, n_strd1=10, n_strd2=10, n_th=10,
                                  n_gpi=10, n_gpe=10, n_rs=10, n_fsi=10, n_stn=10):
        """Implementa os parâmetros da população de neurônios"""

        # Volume cerebral que vai conter a população neuronal
        self.netParams.sizeX = 7500  # x-dimension (horizontal length) size in um
        self.netParams.sizeY = 8800  # y-dimension (vertical height or cortical
                                     # depth) size in um
        self.netParams.sizeZ = 5000  # z-dimension (horizontal length) size in um

        # Volume occupied by each population can be customized (xRange, yRange
        # and zRange) in um.
        #
        # xRange or xnormRange - Range of neuron positions in x-axis
        # (horizontal length), specified 2-element list [min, max].
        #
        # zRange or znormRange - Range of neuron positions in z-axis 
        # (horizontal depth).
        #
        # Establishing 2000 um as a standard coordinate span.

        # 8 regiões do cérebro sendo representadas
        self.netParams.popParams['StrD1'] = {
            'cellModel': 'StrD1', 'cellType': 'StrD1', 'numCells': n_strd1,
            'xRange': [4000, 6000], 'yRange': [3900, 5900], 'zRange': [3000, 5000]}

        self.netParams.popParams['StrD2'] = {
            'cellModel': 'StrD2', 'cellType': 'StrD2', 'numCells': n_strd2,
            'xRange': [4000, 6000], 'yRange': [3900, 5900], 'zRange': [3000, 5000]}

        # considering VPL coordinates
        self.netParams.popParams['TH'] = {
            'cellModel': 'TH', 'cellType': 'Thal', 'numCells': n_th,
            'xRange': [0, 2000], 'yRange': [1600, 3600], 'zRange': [800, 2800]}

        self.netParams.popParams['GPi'] = {
            'cellModel': 'GPi', 'cellType': 'GPi', 'numCells': n_gpi,
            'xRange': [3500, 5500], 'yRange': [200, 2200], 'zRange': [0, 2000]}

        self.netParams.popParams['GPe'] = {
            'cellModel': 'GPe', 'cellType': 'GPe', 'numCells': n_gpe,
            'xRange': [3500, 5500], 'yRange': [1200, 3200], 'zRange': [1700, 3700]}

        # considering M1
        self.netParams.popParams['CTX_RS'] = {
            'cellModel': 'CTX_RS', 'cellType': 'CTX_RS', 'numCells': n_rs,
            'xRange': [5500, 7500], 'yRange': [6800, 8800], 'zRange': [3000, 5000]}

        self.netParams.popParams['CTX_FSI'] = {
            'cellModel': 'CTX_FSI', 'cellType': 'CTX_FSI', 'numCells': n_fsi,
            'xRange': [5500, 7500], 'yRange': [6800, 8800], 'zRange': [3000, 5000]}

        self.netParams.popParams['STN'] = {
            'cellModel': 'STN', 'cellType': 'STN', 'numCells': n_stn,
            'xRange': [1000, 3000], 'yRange': [0, 2000], 'zRange': [200, 2200]}
