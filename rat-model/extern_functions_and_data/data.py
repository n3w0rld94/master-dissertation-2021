""" """

#import random
#random.seed(1)

#TODO This is the Kumaravelu model class. Verify if all possible configurations are flexible, i.e., can be configured when a new object is instantiated

# Configuration 1
# region AP ML DV
# M1 10 6.5 14.4
# GPi 8 3.5 7.8
# GPe 8 5.2 8.8
# Put 8.5 6.5 11.5
# VL 5.5 3.7 10.5
# VPL 4.5 4.3 9.2
# STN 5.5 3.7 7.6

# Configuration 2
# M1 10 6.5 14.4
# S1 8 5.2 15.6
# Put 8.5 6.5 11.5
# VL 5.5 3.7 10.5
# VPL 4.5 4.3 9.2
# STN 5.5 3.7 7.6

# Simulação do LFP (local field potential) e não apenas dos Spikes
# Spike = potencial de ação de um único neurônio, disparo neuronal
  # Spike é um fenômeno de um neurônio. Não de uma população.
# LFP = sinal proporcional aos spikes de uma população neuronal.
# 
# 

# 8 eletrodos nas 8 regiões do cérebro do sagui.
electrodesPos = [ [5000, 4900, 4000],  # StrD1
                  [5000, 4900, 4000],  # StrD2
                  [1000, 2600, 1800],  # TH
                  [4500, 1200, 1000],  # GPi
                  [4500, 2200, 2700],  # GPe
                  [6500, 7800, 4000],  # CtxRS
                  [6500, 7800, 4000],  # CtxFSI
                  [2000, 1200, 1200] ] # STN

# número de eletrodos por região
nelec = 1
#electrodesPos = [ *list( np.random.normal( [5000, 4900, 4000], 500, (nelec,3) ) ),  # StrD1
#                  *list( np.random.normal( [5000, 4900, 4000], 500, (nelec,3) ) ),  # StrD2
#                  *list( np.random.normal( [1000, 2600, 1800], 500, (nelec,3) ) ),  # TH
#                  *list( np.random.normal( [4500, 1200, 1000], 500, (nelec,3) ) ),  # GPi
#                  *list( np.random.normal( [4500, 2200, 2700], 500, (nelec,3) ) ),  # GPe
#                  *list( np.random.normal( [6500, 7800, 4000], 500, (nelec,3) ) ),  # CtxRS
#                  *list( np.random.normal( [6500, 7800, 4000], 500, (nelec,3) ) ),  # CtxFSI
#                  *list( np.random.normal( [2000, 1200, 1200], 500, (nelec,3) ) ) ] # STN

###################### Health / Parkinson ###################
# RS-> StrD1 connections
# GPe-< GPe connections
# Str.mod
#############################################################