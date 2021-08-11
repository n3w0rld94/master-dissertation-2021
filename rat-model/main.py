#!/usr/bin/env python
# Last edition: 23 November 2020, Fernando Zuher
# Refactored from: GenericBg.py, 06 May 2020

from network import Network # neural network designed through netpyne, to be optimized

def main():

    simulation_duration = 1000 * 1 # milliseconds 

    # net = Network(t_sim = simulation_duration, has_pd = False, seed=3)
    # net.simulate(seed=3, has_pd = False)

    net = Network(t_sim = simulation_duration, has_pd = True, seed=3)
    net.simulate(seed=3, has_pd= True)

if __name__ == '__main__':
    main()
