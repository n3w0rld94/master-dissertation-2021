#!/usr/bin/env python
# Last edition: 23 November 2020, Fernando Zuher
# Refactored from: GenericBg.py, 06 May 2020

from network import Network # neural network designed through netpyne, to be optimized

def main():

    simulation_duration = 1000 * 20 # 20 seconds
    net = Network(t_sim = simulation_duration, has_pd = False)
    net.simulate()

if __name__ == '__main__':
    main()
