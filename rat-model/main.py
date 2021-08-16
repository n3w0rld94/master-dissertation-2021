#!/usr/bin/env python
# Last edition: 23 November 2020, Fernando Zuher
# Refactored from: GenericBg.py, 06 May 2020

from network import Network # neural network designed through netpyne, to be optimized

def main():

    simulation_duration = 1000 * 200 # milliseconds 

    # net = Network(t_sim = simulation_duration, has_pd = False, seed=3)
    # net.simulate(seed=3, has_pd = False)

    for seed in range(3, 9, 2):
        net = Network(t_sim = simulation_duration, has_pd = False, seed=seed)
        net.simulate(seed=seed, has_pd=False)

    seed = None

    for seed in range(3, 9, 2):
        net = Network(t_sim = simulation_duration, has_pd = True, seed=seed)
        net.simulate(seed=seed, has_pd= True)

if __name__ == '__main__':
    main()
