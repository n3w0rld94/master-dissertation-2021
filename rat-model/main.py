#!/usr/bin/env python
# Last edition: 23 November 2020, Fernando Zuher
# Refactored from: GenericBg.py, 06 May 2020

from network import Network # neural network designed through netpyne, to be optimized

def main():

    simulation_duration = 1000 * 20 # milliseconds

    has_pd = False
    for i in range(1, 11):
        net = Network(t_sim = simulation_duration, has_pd = has_pd, seed=i)
        net.simulate(seed=i, has_pd = has_pd)

    has_pd = True
    for i in range(1, 11):
        net = Network(t_sim = simulation_duration, has_pd = has_pd, seed=i)
        net.simulate(seed=i, has_pd= has_pd)

if __name__ == '__main__':
    main()
