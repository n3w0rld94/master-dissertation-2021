#!/usr/bin/env python
# Last edition: 23 November 2020, Fernando Zuher
# Refactored from: GenericBg.py, 06 May 2020

from network import Network # neural network designed through netpyne, to be optimized

def main():

    tempo_simulacao = 1000 * 20 # 20 segundos
    net = Network(t_sim = tempo_simulacao, has_pd = False)
    net.simulate()

if __name__ == '__main__':
    main()
