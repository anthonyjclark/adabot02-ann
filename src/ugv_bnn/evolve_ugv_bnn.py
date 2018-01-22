#!/usr/bin/env python3

import cma
from cma.fitness_transformations import EvalParallel

import numpy as np

import subprocess as sp
from math import pi
import sys


MAXVAL = 10
SIGMA = 2

SIM_TIME = 30


def genome_to_args(g):

    # a + (b-a) × x/10

    # TIME_STOP, num_obstacles, obstacle_seed
    args = [str(SIM_TIME), '0', '0']

    # wheel_base 10cm, 8cm to 16cm
    args.append(str(0.08 + (0.16 - 0.08) * g[0] / MAXVAL))

    # track_width 12cm, 8cm to 16cm
    args.append(str(0.08 + (0.16 - 0.08) * g[1] / MAXVAL))

    # wheel_radius 2.5cm, 2cm to 3cm
    args.append(str(0.02 + (0.03 - 0.02) * g[2] / MAXVAL))

    # weg_count 3, 0 to 7
    args.append(str(int(round(7 * g[3] / MAXVAL))))

    # NI, NO, ACT
    NI = 3
    NO = 3
    args.append(str(NI))
    args.append(str(NO))
    args.append(str(int(round(g[4] / MAXVAL * 2))))

    # weights
    N = (NI + 1) * NO
    for i in range(5, 5 + N):
        args.append(str(-4 + 8 * g[i] / MAXVAL))

    return ugv_bnn(' '.join(args))


def ugv_bnn(args):

    cmd = ['./ugv_bnn', args]
    timeout = SIM_TIME * 1.2

    for attempt in range(3):
        try:
            result = sp.run(cmd, stdout=sp.PIPE, stderr=sp.PIPE, timeout=timeout, check=True)

            # target_idx,target_dist,time_remaining
            sim_output = str(result.stdout, 'utf-8').split(',')
            targets_reached = float(sim_output[0])
            dist_to_next = float(sim_output[1])
            time_remaining = float(sim_output[2])

            # Higher is better
            fit1 = 2 * targets_reached

            # Lower is better
            fit2 = min(dist_to_next, 1.5) / 1.5

            # Higher is better
            fit3 = time_remaining / SIM_TIME

            fitness = -fit1 + fit2 - fit3

            break

        except sp.TimeoutExpired as e:
            print('\n\n')
            print(e)
            print('\n\n')
            fitness = 3

        except sp.CalledProcessError as e:
            print('\n\n')
            print(e)
            print('\n\n')
            fitness = 3

    return fitness


def evolve(initial_genome):
    cma_options = {
        'seed': 0,
        'bounds': [0, MAXVAL],
        # 'maxiter': 100,
        'timeout': 2 * 60**2,
        # 'verb_plot': ...,
        'integer_variables': [3, 4],
        # 'verb_log': ...
    }

    es = cma.CMAEvolutionStrategy(initial_genome, SIGMA, cma_options)

    with EvalParallel() as eval_all:
        while not es.stop():
            X = es.ask()
            es.tell(X, eval_all(genome_to_args, X))
            es.disp()
            es.logger.add()


if __name__ == '__main__':

    if len(sys.argv) > 1:
        print(ugv_bnn(sys.argv[1]))
    else:
        initial_genome = np.random.uniform(0, MAXVAL, 17)
        evolve(initial_genome)
