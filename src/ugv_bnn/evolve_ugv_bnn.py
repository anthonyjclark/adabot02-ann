#!/usr/bin/env python3

import cma
from cma.fitness_transformations import EvalParallel

import numpy as np

import subprocess as sp
from math import pi
import sys


MAXVAL = 10
SIGMA = 2


def genome_to_args(g):

    # a + (b-a) × x/10

    # TIME_STOP, num_obstacles, obstacle_seed
    args = ['30.0', '0', '0']

    # wheel_base 10cm, 8cm to 16cm
    args.append(str(0.08 + (0.16 - 0.08) * g[0] / 10))

    # track_width 12cm, 8cm to 16cm
    args.append(str(0.08 + (0.16 - 0.08) * g[1] / 10))

    # wheel_radius 2.5cm, 2cm to 3cm
    args.append(str(0.02 + (0.03 - 0.02) * g[2] / 10))

    # weg_count 3, 0 to 7
    args.append(str(int(round(7 * g[3] / 10))))

    # NI, NO, ACT
    NI = 3
    NO = 3
    args.append(str(NI))
    args.append(str(NO))
    args.append(str(int(round(g[4] / MAXVAL * 2))))

    # weights
    N = (NI + 1) * NO
    for i in range(5, 5 + N):
        args.append(str(g[i]/10))

    return ugv_bnn(' '.join(args))


def ugv_bnn(args):

    cmd = ['../bin/ugv_bnn', args]
    timeout = 40

    for attempt in range(3):
        try:
            result = sp.run(cmd, stdout=sp.PIPE, stderr=sp.PIPE, timeout=timeout, check=True)

            # 'Targets reached : 0\nDist to next    : 0.713236\nTime remaining  : 0\n'
            sim_output = str(result.stdout, 'utf-8').split(',')
            targets_reached = float(sim_output[0])
            dist_to_next = float(sim_output[1])
            time_remaining = float(sim_output[2])

            # Higher is better
            fit1 = 2 * targets_reached

            # Lower is better
            fit2 = min(dist_to_next, 1.5) / 1.5

            # Higher is better
            fit3 = time_remaining / 10

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
        'timeout': 1 * 60**2,
        # 'verb_plot': ...,
        'integer_variables': [3, 4],
        # 'verb_log': ...
    }

    # N = 15
    # initial_genome = np.random.uniform(0, MAXVAL, N)

    es = cma.CMAEvolutionStrategy(initial_genome, SIGMA, cma_options)

    with EvalParallel(es.popsize + 1) as eval_all:
        while not es.stop():
            X = es.ask()
            es.tell(X, eval_all(genome_to_args, X))
            es.disp()
            # es.logger.add()


if __name__ == '__main__':

    if len(sys.argv) > 1:
        print(ugv_bnn(sys.argv[1]))
    else:
        initial_genome = np.random.uniform(0, MAXVAL, 17)
        evolve(initial_genome)


"""
30.0 0.1 0.12 0.025 3 0.4 -10 -10 0.18 -0.18 10 -10 0.09 -10 10 -0.09
-3.0743666666666667
TIME_STOP 30
wheel_base 0.1
track_width 0.12
wheel_radius 0.025
weg_count 3
weg_extension_percent 0.4
forward_left -10
forward_right -10
forward_to_left_lo 0.18
forward_to_right_hi -0.18
left_left 10
left_right -10
left_to_forward_hi 0.09
right_left -10
right_right 10
right_to_forward_lo -0.09
Targets reached : 2
Dist to next    : 1.38845
Time remaining  : 0


30.0 0.1 0.12 0.025 3 0.4 -10 -10 0.18849555921538758 -0.1884955592153874 10.0 -10.0 0.09424777960769379 -10.0 10.0 -0.0942477796076937
-1.4723493333333333
TIME_STOP 30
wheel_base 0.1
track_width 0.12
wheel_radius 0.025
weg_count 3
weg_extension_percent 0.4
forward_left -10
forward_right -10
forward_to_left_lo 0.188496
forward_to_right_hi -0.188496
left_left 10
left_right -10
left_to_forward_hi 0.0942478
right_left -10
right_right 10
right_to_forward_lo -0.0942478
Targets reached : 1
Dist to next    : 0.791476
Time remaining  : 0
"""
