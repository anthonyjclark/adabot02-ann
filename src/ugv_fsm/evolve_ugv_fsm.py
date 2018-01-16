#!/usr/bin/env python3

import cma
from cma.fitness_transformations import EvalParallel

import numpy as np

import subprocess as sp
from math import pi


MAXVAL = 10
SIGMA = 2

def ugv_ann(g):

    # a + (b-a) × x/10
    # "10.0 0.1 0.12 0.03 3 0.4 -10 -10 0.18 6 -6 -0.18 10 -10 -6 0.09 -10 10 -0.09 6"

    # Simulation duration
    args = '10.0'

    # wheel_base 10cm, 8cm to 16cm
    args += str(0.08 + (0.16 - 0.08) * g[0] / 10)

    # track_width 12cm, 8cm to 16cm
    args += str(0.08 + (0.16 - 0.08) * g[1] / 10)

    # wheel_radius 2.5cm, 2cm to 3cm
    args += str(0.02 + (0.03 - 0.02) * g[2] / 10)

    # weg_count 3, 0 to 7
    args += str(round(0 + (7 - 0) * g[3] / 10))

    # weg_extension_percent 1, 0 to 1
    args += str(g[18]/10)

    # forward_left -10, -10 to 0
    args += str(-g[4])

    # forward_right -10
    args += str(-g[5])

    # forward_to_left_lo 10deg, -pi to pi
    args += str(-pi + (pi - (-pi)) * g[6] / 10)

    # forward_to_left_hi 2pi
    args += str(-pi + (pi - (-pi)) * g[7] / 10)

    # forward_to_right_lo -2pi
    args += str(-pi + (pi - (-pi)) * g[8] / 10)

    # forward_to_right_hi -10deg
    args += str(-pi + (pi - (-pi)) * g[9] / 10)

    # left_left 10
    args += str(-10 + (10 - (-10)) * g[10] / 10)

    # left_right -10
    args += str(-10 + (10 - (-10)) * g[10] / 10)

    # left_to_forward_lo -2pi
    args += str(-pi + (pi - (-pi)) * g[12] / 10)

    # left_to_forward_hi 5deg
    args += str(-pi + (pi - (-pi)) * g[13] / 10)

    # right_left -10
    args += str(-10 + (10 - (-10)) * g[10] / 10)

    # right_right 10
    args += str(-10 + (10 - (-10)) * g[10] / 10)

    # right_to_forward_lo -5deg
    args += str(-pi + (pi - (-pi)) * g[16] / 10)

    # right_to_forward_hi 2pi
    args += str(-pi + (pi - (-pi)) * g[17] / 10)


    cmd = ['../bin/ugv_fsm', args]
    timeout = 30

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


if __name__ == '__main__':
    cma_options = {
        'seed': 0,
        'bounds': [0, MAXVAL],
        # 'maxiter': 100,
        'timeout': 1 * 60**2,
        # 'verb_plot': ...,
        'integer_variables': [3],
        # 'verb_log': ...
    }

    N = 19
    initial_genome = np.random.uniform(0, MAXVAL, N)

    es = cma.CMAEvolutionStrategy(initial_genome, SIGMA, cma_options)

    with EvalParallel(es.popsize + 1) as eval_all:
        while not es.stop():
            X = es.ask()
            es.tell(X, eval_all(ugv_ann, X))
            es.disp()
            # es.logger.add()
