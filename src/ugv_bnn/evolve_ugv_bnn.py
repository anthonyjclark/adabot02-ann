#!/usr/bin/env python3

import cma
from cma.fitness_transformations import EvalParallel

import subprocess as sp
import numpy as np

NI = 5
NH = 3
NO = 3
N = (NI + 1) * NH + (NH + 1) * NO
MAXVAL = 10
SIGMA = 2


def ugv_ann(g):

    # a + (b-a) × x/10
    # a = 0, b = 2
    g_act = int(round(g[0] / MAXVAL * 2))
    nn_shape = '{} {} {} {} '.format(NI, NH, NO, g_act)

    # a + (b-a) × x/10
    # a = -10, b = 10
    weights = g[1:] / MAXVAL * 20 - 10

    cmd = ['../bin/ugv_ann', nn_shape + ' '.join(str(w) for w in weights)]
    timeout = 15

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
        'integer_variables': [0],
        # 'verb_log': ...
    }

    initial_genome = np.random.uniform(0, MAXVAL, N + 1)
    # res = cma.fmin(ugv_ann, initial_genome, SIGMA, cma_options)

    es = cma.CMAEvolutionStrategy(initial_genome, SIGMA, cma_options)

    with EvalParallel(es.popsize + 1) as eval_all:
        while not es.stop():
            X = es.ask()
            es.tell(X, eval_all(ugv_ann, X))
            es.disp()
            # es.logger.add()  # doctest:+ELLIPSIS
