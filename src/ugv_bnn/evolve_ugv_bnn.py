#!/usr/bin/env python3

import cma
from cma.fitness_transformations import EvalParallel

import numpy as np

import subprocess as sp
from math import pi, floor, log10
import argparse


MAX_G_VAL = 10
SIGMA = 2
SIM_MAX_DURATION = 30
NUM_OBSTACLES = 0
OBSTACLE_SEED = 0



# Names are just for documentation
genome_to_args_map = [
    {'name': 'wheel_base'   , 'minval':  0.08 , 'maxval': 0.16 , 'T': float},
    {'name': 'track_width'  , 'minval':  0.08 , 'maxval': 0.16 , 'T': float},
    {'name': 'wheel_radius' , 'minval':  0.02 , 'maxval': 0.03 , 'T': float},
    {'name': 'weg_count'    , 'minval':  0    , 'maxval': 7.99 , 'T': int  },
    {'name': 'ACT'          , 'minval':  0    , 'maxval': 2.99 , 'T': int  },
]
genome_weights_to_args_maps = {
     'name': 'weight'       , 'minval': -4    , 'maxval': 4    , 'T': float}



def range_transform(x, a, b, c, d):
    # [a; b] --> [b; c]
    return (x - a) * (d - c) / (b - a) + c



def round_to_n_sigs(x, n):
    return round(x, -int(floor(log10(abs(x)))) + (n - 1))



def genome_to_args(genome):

    # TIME_STOP, num_obstacles, obstacle_seed
    args = [str(SIM_MAX_DURATION), str(NUM_OBSTACLES), str(OBSTACLE_SEED)]

    for m, gval in zip(genome_to_args_map, genome):
        argval = m['T'](range_transform(gval, 0, MAX_G_VAL, m['minval'], m['maxval']))
        args.append(str(round_to_n_sigs(argval, 4)))

    # NI, NO
    NI = 3
    NO = 3
    N = (NI + 1) * NO

    # weights
    m = genome_weights_to_args_maps
    for gval in g[len(genome_to_args):]:
        argval = m['T'](genome_transform(gval, m['minval'], m['maxval'], MAXVAL))
        args.append(str(argval))

    return ' '.join(args)



def ugv_fitness_fcn(genome):
    return ugv_sim(genome_to_args(genome))



def ugv_sim(args, print_command=False):

    cmd = ['../bin/ugv_bnn', args]
    timeout = SIM_MAX_DURATION * 1.2

    if print_command:
        print(cmd)

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
            fit3 = time_remaining / SIM_MAX_DURATION

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


def evolve(initial_genome, seed):
    cma_options = {
        'seed': seed,
        'bounds': [0, MAX_G_VAL],
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
            es.tell(X, eval_all(ugv_fitness_fcn, X))
            es.disp()
            es.logger.add()


if __name__ == '__main__':

    argparser = argparse.ArgumentParser(description='Launch an experiment.')
    argparser.add_argument('--seed', type=int, help='Seed for the random number generator.')
    argparser.add_argument('--args', type=str, help='A UGV args string to evaluate.')
    prog_args = argparser.parse_args()

    default_genome = [range_transform(m['default'], m['minval'], m['maxval'], 0, MAX_G_VAL)
        for m in genome_to_args_map]

    if prog_args.seed != None:
        evolve(default_genome, prog_args.seed)
    elif prog_args.args != None:
        print('Fitness', ugv_sim(prog_args.args, print_command=True))
    else:
        print('Fitness', ugv_sim(genome_to_args(default_genome), print_command=True))
