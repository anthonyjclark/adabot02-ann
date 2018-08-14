#!/usr/bin/env python3

import cma
from cma.fitness_transformations import EvalParallel

import random
import subprocess as sp
from math import pi, floor, log10
import argparse



def range_transform(x, a, b, c, d):
    # [a; b] --> [b; c]
    return (x - a) * (d - c) / (b - a) + c



def round_to_n_sigs(x, n):
    if x == 0:
        return 0
    else:
        return round(x, -int(floor(log10(abs(x)))) + (n - 1))




NI = 3
NO = 3
N = (NI + 1) * NO
random.seed(0)
default_weights = [random.uniform(-4, 4) for i in range(N)]


# Names are just for documentation
genome_to_args_map = [
    {'name': 'wheel_base'             , 'default': 0.08, 'minval': 0.08, 'maxval': 0.16, 'T': float},
    {'name': 'track_width'            , 'default': 0.12, 'minval': 0.08, 'maxval': 0.16, 'T': float},
    {'name': 'wheel_radius'           , 'default': 0.02, 'minval': 0.02, 'maxval': 0.03, 'T': float},
    {'name': 'weg_count'              , 'default': 3   , 'minval': 0   , 'maxval': 7.99, 'T': int  },
    {'name': 'ACT'                    , 'default': 0   , 'minval': 0   , 'maxval': 2.99, 'T': int  },
]
genome_to_args_map.extend([{
    'name': 'weight', 'default': f, 'minval': -4, 'maxval': 4, 'T': float} for f in default_weights])



def genome_to_args(genome, trial_num=0):

    # TIME_STOP, num_obstacles, obstacle_seed
    args = [str(SIM_MAX_DURATION), str(NUM_OBSTACLES), str(OBSTACLE_SEED + trial_num * 100)]

    for m, gval in zip(genome_to_args_map, genome):
        argval = m['T'](range_transform(gval, 0, MAX_G_VAL, m['minval'], m['maxval']))
        args.append(str(round_to_n_sigs(argval, 4)))

    return ' '.join(args)



def ugv_fitness_fcn(genome):
    return sum(ugv_sim(genome_to_args(genome, tn)) for tn in range(NUM_EVALUATIONS)) / NUM_EVALUATIONS



def ugv_sim(args, print_command=False):

    cmd = ['../bin/ugv_bnn_twist', args]
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
    random.seed(seed)
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
    argparser.add_argument('--obst', type=int, help='Number of obstacles to generate.')
    argparser.add_argument('--evals', type=int, help='Number of evaluations to perform.')

    argparser.add_argument('--args', type=str, help='A UGV args string to evaluate.')
    argparser.add_argument('--genome', type=str, help='Evaluate the given genome.')

    prog_args = argparser.parse_args()

    MAX_G_VAL = 10
    SIGMA = 2

    SIM_MAX_DURATION = 30
    NUM_OBSTACLES = prog_args.obst if prog_args.obst else 0
    OBSTACLE_SEED = prog_args.seed if prog_args.seed else 0
    NUM_EVALUATIONS = prog_args.evals if prog_args.evals else 1

    default_genome = [range_transform(m['default'], m['minval'], m['maxval'], 0, MAX_G_VAL)
        for m in genome_to_args_map]

    if prog_args.seed != None:
        evolve(default_genome, prog_args.seed)
    elif prog_args.args != None:
        print('Fitness', ugv_sim(prog_args.args, print_command=True))
    elif prog_args.genome != None:
        given_genome = [float(g) for g in prog_args.genome.split(' ')]
        print('Fitness', ugv_sim(genome_to_args(given_genome), print_command=True))
    else:
        print('Fitness', ugv_sim(genome_to_args(default_genome), print_command=True))
