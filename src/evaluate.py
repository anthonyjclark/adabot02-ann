#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
from subprocess import PIPE, run


NUM_BNN_PARAMS = 17
NUM_FSM_PARAMS = 15


def read_individuals_from_file(filename, num_individuals):

    # with filename.open(mode='rb') as data_file:

    #     # Jump to the second last byte
    #     data_file.seek(-2, os.SEEK_END)

    #     # Read some number of end-of-lines
    #     for _ in range(num_individuals):

    #         # Jump back one line
    #         while data_file.read(1) != b"\n":
    #             data_file.seek(-2, os.SEEK_CUR)

    #         # Jump back before newline that broke the while-loop
    #         data_file.seek(-2, os.SEEK_CUR)

    #     # Read lines, but skip first fraction line (artifact of final seek)
    #     lines = data_file.readlines()[1:]

    # if len(lines) != num_individuals:
    #     print(f'Error: Read {len(lines)} lines, but expected {num_individuals}')

    # return lines

    with filename.open(mode='r') as data_file:
        all_individuals = [l.strip().split() for l in data_file.readlines()[1:]]

    all_individuals.sort(key=lambda ind: float(ind[4]))

    return all_individuals[:num_individuals]


def get_args_from_individuals(individuals, exp_name, max_time, num_obst):

    if 'fsm' in exp_name:
        expected_genome_len = NUM_FSM_PARAMS
        evo_script = 'ugv_fsm/evolve_ugv_fsm.py'
    elif 'twist' in exp_name:
        expected_genome_len = NUM_BNN_PARAMS
        evo_script = 'ugv_bnn_twist/evolve_ugv_bnn_twist.py'
    else:
        expected_genome_len = NUM_BNN_PARAMS
        evo_script = 'ugv_bnn/evolve_ugv_bnn.py'

    eval_inputs = []
    for ind in individuals:

        # Params: iter, evals, sigma, 0, fitness, xbest
        # decoded = ind.decode('utf-8').strip().split()

        # genome = decoded[5:]
        genome = ind[5:]
        if len(genome) != expected_genome_len:
            print(f'Error: Found {len(genome)} genes, but expected {expected_genome_len}')

        run_cmd = [evo_script, '--genome_to_args', ' '.join(genome)]
        proc_output = run(run_cmd, stdout=PIPE, check=True).stdout.decode('utf-8').strip()

        eval_input = proc_output.split()
        eval_input[0] = max_time
        eval_input[1] = num_obst
        eval_inputs.append(eval_input)

    return eval_inputs


def evaluate(args, num_trials, exp_name):
    if 'fsm' in exp_name:
        eval_bin = 'bin/ugv_fsm_eval'
    elif 'twist' in exp_name:
        eval_bin = 'bin/ugv_bnn_twist_eval'
    else:
        eval_bin = 'bin/ugv_bnn_eval'

    for arg in args:
        for trial in range(num_trials):
            arg[2] = str(trial)
            run_cmd = [eval_bin, ' '.join(arg)]
            proc_output = run(run_cmd, stdout=PIPE, check=True).stdout.decode('utf-8').strip()
            print(','.join(arg) + ',' + proc_output)


if __name__ == '__main__':

    argparser = argparse.ArgumentParser(description='Evaluate an experiment.')
    argparser.add_argument('experiment', type=str, help='Director of experiment.')
    argparser.add_argument('time', type=str, help='Simulation time.')
    argparser.add_argument('obst', type=str, help='Number of obstacles to (attempt to) generate.')
    argparser.add_argument('trials', type=int, help='Number of trials per evaluation.')
    argparser.add_argument('genomes', type=int, help='Number of evaluations to perform.')

    prog_args = argparser.parse_args()

    exp_files = Path(prog_args.experiment).glob('**/outcmaesxrecentbest.dat')
    for exp_file in sorted(exp_files):
        individuals = read_individuals_from_file(exp_file, prog_args.genomes)
        eval_args = get_args_from_individuals(individuals, prog_args.experiment, prog_args.time, prog_args.obst)

        print(f'#{exp_file}')
        evaluate(eval_args, prog_args.trials, prog_args.experiment)
