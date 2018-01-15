#!/usr/bin/env python3

import cma
import subprocess as sp
import numpy as np

NI = 2
NH = 3
NO = 1
N = (NI + 1) * NH + (NH + 1) * NO
MAXVAL = 10
SIGMA = 2


def xor_obj_fcn(g):

    # a + (b-a) × x/10
    # a = 0, b = 2
    g_act = int(round(g[0] / MAXVAL * 2))
    nn_shape = '{} {} {} {} '.format(NI, NH, NO, g_act)

    # a + (b-a) × x/10
    # a = -10, b = 10
    weights = g[1:] / MAXVAL * 20 - 10

    # print(nn_shape + ' '.join(str(w) for w in weights))
    # return sum(weights)


    cmd = ['./xor_obj_fcn', nn_shape + ' '.join(str(w) for w in weights)]
    timeout = 0.5

    try:
        result = sp.run(cmd, stdout=sp.PIPE, stderr=sp.PIPE, timeout=timeout, check=True)
        fitness = float(str(result.stdout, 'utf-8'))

    except sp.TimeoutExpired as e:
        raise e
    except sp.CalledProcessError as e:
        print('\n\n')
        print(e)
        print('\n\n')
        raise e

    return fitness


if __name__ == '__main__':
    # fmin(objective_function, x0, sigma0, options={'ftarget': 1e-5})
    cma_options = {
        'seed': 0,
        'bounds': [0, MAXVAL],
        # 'maxiter': ...,
        # 'timeout': 5,
        # 'verb_plot': ...,
        'integer_variables': [0],
        # 'verb_log': ...
    }

    initial_genome = np.random.uniform(0, MAXVAL, N + 1)
    res = cma.fmin(xor_obj_fcn, initial_genome, SIGMA, cma_options)


# CMA_on
#     1  # multiplier for all covariance matrix updates

# CMA_stds
#     None  # multipliers for sigma0 in each coordinate, not represented in
#       C, makes scaling_of_variables obsolete

# CMA_sampler_options
#     {}  # options passed to `CMA_sampler` class init as keyword arguments

# mindx
#     0  #v minimal std in any arbitrary direction, cave interference with tol*

# CMA_const_trace
#     False  # normalize trace, value CMA_const_trace=2 normalizes sum log
#       eigenvalues to zero

# CMA_mirrors
#     popsize < 6  # values <0.5 are interpreted as fraction, values >1 as
#       numbers (rounded), otherwise about 0.16 is used

# verb_filenameprefix
#     outcmaes  # output filenames prefix

# verb_time
#     True  #v output timings on console

# CMA_dampsvec_fac
#     np.Inf  # tentative and subject to changes, 0.5 would be a "default"
#       damping for sigma vector update

# tolx
#     1e-11  #v termination criterion: tolerance in x-changes

# vv
#     0  #? versatile variable for hacking purposes, value found in
#       self.opts["vv"]

# updatecovwait
#     None  #v number of iterations without distribution update, name is
#       subject to future changes

# CMA_rankmu
#     1.0  # multiplier for rank-mu update learning rate of covariance matrix

# tolupsigma
#     1e20  #v sigma/sigma0 > tolupsigma * max(eivenvals(C)**0.5) indicates
#       "creeping behavior" with usually minor improvements

# CSA_dampfac
#     1  #v positive multiplier for step-size damping, 0.3 is close to
#       optimal on the sphere

# pc_line_samples
#     False #v one line sample along the evolution path pc

# verb_append
#     0  # initial evaluation counter, if append, do not overwrite output files

# is_feasible
#     is_feasible  #v a function that computes feasibility, by default
#       lambda x, f: f not in (None, np.NaN)

# typical_x
#     None  # used with scaling_of_variables

# CMA_mu
#     None  # parents selection parameter, default is popsize // 2

# CMA_active
#     True  # negative update, conducted after the original update

# CMA_sample_on_sphere_surface
#     False  #v all mutation vectors have the same length, currently
#       (with new_sampling) not in effect

# seed
#     None  # random number seed

# minstd
#     0  #v minimal std (scalar or vector) in any coordinate direction,
#       cave interference with tol*

# verb_disp
#     100  #v verbosity: display console output every verb_disp iteration

# tolfun
#     1e-11  #v termination criterion: tolerance in function value,
#       quite useful

# BoundaryHandler
#     BoundTransform  # or BoundPenalty, unused when
#       ``bounds in (None, [None, None])``

# CMA_sampler
#     None  # a class or instance that implements the interface of
#       `cma.interfaces.StatisticalModelSamplerWithZeroMeanBaseClass`

# bounds
#     [None, None]  # lower (=bounds[0]) and upper domain boundaries,
#       each a scalar or a list/vector

# maxfevals
#     inf  #v maximum number of function evaluations

# popsize
#     4+int(3*np.log(N))  # population size, AKA lambda, number of new
#       solution per iteration

# tolstagnation
#     int(100 + 100 * N**1.5 / popsize)  #v termination if no improvement
#       over tolstagnation iterations

# randn
#     np.random.randn  #v randn(lam, N) must return an np.array of
#       shape (lam, N)

# maxiter
#     100 + 150 * (N+3)**2 // popsize**0.5  #v maximum number of iterations

# tolfunhist
#     1e-12  #v termination criterion: tolerance in function value history

# CSA_squared
#     False  #v use squared length for sigma-adaptation

# CMA_dampsvec_fade
#     0.1  # tentative fading out parameter for sigma vector update

# fixed_variables
#     None  # dictionary with index-value pairs like {0:1.1, 2:0.1} that are
#       not optimized

# CMA_cmean
#     1  # learning rate for the mean value

# timeout
#     inf  #v stop if timeout seconds are exceeded, the string
#       "2.5 * 60**2" evaluates to 2 hours and 30 minutes

# AdaptSigma
#     True  # or False or any CMAAdaptSigmaBase class e.g. CMAAdaptSigmaTPA,
#       CMAAdaptSigmaCSA

# CMA_elitist
#     False  #v or "initial" or True, elitism likely impairs global search
#       performance

# mean_shift_line_samples
#     False #v sample two new solutions colinear to previous mean shift

# CMA_mirrormethod
#     2  # 0=unconditional, 1=selective, 2=selective with delay

# CMA_teststds
#     None  # factors for non-isotropic initial distr. of C, mainly for test
#       purpose, see CMA_stds for production

# ftarget
#     -inf  #v target function value, minimization

# CSA_clip_length_value
#     None  #v untested, [0, 0] means disregarding length completely

# CMA_rankone
#     1.0  # multiplier for rank-one update learning rate of covariance matrix

# CMA_recombination_weights
#     None  # a list, see class RecombinationWeights, overwrites CMA_mu and
#       popsize options

# verb_plot
#     0  #v in fmin(): plot() is called every verb_plot iteration

# scaling_of_variables
#     None  # depreciated, rather use fitness_transformations.ScaleCoordinates
#       instead (or possibly CMA_stds). Scale for each variable in that
#       effective_sigma0 = sigma0*scaling. Internally the variables are divided
#       by scaling_of_variables and sigma is unchanged, default is `np.ones(N)`

# integer_variables
#     []  # index list, invokes basic integer handling: prevent std dev to
#       become too small in the given variables

# transformation
#     None  # depreciated, use cma.fitness_transformations.FitnessTransformation
#       instead.
#       [t0, t1] are two mappings, t0 transforms solutions from CMA-representation
#       to f-representation (tf_pheno), t1 is the (optional) back transformation,
#       see class GenoPheno

# CSA_disregard_length
#     False  #v True is untested

# tolfacupx
#     1e3  #v termination when step-size increases by tolfacupx (diverges). That is,
#       the initial step-size was chosen far too small and better solutions were
#       found far away from the initial solution x0

# verbose
#     3  #v verbosity e.v. of initial/final message, -1 is very quiet, -9 maximally
#       quiet, not yet fully implemented

# termination_callback
#     None  #v a function returning True for termination, called in `stop` with
#       `self` as argument, could be abused for side effects

# CMA_eigenmethod
#     np.linalg.eigh  # or cma.utils.eig or pygsl.eigen.eigenvectors

# verb_log
#     1  #v verbosity: write data to files every verb_log iteration, writing can
#       be time critical on fast to evaluate functions

# signals_filename
#     None  # cma_signals.in  # read versatile options from this file which
#       contains a single options dict, e.g. ``{"timeout": 0}`` to stop,
#       string-values are evaluated, e.g. "np.inf" is valid

# CMA_diagonal
#     0*100*N/popsize**0.5  # nb of iterations with diagonal covariance matrix,
#       True for always

# maxstd
#     inf  #v maximal std in any coordinate direction

# tolconditioncov
#     1e14  #v stop if the condition of the covariance matrix is above
#       `tolconditioncov`

# CSA_damp_mueff_exponent
#     0.5  # zero would mean no dependency of damping on mueff, useful with
#       CSA_disregard_length option
