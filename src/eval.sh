#!/usr/bin/env bash

time python3 -u evaluate.py ../experiments/fsm-40-2/ 90 80 3 5 > fsm_eval_40-2_90-80-3-5.csv
time python3 -u evaluate.py ../experiments/bnn-40-2/ 90 80 3 5 > bnn_eval_40-2_90-80-3-5.csv
time python3 -u evaluate.py ../experiments/bnn_twist-40-2/ 90 80 3 5 > tws_eval_40-2_90-80-3-5.csv
