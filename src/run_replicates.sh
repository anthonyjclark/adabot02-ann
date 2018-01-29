#!/usr/bin/env bash

NUM_OBSTACLES="0"

if [[ "$1" == "fsm" || "$1" == "bnn" ]]; then
    BIN_NAME="$1"
else
    echo "First argument (valid experiments): fsm|bnn" >&2; exit 1;
fi

if [[ -n "$2" && $2 =~ ^[0-9]+$ ]]; then
    NUM_OBSTACLES="$2"
else
    echo "Second argument (number of obstacles) must be an integer" >&2; exit 1;
fi

function run_and_time () {
    echo $@
    { $@ &> run_log.txt ; } &> run_time.txt
}


NUM_REPS=20

for (( i = 1; i <= $NUM_REPS; i++ )); do

    repname="seed"$(printf %02d $i)

    mkdir $repname
    cd $repname

    run_and_time "../bin/evolve_ugv_""$BIN_NAME"".py --seed $i --obst $NUM_OBSTACLES"

    cd ..

done
