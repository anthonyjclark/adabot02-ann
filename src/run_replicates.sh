#!/usr/bin/env bash


if [[ "$1" != "fsm" && "$1" != "bnn" ]]; then
    echo "Valid experiments: fsm and bnn"
    exit 0
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

    run_and_time "../bin/evolve_ugv_""$1"".py --seed $i"

    cd ..

done
