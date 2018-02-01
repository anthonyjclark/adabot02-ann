#!/usr/bin/env bash

# Example: ./start_experiment.sh fsm 0 1 &> fsm-0-1_log.txt &!

set -e



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

if [[ -n "$3" && $3 =~ ^[0-9]+$ ]]; then
    NUM_TRIALS="$3"
else
    echo "Third argument (number of trials) must be an integer" >&2; exit 1;
fi


if [ ! -d "experiments" ]; then
    echo "Expected to find 'experiments' directory" >&2; exit 1;
fi



function run_and_time () {
    echo $@
    { $@ &> run_log.txt ; } &> run_time.txt
}


# Build the binaries
cd src
./build_linux.sh all
cd ..

# Create the directory and copy files
EXP_DIR="./experiments/""$BIN_NAME""-""$NUM_OBSTACLES""-""$NUM_TRIALS"

mkdir -p "$EXP_DIR""/bin"
cp "src/bin/ugv_""$BIN_NAME" "$EXP_DIR""/bin"
cp "src/ugv_""$BIN_NAME""/evolve_ugv_""$BIN_NAME"".py" "$EXP_DIR""/bin"
cd "$EXP_DIR"


NUM_REPS=20

for (( i = 1; i <= $NUM_REPS; i++ )); do

    repname="seed"$(printf %02d $i)

    # Don't recreate or overwrite current directories
    if [ ! -d "$repname" ]; then
        mkdir $repname
        cd $repname

        run_and_time "../bin/evolve_ugv_""$BIN_NAME"".py --seed $i --obst $NUM_OBSTACLES --evals $NUM_TRIALS"

        cd ..

    else
        echo "$repname"" already exists"
    fi

done
