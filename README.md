# Adabot - Evolving ANNs and FSMs

Build with:

```bash
cd src/
./build build ugv
```

Run with

```bash
cd src/
./build run ugv
```

The build script adds `$HOME/.local/include` to the include path, and `$HOME/.local/lib` to the `DYLD_LIBRARY_PATH` variable.

## Revisit Logger

Use the following to add the `logger` submodule:

```bash
git submodule add revisit/logger
git submodule update --init
cd logger
git remote rm origin
git remote add origin https://github.com/revisit/logger.git
git push origin master
```

To update the submodule, use:

```bash
cd revisit-logger
git pull
```

# Generate Visualization/Trajectory/Evaluation Data

1. Pick individual from evolutionary history
    + high negative fitness from bottom of a `outcmaesxrecentbest.dat` file
2. Copy genome (in range 0 to 10)
    + all values from fitness to end of line
3. Run experiment with `--genome` flag
    + `./evolve_ugv_bnn.py --genome "2.68489637316 9.99723776033 2.83092512356 0.743949436278 1.45031247252 9.84016036964 1.02868840043 4.73290186541 1.72245190821 9.72243287632 2.21900453463 6.49723985829 9.34603023206 9.29940244589 3.43749814329 7.86080041933 0.486365856971"`
4. (Compare fitness with expected)
5. Copy output string (real values)
    + these are genome values converted to simulation values
6. Run `*{_vis|_eval}` binary
    + `../bin/ugv_bnn_eval '30 0 0 0.1015 0.16 0.02283 0 0 3.872 -3.177 -0.2137 -2.622 3.778 -2.225 1.198 3.477 3.44 -1.25 2.289 -3.611' > tmp.csv`
    + first value is TIME_STOP
    + second value is NUM_OBSTACLES
    + third value is OBSTACLE_SEED
7. Output to animation file

# BNN Sweep

1. Pick individual from evolutionary history
2. Copy genome (in range 0 to 10)
3. Run experiment with `--genome` flag
4. (Compare fitness with expected)
5. Copy only BNN parameters from output string (real values)

## TODO

- template NN and BNN
- really need to convert UGV to a class (tons of code duplication)
- refactor controllers into separate files


- Get good trajectory/visualization controllers from evaluate runs (note the seed even though the spacing will be different).
