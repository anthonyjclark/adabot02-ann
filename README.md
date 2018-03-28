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

# Generate Trajectory Data

1. Pick individual from evolutionary history
2. Copy genome (in range 0 to 10)
3. Run experiment with `--genome` flag
4. (Compare fitness with expected)
5. Copy output string (real values)
6. Run `*_vis` binary
7. Output to animation file

# BNN Sweep

1. Pick individual from evolutionary history
2. Copy genome (in range 0 to 10)
3. Run experiment with `--genome` flag
4. (Compare fitness with expected)
5. Copy only BNN parameters from output string (real values)

## TODO

- template NN and BNN
