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
```

To update the submodule, use:

```bash
cd revisit-logger
git pull
```
