This is a synthetic benchmark that runs at varied levels of arithmetic
intensity.

# Building
    Run `make` to build the program, `main`.

# Running
## Help
    $ ./main -h
    Run a benchmark with configurable arithmetic intensity and MPI imbalance
    Usage: ./main [OPTIONS]
    
    Options:
      -h,--help                   Print this help message and exit
      --slowdown UINT=1           When imbalance is present, this specifies the amount of work for slow ranks to perform, as a factor of the amount of work the fast ranks perform.
      --slow-ranks UINT=0         The number of ranks to run with extra work for an imbalanced load.
      --segments UINT=67108864    The number of 64-byte-sized segments in the problem array.
      -v,--verbose                Run in verbose mode
      -s,--single-precision       Run in single-precision mode
      -i,--iterations UINT=5      The number of times to run each phase of the benchmark
      -b,--benchmarks UINT ...    List of benchmark intensity variants to run (all are run if this option is not specified)


## Execution
The program is an MPI app. It uses as many ranks as are available.

For example, use the following command to launch 44 ranks.

    geopmlaunch srun -N 1 -n 44 ./main
