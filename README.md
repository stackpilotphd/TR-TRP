# The Tower Repositioning and Technician Routing Problem for Communication Recovery

This repository provides the data, source code, computational results, and
supporting documentation for the **Tower Repositioning and Technician Routing
Problem for Communication Recovery (TR-TRP)**.

The source code, configuration files, instance archives, and numerical
results are provided to support reproducibility and further computational
experimentation.

## Repository structure

```text
TR-TRP/
├── README.md
├── LICENSE
├── src/
├── config/
│   ├── solver.properties
│   └── run.properties
├── instances/
│   ├── TestInstances.zip
│   └── RobustInstances.zip
├── results/
│   └── ExperimentsSummary.xlsx
└── docs/
    ├── RUNNING.md
    └── INSTANCE_FORMAT.md
```

## Repository contents

- [`src/`](src/) contains the Java source code.
- [`config/`](config/) contains the runtime configuration files.
- [`instances/TestInstances.zip`](instances/TestInstances.zip) contains the
  basic deterministic test instances.
- [`instances/RobustInstances.zip`](instances/RobustInstances.zip) contains the
  instances used for experiments with uncertainty.
- [`results/ExperimentsSummary.xlsx`](results/ExperimentsSummary.xlsx) contains
  the numerical results from the computational experiments.
- [`docs/RUNNING.md`](docs/RUNNING.md) provides complete installation,
  configuration, and execution instructions.
- [`docs/INSTANCE_FORMAT.md`](docs/INSTANCE_FORMAT.md) explains the contents and
  required format of the instance files.

## Software requirements

The implementation is written in Java and requires **IBM ILOG CPLEX** and
**Gurobi Optimizer**, together with valid licenses and their Java libraries.

The solver used for an experiment can be selected in
[`config/solver.properties`](config/solver.properties). See the
[setup and execution guide](docs/RUNNING.md) for the complete requirements and
step-by-step instructions.

## Running experiments

Experiments are controlled through
[`config/run.properties`](config/run.properties). This file specifies the
algorithm, running-time limit, number of towers, and robust uncertainty
parameters.

After a successful run, computational summaries and other enabled outputs are
created automatically. Refer to [RUNNING.md](docs/RUNNING.md) before executing
the code and to [INSTANCE_FORMAT.md](docs/INSTANCE_FORMAT.md) when working with
the instance data.

## Instance data

The [`instances/`](instances/) directory contains two archived instance sets:

- **`TestInstances.zip`**: deterministic test instances.
- **`RobustInstances.zip`**: instances used for the robust computational
  experiments.

For a description of the instance-file structure and required fields, see
[`docs/INSTANCE_FORMAT.md`](docs/INSTANCE_FORMAT.md).

## Numerical results

The workbook
[`results/ExperimentsSummary.xlsx`](results/ExperimentsSummary.xlsx) provides
the reported numerical results in a form suitable for inspection and
comparison.

Together with the included source code, configuration files, and complete
instance archives, these results support reproducibility and further
computational experimentation.

## License

See [`LICENSE`](LICENSE) for the terms under which the contents of this
repository are made available.
