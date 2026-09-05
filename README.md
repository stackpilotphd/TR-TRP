# The Tower Repositioning and Technician Routing Problem for Communication Recovery

This repository provides the data, source code, computational results, and
supporting documentation for the **Tower Repositioning and Technician Routing
Problem for Communication Recovery (TR-TRP)**.

The project studies the coordinated deployment of mobile communication towers
and technician crews following a disruption. Towers must be routed and
positioned among affected zones, while technicians travel between repair tasks.
The two routing decisions are linked through the communication zones served by
each task. The repository includes deterministic and robust experiment sets,
with the robust variants accounting for uncertainty at several uncertainty
budgets and parameter levels.

## Repository contents

- [`src/`](src/) contains the Java source code.
- [`TestInstances/`](TestInstances/) contains the basic deterministic test
  instances.
- [`RobustInstances/`](RobustInstances/) contains instances used for experiments
  with uncertainty.
- [`ExperimentsSummary.xlsx`](ExperimentsSummary.xlsx) contains the numerical
  results from the computational experiments.
- [`config/`](config/) contains the runtime configuration files.
- [`RUNNING.md`](RUNNING.md) gives complete installation, configuration, and
  execution instructions.
- [`INSTANCE_FORMAT.md`](INSTANCE_FORMAT.md) explains the contents and required
  format of the instance files.

## Software requirements

The implementation is written in Java and requires **IBM ILOG CPLEX** and
**Gurobi Optimizer**, together with valid licenses and their Java libraries.
The solver used for an experiment can be selected in
[`config/solver.properties`](config/solver.properties). See the
[setup and execution guide](RUNNING.md) for the complete requirements and
step-by-step instructions.

## Running experiments

Experiments are controlled through [`config/run.properties`](config/run.properties).
This file specifies the algorithm, running-time limit, number of towers, and
robust uncertainty parameters.

After a successful run, computational summaries and other enabled outputs are
created automatically. Refer to [RUNNING.md](RUNNING.md) before executing the
code and to [INSTANCE_FORMAT.md](INSTANCE_FORMAT.md) when adding or modifying
instances.

## Numerical results

The workbook [`ExperimentsSummary.xlsx`](ExperimentsSummary.xlsx) provides the
reported numerical results in a form suitable for inspection and comparison.
Together with the included source code, configurations, and complete instance
sets, it supports reproducibility and further computational experimentation.
