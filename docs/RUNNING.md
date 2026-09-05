# Setting up and running TR-TRP

This guide explains how to configure and run the TR-TRP experiments. Commands
and relative paths assume that the current working directory is the project
root (the directory containing `src`, `config`, and the instance folders).

## Requirements

Install the following software before building the project:

- **JDK 21 or later.** The code uses Java APIs available in Java 21.
- **IBM ILOG CPLEX Optimization Studio**, including its Java API and a valid
  CPLEX license.
- **Gurobi Optimizer**, including its Java API and a valid Gurobi license.
- **IntelliJ IDEA** is recommended.

Both solver installations are required to compile the complete source tree:
some classes import CPLEX APIs and the generic solver layer imports both CPLEX
and Gurobi APIs. Either solver can be selected for a particular run, which is configured later.

Make sure each solver works independently, its license is available, and its
native libraries can be found by the operating system. Consult the installation
guide for the installed solver version where necessary.

## 1. Clone the repository

```bash
git clone <repository-url>
cd TR-TRP
```

Always run the program with this directory as its working directory. The code
loads `config/run.properties`, `config/solver.properties`, and the instance
folders by relative path.

## 2. Open and configure the project in IntelliJ IDEA

1. Select **File > Open** and open the cloned project directory.
2. In **File > Project Structure > Project**, choose JDK 21 or later as the
   Project SDK.
3. In **Project Structure > Modules > Dependencies**, add the CPLEX Java JAR.
   It is normally named `cplex.jar` and is located under the CPLEX Optimization
   Studio installation directory.
4. Add the Gurobi Java JAR in the same way. It is normally named `gurobi.jar`
   and is located under the Gurobi installation directory.
5. Set the scope of both libraries to **Compile** and apply the changes.
6. Ensure that `src` is marked as a **Sources Root**.

The checked-in module file refers to libraries named `cplex` and `gurobi`, but
library definitions and installation paths are machine-specific. Each user
must point those library entries to the JARs installed on their own computer.

## 3. Place the instances

Basic (deterministic) instances must be placed directly in `TestInstances` in
the project root:

```text
TestInstances/
  01_NSH_6-20.txt
  ...
```

Instances with uncertainty must be stored under `RobustInstances`. The first
subfolder specifies the uncertainty budget and the second specifies the
uncertainty parameter:

```text
RobustInstances/
  g1/
    010/    # alpha = 0.10
    025/    # alpha = 0.25
    035/    # alpha = 0.35
  g3/
    010/
    025/
    035/
  g5/
    010/
    025/
    035/
```

Thus, an instance for budget 3 and uncertainty parameter 0.25 belongs in
`RobustInstances/g3/025`. These directory names are significant: the program
constructs the robust input path from the configured budget and alpha value.
The instance file format is documented in the main [README](README.md).

## 4. Select the optimization solver

Edit `config/solver.properties`. Select exactly one runtime solver with:

```properties
solver.name=gurobi
```

or:

```properties
solver.name=cplex
```

The accepted names are lowercase `gurobi` and `cplex`. The same file also
controls the solver time limit, threads, console output, logs, and solution
files. For example:

```properties
solver.timelimit=3600
solver.threads=0
solver.log=false
solver.console=true
solver.separateLog=true
solver.solution=true
solver.logfile=logfile.log
solver.logpath=logs/solver/
solver.solpath=solutions/solver/
```

`solver.threads=0` leaves thread selection at the solver default. Paths are
relative to the project root unless made absolute.

## 5. Configure the experiment

Edit `config/run.properties`. The principal settings are:

```properties
# Overall algorithm time limit in seconds
algorithm.timeLimitSeconds=3600

# One tower count or a comma-separated list
experiment.towerNumbers=3

# Algorithm to execute
algorithm.code=29

# Used by robust algorithm variants
robust.uncertaintyBudget=3
robust.alpha=0.25
```

The comments in `config/run.properties` list the supported algorithm codes.
Common choices include:

- `29`: deterministic ICEA (default multi-threaded implementation)
- `69`: deterministic branch-and-price
- `2`: deterministic MILP
- `79`: robust ICEA
- `70`: robust MILP

Deterministic algorithms read all instance files found in `TestInstances`.
Robust algorithms use `robust.uncertaintyBudget` and `robust.alpha` to select
the corresponding `RobustInstances/g*/0*` folder. For example, budget `3` and
alpha `0.25` select `RobustInstances/g3/025`.

The algorithm time limit and `solver.timelimit` are separate settings. Set both
appropriately for the intended experiment.

## 6. Run the program

Create an IntelliJ Application run configuration with:

- **Main class:** `Main`
- **Use classpath of module:** `TR-TRP`
- **Working directory:** the project root
- **Program arguments:** leave empty for the standard configured run

Run that configuration. With no program arguments, deterministic experiments
start from `TestInstances`. Robust algorithm codes automatically redirect input
to the robust folder selected by the budget and alpha settings.

The equivalent command after compiling the project is conceptually:

```bash
java -cp "<compiled-classes><path-separator><cplex.jar><path-separator><gurobi.jar>" Main
```

Replace the placeholders with the local paths and use `;` as the classpath
separator on Windows or `:` on Linux/macOS. The solver native-library paths and
licenses must also be available to that process. Because solver installation
locations vary, the IntelliJ run configuration is generally the simplest way
to start the project.

## 7. Results

If execution succeeds, the program automatically creates computational summary
files under `summary`. Depending on the selected algorithm and output settings,
it may also create solution files and solver logs under directories such as
`solutions` and `logs`. These output directories do not need to exist before
the run; the program creates the required summary directories as needed.
