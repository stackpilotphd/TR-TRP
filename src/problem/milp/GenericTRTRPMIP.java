package problem.milp;

import com.gurobi.gurobi.GRBException;
import ilog.concert.IloException;
import lib.LocalWriter;
import lib.Msg;
import lib.StaticSharedValues;
import problem.Constants;
import problem.graph.Data;
import problem.milp.genericSolverTools.*;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;


public class GenericTRTRPMIP {

    private static final int NO_VAR = -1;
    private static final double EPS = 1e-6;

    private static final Object EXPERIMENT_CSV_LOCK = new Object();
    private static final Object SOLUTION_ANALYSIS_CSV_LOCK = new Object();
    private static final double[] SERVICE_CHECKPOINT_HOURS = {2.5, 5.0, 7.5, 10.0};
    private static final double[] RESTORATION_TARGETS = {0.20, 0.40, 0.60, 0.80, 1.0}; //{0.50, 0.80, 0.95};
    private static final String NUMBER_REGEX =
            "[-+]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][-+]?\\d+)?";
    private static final Pattern GUROBI_ROOT_RELAXATION_PATTERN = Pattern.compile(
            "Root relaxation:\\s*objective\\s+(" + NUMBER_REGEX + ")",
            Pattern.CASE_INSENSITIVE
    );
    private static final Pattern CPLEX_ROOT_NODE_PATTERN = Pattern.compile(
            "^\\s*0\\s+\\d+\\s+(" + NUMBER_REGEX + ")\\b"
    );

    private final SolverTools solver;
    private final SolverConfig config;
    private final Data data;
    private final SolverType solverType;
    private final boolean relaxMip;
    private final boolean keepIntegerY;
    private final boolean addValidCuts;
    private final boolean robustRepairTimes;
    private final int repairTimeBudget;

    // Snapshot experiment metadata so the CSV row is not affected by later global changes.
    private final int experimentTowerNumber;
    private final int experimentCrewNumber;
    private final String experimentFilename;
    private final String experimentBudget;
    private final String experimentAlpha;

    private final int threads;
    private final double timeLimit;
    private final String mipLogFile;
    private final DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");
    private final String currentDateTime;
    private final int crewDepotStart = 0;
    private final int towerDepotStart = 0;
    private final int crewDepotEnd;
    private final int towerDepotEnd;

    private int[][] varXTowers;     // xT[i][j]
    private int[][] varXCrews;      // xR[i][j]
    private int[] varY;             // yT[i]
    private int[] varExtra;         // e[i]
    private int[] varGamma;         // gT[i]
    private int[] varZ;             // zT[i]
    private int[] varOmega;         // oC[i], used in the nominal model
    private int[][] varOmegaRobust; // oC[i][g], used for robust repair times
    private int[] varDelta;         // delta[i]

    private double objVal = Double.NaN;
    private double bestBound = Double.NaN;
    private double mipGap = Double.NaN;
    private double rootNodeLowerBound = Double.NaN;
    private double cpuSeconds = Double.NaN;

    public GenericTRTRPMIP(SolverConfig config) {
        this(config, Constants.RELAX_MIP, Constants.KEEP_INTEGER_Ys, false);
    }

    public GenericTRTRPMIP(
            SolverConfig config,
            boolean relaxMip,
            boolean keepIntegerY,
            boolean addValidCuts
    ) {
        this.currentDateTime = LocalDateTime.now().format(formatter);
        this.config = config;
        this.solverType = SolverType.fromString(config.solverName);
        this.solver = new SolverTools(solverType);
        this.data = Data.getInstance();
        this.relaxMip = relaxMip;
        this.keepIntegerY = keepIntegerY;
        this.addValidCuts = addValidCuts;
        this.robustRepairTimes = Constants.ROBUST && Constants.R_TOGGLE != 1;
        this.repairTimeBudget = robustRepairTimes ? StaticSharedValues.budget : 1;
        this.experimentTowerNumber = data.getTowerNumber();
        experimentFilename = LocalWriter.filename;
        experimentCrewNumber = data.getCrewNumber();
        this.experimentBudget = String.valueOf(Constants.ROBUST ? Constants.BUDGET : 0);
        this.experimentAlpha = String.valueOf(Constants.ROBUST ? Constants.ALPHA : 0);
        if (robustRepairTimes && repairTimeBudget <= 0) {
            throw new IllegalArgumentException(
                    "Robust repair-time budget must be positive; StaticSharedValues.budget = "
                            + StaticSharedValues.budget
            );
        }
        this.threads = config.threads;
        this.timeLimit = config.timeLimit;
        this.crewDepotEnd = data.getDepotEnd(true);
        this.towerDepotEnd = data.getDepotEnd(false);

        String logFolder;
        switch (solverType) {
            case CPLEX -> logFolder = config.logPath + "cplex/";
            case GUROBI -> logFolder = config.logPath + "gurobi/";
            default -> throw new IllegalStateException("Unrecognized solver: " + solverType);
        }
        this.mipLogFile = logFolder + config.logFile;
    }

    public void build() throws IloException, GRBException {
        if (Constants.ROBUST && Constants.R_TOGGLE == 1) {
            throw new UnsupportedOperationException(
                    "GenericTRTRPMIP supports the nominal model and the robust repair-time model; "
                            + "robust crew-travel-time uncertainty is still implemented only in MIP."
            );
        }

        System.out.println(
                robustRepairTimes
                        ? "Initializing the robust repair-time TR-TRP generic model."
                        : "Initializing the TR-TRP generic model."
        );
        solver.initializeModel(threads, timeLimit, mipLogFile, !config.log, !config.console);

        createVariables();
        setObjective();
        addConstraints();
    }

    private void createVariables() throws IloException, GRBException {
        System.out.println("Creating variables.");

        varXTowers = new int[data.getNodeNumber()][data.getNodeNumber() + 1];
        varXCrews = new int[data.getTasks()][data.getTasks() + 1];
        fill2D(varXTowers, NO_VAR);
        fill2D(varXCrews, NO_VAR);

        varY = filled1D(data.getNodeNumber(), NO_VAR);
        varExtra = filled1D(data.getNodeNumber(), NO_VAR);
        varGamma = filled1D(data.getNodeNumber() + 1, NO_VAR);
        varZ = filled1D(data.getNodeNumber(), NO_VAR);
        varOmega = filled1D(data.getTasks() + 1, NO_VAR);
        varOmegaRobust = new int[data.getTasks() + 1][repairTimeBudget];
        fill2D(varOmegaRobust, NO_VAR);
        varDelta = filled1D(data.getNodeNumber(), NO_VAR);

        VariableType arcType = relaxMip ? VariableType.CONTINUOUS : VariableType.BINARY;

        // Tower routing variables xT[i][j].
        for (int i = 0; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber() + 1; j++) {
                if (i == towerDepotStart && j == towerDepotEnd) continue;
                if (i == j) continue;
                varXTowers[i][j] = solver.createVariable(
                        arcType, 0.0, 1.0, name("xT", i, j)
                );
            }
        }

        // Crew routing variables xR[i][j].
        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks() + 1; j++) {
                if (i == crewDepotStart && j == crewDepotEnd) continue;
                if (i == j) continue;
                varXCrews[i][j] = solver.createVariable(
                        arcType, 0.0, 1.0, name("xR", i, j)
                );
            }
        }

        // Zone-level variables.
        for (int i = 0; i < data.getNodeNumber(); i++) {
            varExtra[i] = solver.createVariable(VariableType.CONTINUOUS, 0.0, Double.MAX_VALUE, name("e", i));
            varGamma[i] = solver.createVariable(VariableType.CONTINUOUS, 0.0, Double.MAX_VALUE, name("gT", i));
            varZ[i] = solver.createVariable(VariableType.CONTINUOUS, 0.0, Double.MAX_VALUE, name("zT", i));

            if (i == 0) {
                solver.setVariableBounds(varGamma[i], 0.0, 0.0);
                solver.setVariableBounds(varZ[i], 0.0, 0.0);
                continue;
            }

            VariableType yType;
            if (!relaxMip || keepIntegerY) {
                yType = VariableType.BINARY;
            } else {
                yType = VariableType.CONTINUOUS;
            }
            varY[i] = solver.createVariable(yType, 0.0, 1.0, name("yT", i));
        }

        // Artificial tower end-depot arrival time.
        if (!hasVar(varGamma[towerDepotEnd])) {
            varGamma[towerDepotEnd] = solver.createVariable(
                    VariableType.CONTINUOUS, 0.0, Double.MAX_VALUE, name("gT", towerDepotEnd)
            );
        }

        // Crew completion time variables.
        if (robustRepairTimes) {
            for (int i = 0; i < data.getTasks() + 1; i++) {
                for (int g = 0; g < repairTimeBudget; g++) {
                    varOmegaRobust[i][g] = solver.createVariable(
                            VariableType.CONTINUOUS,
                            0.0,
                            Double.MAX_VALUE,
                            name("oC", i, g)
                    );
                    if (i == 0) {
                        solver.setVariableBounds(varOmegaRobust[i][g], 0.0, 0.0);
                    }
                }
            }
        } else {
            for (int i = 0; i < data.getTasks() + 1; i++) {
                varOmega[i] = solver.createVariable(VariableType.CONTINUOUS, 0.0, Double.MAX_VALUE, name("oC", i));
                if (i == 0) {
                    solver.setVariableBounds(varOmega[i], 0.0, 0.0);
                }
            }
        }

        for (int i = 1; i < data.getNodeNumber(); i++) {
            varDelta[i] = solver.createVariable(VariableType.CONTINUOUS, 0.0, Double.MAX_VALUE, name("delta", i));
        }
    }

    private void setObjective() throws IloException, GRBException {
        System.out.println("Setting objective.");
        solver.initializeExpression();
        for (int i = 1; i < data.getNodeNumber(); i++) {
            solver.addTerm(data.getWeights()[i], varDelta[i]);
            solver.addTerm(-data.getWeights()[i], varZ[i]);
        }
        solver.setObjective(ObjectiveSense.MIN);
    }

    private void addConstraints() throws IloException, GRBException {
        System.out.println("Adding constraints.");

        addCrewDepotConstraints();
        addCrewFlowAndVisitConstraints();
        addCrewTimeConstraints();
        addZoneRestorationConstraints();

        addTowerVisitAndDepotConstraints();
        addTowerFlowConstraints();
        addTowerTimeConstraints();
        addServiceUpperBoundConstraints();
        addExtraLinearizationConstraints();

        if (addValidCuts) {
            addValidInequalities();
        }
    }

    private void addCrewDepotConstraints() throws IloException, GRBException {
        // sum_i xR[0][i] = number of crews
        solver.initializeExpression();
        for (int i = 1; i < data.getTasks(); i++) {
            addIfPresent(1.0, varXCrews[crewDepotStart][i]);
        }
        solver.addConstraint(ConstraintSense.EQ, data.getCrewNumber(), "crewFromDepot");

        // sum_i xR[i][end] = number of crews
        solver.initializeExpression();
        for (int i = 1; i < data.getTasks(); i++) {
            addIfPresent(1.0, varXCrews[i][crewDepotEnd]);
        }
        solver.addConstraint(ConstraintSense.EQ, data.getCrewNumber(), "crewToDepot");
    }

    private void addCrewFlowAndVisitConstraints() throws IloException, GRBException {
        // Flow conservation at each task.
        for (int i = 1; i < data.getTasks(); i++) {
            solver.initializeExpression();
            for (int j = 1; j < data.getTasks() + 1; j++) {
                if (i != j) addIfPresent(1.0, varXCrews[i][j]);
            }
            for (int j = 0; j < data.getTasks(); j++) {
                if (i != j) addIfPresent(-1.0, varXCrews[j][i]);
            }
            solver.addConstraint(ConstraintSense.EQ, 0.0, name("flow", i));
        }

        // Each task is visited exactly once.
        for (int i = 1; i < data.getTasks(); i++) {
            solver.initializeExpression();
            for (int j = 1; j < data.getTasks() + 1; j++) {
                if (i != j) addIfPresent(1.0, varXCrews[i][j]);
            }
            solver.addConstraint(ConstraintSense.EQ, 1.0, name("visit", i));
        }
    }

    private void addCrewTimeConstraints() throws IloException, GRBException {
        if (robustRepairTimes) {
            addRobustRepairTimeCrewConstraints();
            return;
        }

        double bigM = data.getHorizon();

        // omega_i - omega_j + (M + travel_ij + service_j) xR_ij <= M
        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks() + 1; j++) {
                if (i == crewDepotStart && j == crewDepotEnd) continue;
                if (i == j) continue;
                if (!hasVar(varXCrews[i][j])) continue;

                solver.initializeExpression();
                solver.addTerm(1.0, varOmega[i]);
                solver.addTerm(-1.0, varOmega[j]);
                solver.addTerm(
                        bigM + data.getCrewTravelTimeMatrix()[i][j] + data.getServiceTimeMatrix()[j],
                        varXCrews[i][j]
                );
                solver.addConstraint(ConstraintSense.LE, bigM, name("taskTime", i, j));
            }
        }

        // omega_end <= horizon
        solver.initializeExpression();
        solver.addTerm(1.0, varOmega[crewDepotEnd]);
        solver.addConstraint(ConstraintSense.LE, data.getHorizon(), "crewShift");
    }

    private void addRobustRepairTimeCrewConstraints() throws IloException, GRBException {
        // Nominal propagation for every uncertainty-budget layer g:
        // omega_i,g - omega_j,g + (M_ij + travel_ij + service_j) xR_ij <= M_ij.
        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks() + 1; j++) {
                if (i == crewDepotStart && j == crewDepotEnd) continue;
                if (i == j) continue;
                if (!hasVar(varXCrews[i][j])) continue;

                for (int g = 0; g < repairTimeBudget; g++) {
                    double bigM = robustRepairTimeBigM(i, j);
                    solver.initializeExpression();
                    solver.addTerm(1.0, varOmegaRobust[i][g]);
                    solver.addTerm(-1.0, varOmegaRobust[j][g]);
                    solver.addTerm(
                            bigM + data.getCrewTravelTimeMatrix()[i][j] + data.getServiceTimeMatrix()[j],
                            varXCrews[i][j]
                    );
                    solver.addConstraint(ConstraintSense.LE, bigM, name("taskTime1", i, j, g));
                }
            }
        }

        // Deviation propagation: spending one unit of uncertainty budget on repair time j.
        // omega_i,g-1 - omega_j,g
        //     + (M_ij + travel_ij + service_j + deviation_j) xR_ij <= M_ij.
        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks() + 1; j++) {
                if (i == crewDepotStart && j == crewDepotEnd) continue;
                if (i == j) continue;
                if (!hasVar(varXCrews[i][j])) continue;

                for (int g = 1; g < repairTimeBudget; g++) {
                    double bigM = robustRepairTimeBigM(i, j);
                    solver.initializeExpression();
                    solver.addTerm(1.0, varOmegaRobust[i][g - 1]);
                    solver.addTerm(-1.0, varOmegaRobust[j][g]);
                    solver.addTerm(
                            bigM
                                    + data.getCrewTravelTimeMatrix()[i][j]
                                    + data.getServiceTimeMatrix()[j]
                                    + data.getServiceDeviation()[j],
                            varXCrews[i][j]
                    );
                    solver.addConstraint(ConstraintSense.LE, bigM, name("taskTime2", i, j, g));
                }
            }
        }

        for (int g = 0; g < repairTimeBudget; g++) {
            solver.initializeExpression();
            solver.addTerm(1.0, varOmegaRobust[crewDepotEnd][g]);
            solver.addConstraint(ConstraintSense.LE, data.getHorizon(), name("crewShift", g));
        }
    }

    private void addZoneRestorationConstraints() throws IloException, GRBException {
        // delta_i >= omega_q for every task q associated with zone i.
        for (int i = 1; i < data.getNodeNumber(); i++) {
            for (Integer q : data.getZone2tasks()[i]) {
                if (robustRepairTimes) {
                    for (int g = 0; g < repairTimeBudget; g++) {
                        solver.initializeExpression();
                        solver.addTerm(1.0, varDelta[i]);
                        solver.addTerm(-1.0, varOmegaRobust[q][g]);
                        solver.addConstraint(
                                ConstraintSense.GE,
                                0.0,
                                name("deltaToMaxTaskTime", i, q, g)
                        );
                    }
                } else {
                    solver.initializeExpression();
                    solver.addTerm(1.0, varDelta[i]);
                    solver.addTerm(-1.0, varOmega[q]);
                    solver.addConstraint(ConstraintSense.GE, 0.0, name("deltaToMaxTaskTime", i, q));
                }
            }
        }
    }

    private void addTowerVisitAndDepotConstraints() throws IloException, GRBException {
        // y_i = sum_j xT[j][i]
        for (int i = 1; i < data.getNodeNumber(); i++) {
            solver.initializeExpression();
            solver.addTerm(1.0, varY[i]);
            for (int j = 0; j < data.getNodeNumber(); j++) {
                if (i != j) addIfPresent(-1.0, varXTowers[j][i]);
            }
            solver.addConstraint(ConstraintSense.EQ, 0.0, name("linkZone", i));
        }

        // sum_i xT[0][i] = number of towers
        solver.initializeExpression();
        for (int i = 1; i < data.getNodeNumber(); i++) {
            addIfPresent(1.0, varXTowers[towerDepotStart][i]);
        }
        solver.addConstraint(ConstraintSense.EQ, data.getTowerNumber(), "towerFromDepot");

        // sum_i xT[i][end] = number of towers
        solver.initializeExpression();
        for (int i = 1; i < data.getNodeNumber(); i++) {
            addIfPresent(1.0, varXTowers[i][towerDepotEnd]);
        }
        solver.addConstraint(ConstraintSense.EQ, data.getTowerNumber(), "towerToDepot");
    }

    private void addTowerFlowConstraints() throws IloException, GRBException {
        for (int i = 1; i < data.getNodeNumber(); i++) {
            solver.initializeExpression();
            for (int j = 1; j < data.getNodeNumber() + 1; j++) {
                if (i != j) addIfPresent(1.0, varXTowers[i][j]);
            }
            for (int j = 0; j < data.getNodeNumber(); j++) {
                if (i != j) addIfPresent(-1.0, varXTowers[j][i]);
            }
            solver.addConstraint(ConstraintSense.EQ, 0.0, name("flowT", i));
        }
    }

    private void addTowerTimeConstraints() throws IloException, GRBException {
        // gamma_i - gamma_j + z_i + (M_i + travel/position increment) xT_ij <= M_i
        for (int i = 0; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber() + 1; j++) {
                if (i == towerDepotStart && j == towerDepotEnd) continue;
                if (i == j) continue;
                if (!hasVar(varXTowers[i][j])) continue;

                double minReturnToCrewDepot = minCrewReturnTimeForZoneOrZero(i);
                double bigM = data.getHorizon() - minReturnToCrewDepot;
                double increment = towerIncrement(i, j);

                solver.initializeExpression();
                solver.addTerm(1.0, varGamma[i]);
                solver.addTerm(-1.0, varGamma[j]);
                solver.addTerm(1.0, varZ[i]);
                solver.addTerm(bigM + increment, varXTowers[i][j]);
                solver.addConstraint(ConstraintSense.LE, bigM, name("TowerTimes", i, j));
            }
        }
    }

    private void addServiceUpperBoundConstraints() throws IloException, GRBException {
        // gamma_i + z_i <= extra_i
        for (int i = 1; i < data.getNodeNumber(); i++) {
            solver.initializeExpression();
            solver.addTerm(1.0, varGamma[i]);
            solver.addTerm(1.0, varZ[i]);
            solver.addTerm(-1.0, varExtra[i]);
            solver.addConstraint(ConstraintSense.LE, 0.0, name("serviceUB", i));
        }
    }

    private void addExtraLinearizationConstraints() throws IloException, GRBException {
        // E1: extra_i <= M_i y_i
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double bigM = data.getHorizon() - minCrewReturnTimeForZoneOrZero(i);
            solver.initializeExpression();
            solver.addTerm(1.0, varExtra[i]);
            solver.addTerm(-bigM, varY[i]);
            solver.addConstraint(ConstraintSense.LE, 0.0, name("E1", i));
        }

        // E2: extra_i <= delta_i
        for (int i = 1; i < data.getNodeNumber(); i++) {
            solver.initializeExpression();
            solver.addTerm(1.0, varExtra[i]);
            solver.addTerm(-1.0, varDelta[i]);
            solver.addConstraint(ConstraintSense.LE, 0.0, name("E2", i));
        }

        // E3: extra_i >= delta_i + M_i y_i - horizon
        // Written as: extra_i - M_i y_i - delta_i >= -horizon.
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double bigM = data.getHorizon() - minCrewReturnTimeForZoneOrZero(i);
            solver.initializeExpression();
            solver.addTerm(1.0, varExtra[i]);
            solver.addTerm(-bigM, varY[i]);
            solver.addTerm(-1.0, varDelta[i]);
            solver.addConstraint(ConstraintSense.GE, -data.getHorizon(), name("E3", i));
        }
    }

    private void addValidInequalities() throws IloException, GRBException {
        // Restoration lower bound for each task.
        for (int i = 1; i < data.getTasks(); i++) {
            double lb = data.getCrewTravelTimeMatrix()[0][i] + data.getServiceTimeMatrix()[i];
            if (robustRepairTimes) {
                for (int g = 0; g < repairTimeBudget; g++) {
                    solver.initializeExpression();
                    solver.addTerm(1.0, varOmegaRobust[i][g]);
                    solver.addConstraint(ConstraintSense.GE, lb, name("RestorationLB", i, g));
                }
            } else {
                solver.initializeExpression();
                solver.addTerm(1.0, varOmega[i]);
                solver.addConstraint(ConstraintSense.GE, lb, name("RestorationLB", i));
            }
        }

        // Zone restoration lower bound.
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double max = 0.0;
            for (Integer q : data.getZone2tasks()[i]) {
                double value = data.getCrewTravelTimeMatrix()[0][q] + data.getServiceTimeMatrix()[q];
                max = Math.max(max, value);
            }
            solver.initializeExpression();
            solver.addTerm(1.0, varDelta[i]);
            solver.addConstraint(ConstraintSense.GE, max, name("ZoneRestorationLB", i));
        }

        // Upper bound on the tower end-depot time.
        double minCrewReturn = Double.MAX_VALUE;
        for (int k = 0; k < data.getTasks(); k++) {
            minCrewReturn = Math.min(minCrewReturn, data.getCrewTravelTimeMatrix()[k][crewDepotEnd]);
        }
        double maxTowerReturn = 0.0;
        for (int i = 0; i < data.getNodeNumber(); i++) {
            maxTowerReturn = Math.max(maxTowerReturn, data.getTowerTravelTimeMatrix()[i][towerDepotEnd]);
        }
        double upperBound = data.getHorizon() - minCrewReturn + maxTowerReturn;
        solver.initializeExpression();
        solver.addTerm(1.0, varGamma[towerDepotEnd]);
        solver.addConstraint(ConstraintSense.LE, upperBound, "validCutForTowers");

        // Bound on total service duration.
        solver.initializeExpression();
        for (int i = 1; i < data.getNodeNumber(); i++) {
            solver.addTerm(1.0, varZ[i]);
        }
        solver.addConstraint(ConstraintSense.LE, data.getMaxZ(), "serviceBound");
    }

    private static Path findFixedCrewSolution(Path solutionDir, String instanceFilename)
            throws IOException {

        String filename = Path.of(instanceFilename)
                .getFileName()
                .toString();

        String instanceName = filename.contains(".")
                ? filename.substring(0, filename.lastIndexOf('.'))
                : filename;

        Pattern pattern = Pattern.compile(
                Pattern.quote(instanceName) + "-\\d+-\\d+\\.txt"
        );

        List<Path> matches;

        try (Stream<Path> files = Files.list(solutionDir)) {
            matches = files
                    .filter(Files::isRegularFile)
                    .filter(path -> pattern.matcher(path.getFileName().toString()).matches())
                    .sorted(Comparator.comparing(path -> path.getFileName().toString()))
                    .toList();
        }

        if (matches.isEmpty()) {
            throw new IllegalArgumentException(
                    "No fixed crew solution found for instance " + instanceFilename
                            + " in folder " + solutionDir
            );
        }

        if (matches.size() > 1) {
            throw new IllegalArgumentException(
                    "Multiple fixed crew solutions found for instance " + instanceFilename
                            + ": " + matches
            );
        }

        return matches.get(0);
    }

    /**
     * Fix crew arcs from a file containing one route per line, e.g. [0, 1, 4, 31].
     * Call after build() and before solveMILP().
     */
    public void fixCrewRoutesFromFile(Path filePath,boolean forbidOtherArcs)
            throws IOException, IloException, GRBException {

        System.out.println("Applying FIXED CREW ROUTES");


        boolean[][] used = new boolean[data.getTasks()][data.getTasks() + 1];

        for (String line : Files.readAllLines(filePath)) {

            line = line.trim();

            // Skip empty lines if any
            if (line.isEmpty()) {
                continue;
            }

            if(line.equals("Technician Plans")){
                continue;
            }
            if(line.equals("Tower Routes")){
                break;
            }

            // Remove "[" and "]"
            line = line.replace("[", "").replace("]", "").trim();

            String[] tokens = line.split(",");

            ArrayList<Integer> route = new ArrayList<>();

            for (String token : tokens) {
                route.add(Integer.parseInt(token.trim()));
            }

            if (route.size() < 2) {
                continue;
            }

            for (int k = 0; k < route.size() - 1; k++) {
                int from = route.get(k);
                int to = route.get(k + 1);
                if (!hasVar(varXCrews[from][to])) {
                    throw new IllegalArgumentException("Invalid crew arc in fixed route: " + from + " -> " + to);
                }
                solver.setVariableBounds(varXCrews[from][to], 1.0, 1.0);
                used[from][to] = true;
            }
        }



        if (forbidOtherArcs) {
            for (int i = 0; i < data.getTasks(); i++) {
                for (int j = 1; j < data.getTasks() + 1; j++) {
                    if (i != j && hasVar(varXCrews[i][j]) && !used[i][j]) {
                        solver.setVariableBounds(varXCrews[i][j], 0.0, 0.0);
                    }
                }
            }
        }
    }

    /**
     * Fix tower arcs from a file containing one route per line, e.g. [0, 3, 8, 26].
     * Call after build() and before solveMILP().
     */
    public void fixTowerRoutesFromFile(Path filePath, boolean forbidOtherArcs)
            throws IOException, IloException, GRBException {
        System.out.println("Applying FIXED TOWER ROUTES");
        boolean[][] used = new boolean[data.getNodeNumber()][data.getNodeNumber() + 1];

        boolean hasFound = false;
        for (String line : Files.readAllLines(filePath)) {



            line = line.trim();

            if(!hasFound){
                if(!line.equals("Tower Routes"))
                    continue;
                else {
                    hasFound = true;
                    continue; //skip the line "Tower Routes"
                }
            }



            // Skip empty lines if any
            if (line.isEmpty()) {
                continue;
            }


            // Remove "[" and "]"
            line = line.replace("[", "").replace("]", "").trim();

            String[] tokens = line.split(",");

            ArrayList<Integer> route = new ArrayList<>();

            for (String token : tokens) {
                route.add(Integer.parseInt(token.trim()));
            }

            if (route.size() < 2) {
                continue;
            }

            for (int k = 0; k < route.size() - 1; k++) {
                int from = route.get(k);
                int to = route.get(k + 1);
                if (!hasVar(varXTowers[from][to])) {
                    throw new IllegalArgumentException("Invalid tower arc in fixed route: " + from + " -> " + to);
                }
                solver.setVariableBounds(varXTowers[from][to], 1.0, 1.0);
                used[from][to] = true;
            }
        }




        if (forbidOtherArcs) {
            for (int i = 0; i < data.getNodeNumber(); i++) {
                for (int j = 1; j < data.getNodeNumber() + 1; j++) {
                    if (i != j && hasVar(varXTowers[i][j]) && !used[i][j]) {
                        solver.setVariableBounds(varXTowers[i][j], 0.0, 0.0);
                    }
                }
            }
        }
    }

    public boolean solveMILP() throws IloException, GRBException {
        System.out.print("Starting to solve the TR-TRP MIP at ");
        System.out.println(LocalDateTime.now().format(formatter));

        // Reset per-run metrics in case this object is reused.
        this.objVal = Double.NaN;
        this.bestBound = Double.NaN;
        this.mipGap = Double.NaN;
        this.rootNodeLowerBound = Double.NaN;
        this.cpuSeconds = Double.NaN;

        double start = System.nanoTime();
        solveRoot();
        boolean feasible = solver.solveModel();
        this.cpuSeconds = (System.nanoTime() - start) * 1e-9;
        this.bestBound = solver.getBestBound();
//        this.rootNodeLowerBound = readRootNodeLowerBoundFromLog();

        System.out.println(
                "Spent " + Operators.truncate(cpuSeconds, 3) + " seconds solving the model."
        );
        if (Double.isFinite(rootNodeLowerBound)) {
            System.out.println("Root Node LB = " + rootNodeLowerBound);
        } else {
            System.out.println(
                    "Root Node LB is unavailable. Ensure the solver log is enabled and uses "
                            + "a standard " + solverType + " log format."
            );
        }
        return feasible;
    }

    public boolean hasFeasibleSolution() throws IloException, GRBException {
        return solver.hasFeasibleSolution();
    }

    public void analyzeResults(boolean feasible) throws IloException, GRBException {
        if (!feasible) {
            System.out.println("The MIP model is infeasible.");
            return;
        }
        if (!solver.hasFeasibleSolution()) {
            System.out.println("No integer feasible solution was found within the time limit.");
            return;
        }

        this.objVal = solver.getBestIntegerFeasibleSolutionValue();
        this.bestBound = solver.getBestBound();
        this.mipGap = solver.getMIPGapPercentage();

        System.out.println("MIP Objective Value = " + objVal);
        System.out.println("Best Bound at Termination = " + bestBound);
        System.out.println("MIP Gap at Termination = " + mipGap + " %");
    }

    public void printNonzeroSolution() throws IloException, GRBException {
        System.out.println("--------------- TR-TRP SOLUTION OUTPUT ----------------");
        printNonzeroArcs("Crew", varXCrews, data.getTasks(), data.getTasks() + 1);
        printNonzeroArcs("Tower", varXTowers, data.getNodeNumber(), data.getNodeNumber() + 1);

        System.out.println("Zone repair times delta:");
        for (int i = 1; i < data.getNodeNumber(); i++) {
            printIfNonZero("delta", i, varDelta[i]);
        }

        System.out.println("Task completion times omega:");
        if (robustRepairTimes) {
            for (int i = 1; i < data.getTasks(); i++) {
                for (int g = 0; g < repairTimeBudget; g++) {
                    printIfNonZero("omega", i, g, varOmegaRobust[i][g]);
                }
            }
        } else {
            for (int i = 1; i < data.getTasks(); i++) {
                printIfNonZero("omega", i, varOmega[i]);
            }
        }

        System.out.println("Tower service durations z:");
        for (int i = 1; i < data.getNodeNumber(); i++) {
            printIfNonZero("z", i, varZ[i]);
        }

        System.out.println("Tower selected zones y:");
        for (int i = 1; i < data.getNodeNumber(); i++) {
            printIfNonZero("y", i, varY[i]);
        }

        System.out.println("Tower arrival times gamma:");
        for (int i = 1; i < data.getNodeNumber() + 1; i++) {
            printIfNonZero("gamma", i, varGamma[i]);
        }
    }

    /**
     * Writes five CSV files describing the current integer solution:
     * <ul>
     *     <li>{@code *-zones.csv}: tower timing, zone restoration timing, and zone task structure,</li>
     *     <li>{@code *-tasks.csv}: task timing, crew assignment, zone multiplicity, and task impact,</li>
     *     <li>{@code *-crew-routes.csv}: each crew route, travel time, repair time, and return time,</li>
     *     <li>{@code *-task-zone-incidence.csv}: one row per task-zone association, including
     *         leave-one-out completion impact,</li>
     *     <li>{@code *-task-sharing-summary.csv}: instance-level structural and solution-dependent
     *         task-sharing statistics.</li>
     * </ul>
     *
     * <p>The leave-one-out completion gap for task q in zone i is
     * {@code max(0, omega[q] - max_{r in zone i, r != q} omega[r])}. For a singleton-task
     * zone, the comparison baseline is zero. A positive gap means that the task is the unique
     * completion bottleneck for that zone. Summing this gap over all zones associated with a
     * shared task measures how broadly its completion time can affect zone restoration.</p>
     *
     * Call this after {@link #solveMILP()} has produced a feasible integer solution and
     * before {@link #clear()}.
     *
     * @param outputDirectory directory in which the CSV files are created
     * @param outputPrefix prefix used for all five files; a final filename extension is removed
     * @return paths of the zone, task, crew-route, task-zone-incidence, and sharing-summary
     *         CSV files, in that order
     */
    public List<Path> writeSolutionAnalysisCsv(Path outputDirectory, String outputPrefix)
            throws IOException, IloException, GRBException {

        if (relaxMip) {
            throw new IllegalStateException(
                    "Route analysis requires an integer solution; relaxMip must be false."
            );
        }
        if (!solver.hasFeasibleSolution()) {
            throw new IllegalStateException(
                    "No feasible integer solution is available for CSV analysis."
            );
        }

        Files.createDirectories(outputDirectory);
        String prefix = safeFileStem(outputPrefix);

        Path zonesFile = outputDirectory.resolve(prefix + "-zones.csv");
        Path tasksFile = outputDirectory.resolve(prefix + "-tasks.csv");
        Path crewRoutesFile = outputDirectory.resolve(prefix + "-crew-routes.csv");
        Path taskZoneIncidenceFile = outputDirectory.resolve(prefix + "-task-zone-incidence.csv");
        Path taskSharingSummaryFile = outputDirectory.resolve(prefix + "-task-sharing-summary.csv");

        List<List<Integer>> crewRoutes = extractSelectedRoutes(
                varXCrews,
                crewDepotEnd,
                data.getCrewNumber(),
                true,
                "crew"
        );
        List<List<Integer>> towerRoutes = extractSelectedRoutes(
                varXTowers,
                towerDepotEnd,
                data.getTowerNumber(),
                false,
                "tower"
        );

        List<List<Integer>> zonesByTask = buildZonesByTask();

        int[] crewRouteByTask = new int[data.getTasks()];
        int[] towerRouteByZone = new int[data.getNodeNumber()];
        Arrays.fill(crewRouteByTask, -1);
        Arrays.fill(towerRouteByZone, -1);

        double[] calculatedTaskCompletion = new double[data.getTasks()];
        double[] crewTravelTime = new double[crewRoutes.size()];
        double[] crewRepairTime = new double[crewRoutes.size()];
        double[] calculatedCrewReturn = new double[crewRoutes.size()];
        double[] omegaBasedCrewReturn = new double[crewRoutes.size()];
        Arrays.fill(calculatedTaskCompletion, Double.NaN);

        for (int routeIndex = 0; routeIndex < crewRoutes.size(); routeIndex++) {
            int routeId = routeIndex + 1;
            List<Integer> route = crewRoutes.get(routeIndex);
            double routeTravelTime = 0.0;
            double routeRepairTime = 0.0;
            double routeTime = 0.0;

            for (int position = 1; position < route.size(); position++) {
                int from = route.get(position - 1);
                int to = route.get(position);

                double travelTime = data.getCrewTravelTimeMatrix()[from][to];
                routeTravelTime += travelTime;
                routeTime += travelTime;

                // Repair/service time is counted only for actual tasks, not the end depot.
                if (to != crewDepotEnd) {
                    double repairTime = data.getServiceTimeMatrix()[to];
                    routeRepairTime += repairTime;
                    routeTime += repairTime;

                    crewRouteByTask[to] = routeId;
                    calculatedTaskCompletion[to] = routeTime;
                }
            }

            crewTravelTime[routeIndex] = routeTravelTime;
            crewRepairTime[routeIndex] = routeRepairTime;
            calculatedCrewReturn[routeIndex] = routeTime;
            int lastTask = route.get(route.size() - 2);
            omegaBasedCrewReturn[routeIndex] =
                    taskCompletionValue(lastTask)
                            + data.getCrewTravelTimeMatrix()[lastTask][crewDepotEnd]
                            + data.getServiceTimeMatrix()[crewDepotEnd];
        }

        for (int routeIndex = 0; routeIndex < towerRoutes.size(); routeIndex++) {
            int routeId = routeIndex + 1;
            List<Integer> route = towerRoutes.get(routeIndex);
            for (int position = 1; position < route.size() - 1; position++) {
                int zone = route.get(position);
                if (towerRouteByZone[zone] != -1) {
                    throw new IllegalStateException(
                            "Zone " + zone + " appears in more than one tower route."
                    );
                }
                towerRouteByZone[zone] = routeId;
            }
        }

        writeZoneAnalysisCsv(zonesFile, towerRouteByZone, zonesByTask);
        writeTaskAnalysisCsv(
                tasksFile,
                zonesByTask,
                crewRouteByTask,
                calculatedTaskCompletion
        );
        writeCrewRouteAnalysisCsv(
                crewRoutesFile,
                crewRoutes,
                crewTravelTime,
                crewRepairTime,
                calculatedCrewReturn,
                omegaBasedCrewReturn
        );
        writeTaskZoneIncidenceCsv(taskZoneIncidenceFile, zonesByTask);
        writeTaskSharingSummaryCsv(taskSharingSummaryFile, zonesByTask);

        return List.of(
                zonesFile,
                tasksFile,
                crewRoutesFile,
                taskZoneIncidenceFile,
                taskSharingSummaryFile
        );
    }

    /**
     * Builds the inverse task-to-zone incidence lists and validates the stated data assumptions:
     * every task belongs to at least one zone, every zone requires at least one task, and a
     * task-zone association is not duplicated inside a zone.
     */
    private List<List<Integer>> buildZonesByTask() {
        List<List<Integer>> zonesByTask = new ArrayList<>(data.getTasks());
        for (int task = 0; task < data.getTasks(); task++) {
            zonesByTask.add(new ArrayList<>());
        }

        for (int zone = 1; zone < data.getNodeNumber(); zone++) {
            List<Integer> zoneTasks = data.getZone2tasks()[zone];
            if (zoneTasks == null || zoneTasks.isEmpty()) {
                throw new IllegalStateException(
                        "Zone " + zone + " has no associated tasks."
                );
            }

            for (Integer task : zoneTasks) {
                if (task == null || task <= 0 || task >= data.getTasks()) {
                    throw new IllegalStateException(
                            "Zone " + zone + " contains invalid task index " + task
                    );
                }
                if (zonesByTask.get(task).contains(zone)) {
                    throw new IllegalStateException(
                            "Task " + task + " is duplicated within zone " + zone + "."
                    );
                }
                zonesByTask.get(task).add(zone);
            }
        }

        for (int task = 1; task < data.getTasks(); task++) {
            if (zonesByTask.get(task).isEmpty()) {
                throw new IllegalStateException(
                        "Task " + task + " is not associated with any zone."
                );
            }
        }
        return zonesByTask;
    }

    private void writeZoneAnalysisCsv(
            Path file,
            int[] towerRouteByZone,
            List<List<Integer>> zonesByTask
    ) throws IOException, IloException, GRBException {

        try (BufferedWriter writer = Files.newBufferedWriter(
                file, StandardCharsets.UTF_8
        )) {
            writeCsvRow(writer,
                    "zone_id",
                    "tower_route_id",
                    "task_ids",
                    "task_count",
                    "shared_task_count",
                    "shared_task_fraction",
                    "mean_zones_per_task",
                    "max_zones_per_task",
                    "latest_task_ids",
                    "latest_task_count",
                    "latest_task_is_shared",
                    "unique_bottleneck_task_id",
                    "unique_bottleneck_is_shared",
                    "zone_leave_one_out_completion_gap",
                    "tower_arrival_time",
                    "tower_service_duration",
                    "tower_departure_time",
                    "zone_restoration_time",
                    "max_task_restoration_time",
                    "restoration_minus_departure",
                    "delta_minus_max_task_time",
                    "timing_relation",
                    "tower_departs_before_restoration",
                    "departure_after_restoration_violation"
            );

            for (int zone = 1; zone < data.getNodeNumber(); zone++) {
                if (towerRouteByZone[zone] < 0) {
                    continue;
                }

                List<Integer> zoneTasks = data.getZone2tasks()[zone];
                int sharedTaskCount = 0;
                int maxZonesPerTask = 0;
                int totalZonesPerTask = 0;
                for (Integer task : zoneTasks) {
                    int multiplicity = zonesByTask.get(task).size();
                    totalZonesPerTask += multiplicity;
                    maxZonesPerTask = Math.max(maxZonesPerTask, multiplicity);
                    if (multiplicity > 1) {
                        sharedTaskCount++;
                    }
                }

                double maxTaskRestoration = zoneMaxTaskCompletion(zone, -1);
                List<Integer> latestTasks = new ArrayList<>();
                for (Integer task : zoneTasks) {
                    double taskCompletion = taskCompletionValue(task);
                    if (Math.abs(taskCompletion - maxTaskRestoration) <= EPS) {
                        latestTasks.add(task);
                    }
                }

                boolean latestTaskIsShared = false;
                for (Integer task : latestTasks) {
                    if (zonesByTask.get(task).size() > 1) {
                        latestTaskIsShared = true;
                        break;
                    }
                }

                int uniqueBottleneckTask = latestTasks.size() == 1 ? latestTasks.get(0) : -1;
                boolean uniqueBottleneckIsShared = uniqueBottleneckTask >= 0
                        && zonesByTask.get(uniqueBottleneckTask).size() > 1;
                double otherTasksMax = uniqueBottleneckTask >= 0
                        ? zoneMaxTaskCompletion(zone, uniqueBottleneckTask)
                        : maxTaskRestoration;
                if (otherTasksMax == Double.NEGATIVE_INFINITY) {
                    otherTasksMax = 0.0;
                }
                double zoneLeaveOneOutGap = uniqueBottleneckTask >= 0
                        ? Math.max(0.0, maxTaskRestoration - otherTasksMax)
                        : 0.0;

                double arrival = solver.getVariableValue(varGamma[zone]);
                double service = solver.getVariableValue(varZ[zone]);
                double departure = arrival + service;
                double restoration = solver.getVariableValue(varDelta[zone]);

                double restorationMinusDeparture = restoration - departure;
                double deltaSlack = restoration - maxTaskRestoration;
                boolean departsBeforeRestoration = departure < restoration - EPS;
                boolean departureAfterRestoration = departure > restoration + EPS;
                String timingRelation = departsBeforeRestoration
                        ? "BEFORE_RESTORATION"
                        : departureAfterRestoration
                        ? "AFTER_RESTORATION"
                        : "AT_RESTORATION";

                if(!timingRelation.equals("AT_RESTORATION"))
                    Msg.stop("gotcha!!!!!");

                writeCsvRow(writer,
                        Integer.toString(zone),
                        Integer.toString(towerRouteByZone[zone]),
                        zoneTasks.toString(),
                        Integer.toString(zoneTasks.size()),
                        Integer.toString(sharedTaskCount),
                        formatCsvDouble((double) sharedTaskCount / zoneTasks.size()),
                        formatCsvDouble((double) totalZonesPerTask / zoneTasks.size()),
                        Integer.toString(maxZonesPerTask),
                        latestTasks.toString(),
                        Integer.toString(latestTasks.size()),
                        Boolean.toString(latestTaskIsShared),
                        uniqueBottleneckTask >= 0
                                ? Integer.toString(uniqueBottleneckTask)
                                : "",
                        Boolean.toString(uniqueBottleneckIsShared),
                        formatCsvDouble(zoneLeaveOneOutGap),
                        formatCsvDouble(arrival),
                        formatCsvDouble(service),
                        formatCsvDouble(departure),
                        formatCsvDouble(restoration),
                        formatCsvDouble(maxTaskRestoration),
                        formatCsvDouble(restorationMinusDeparture),
                        formatCsvDouble(deltaSlack),
                        timingRelation,
                        Boolean.toString(departsBeforeRestoration),
                        Boolean.toString(departureAfterRestoration)
                );
            }
        }
    }

    private void writeTaskAnalysisCsv(
            Path file,
            List<List<Integer>> zonesByTask,
            int[] crewRouteByTask,
            double[] calculatedTaskCompletion
    ) throws IOException, IloException, GRBException {

        try (BufferedWriter writer = Files.newBufferedWriter(
                file, StandardCharsets.UTF_8
        )) {
            writeCsvRow(writer,
                    "task_id",
                    "zone_ids",
                    "zone_count",
                    "is_shared_task",
                    "service_time",
                    "total_associated_zone_weight",
                    "crew_route_id",
                    "model_restoration_time",
                    "route_calculated_restoration_time",
                    "model_minus_calculated_time",
                    "latest_zone_count",
                    "unique_bottleneck_zone_count",
                    "unique_bottleneck_zone_weight",
                    "total_leave_one_out_completion_gap",
                    "weighted_leave_one_out_completion_gap",
                    "potential_duplicate_service_time_avoided"
            );

            for (int task = 1; task < data.getTasks(); task++) {
                if (crewRouteByTask[task] < 0) {
                    throw new IllegalStateException(
                            "Task " + task + " is not assigned to a depot-rooted crew route."
                    );
                }

                List<Integer> taskZones = zonesByTask.get(task);
                if (taskZones.isEmpty()) {
                    throw new IllegalStateException(
                            "Task " + task + " is not assigned to a zone."
                    );
                }

                double modelTime = taskCompletionValue(task);
                double calculatedTime = calculatedTaskCompletion[task];
                double totalZoneWeight = 0.0;
                double uniqueBottleneckZoneWeight = 0.0;
                double totalLeaveOneOutGap = 0.0;
                double weightedLeaveOneOutGap = 0.0;
                int latestZoneCount = 0;
                int uniqueBottleneckZoneCount = 0;

                for (Integer zone : taskZones) {
                    double zoneWeight = data.getWeights()[zone];
                    double zoneMax = zoneMaxTaskCompletion(zone, -1);
                    double otherTasksMax = zoneMaxTaskCompletion(zone, task);
                    if (otherTasksMax == Double.NEGATIVE_INFINITY) {
                        otherTasksMax = 0.0;
                    }
                    double leaveOneOutGap = Math.max(0.0, modelTime - otherTasksMax);

                    totalZoneWeight += zoneWeight;
                    if (Math.abs(modelTime - zoneMax) <= EPS) {
                        latestZoneCount++;
                    }
                    if (leaveOneOutGap > EPS) {
                        uniqueBottleneckZoneCount++;
                        uniqueBottleneckZoneWeight += zoneWeight;
                    }
                    totalLeaveOneOutGap += leaveOneOutGap;
                    weightedLeaveOneOutGap += zoneWeight * leaveOneOutGap;
                }

                double serviceTime = data.getServiceTimeMatrix()[task];
                double duplicateServiceAvoided = serviceTime * (taskZones.size() - 1);

                writeCsvRow(writer,
                        Integer.toString(task),
                        taskZones.toString(),
                        Integer.toString(taskZones.size()),
                        Boolean.toString(taskZones.size() > 1),
                        formatCsvDouble(serviceTime),
                        formatCsvDouble(totalZoneWeight),
                        Integer.toString(crewRouteByTask[task]),
                        formatCsvDouble(modelTime),
                        formatCsvDouble(calculatedTime),
                        formatCsvDouble(modelTime - calculatedTime),
                        Integer.toString(latestZoneCount),
                        Integer.toString(uniqueBottleneckZoneCount),
                        formatCsvDouble(uniqueBottleneckZoneWeight),
                        formatCsvDouble(totalLeaveOneOutGap),
                        formatCsvDouble(weightedLeaveOneOutGap),
                        formatCsvDouble(duplicateServiceAvoided)
                );
            }
        }
    }

    private void writeCrewRouteAnalysisCsv(
            Path file,
            List<List<Integer>> crewRoutes,
            double[] crewTravelTime,
            double[] crewRepairTime,
            double[] calculatedCrewReturn,
            double[] omegaBasedCrewReturn
    ) throws IOException, IloException, GRBException {

        double sharedEndDepotTime = taskCompletionValue(crewDepotEnd);

        try (BufferedWriter writer = Files.newBufferedWriter(
                file, StandardCharsets.UTF_8
        )) {
            writeCsvRow(writer,
                    "crew_route_id",
                    "route",
                    "last_task_id",
                    "total_travel_time",
                    "total_repair_time",
                    "total_route_time",
                    "route_calculated_return_time",
                    "return_time_from_last_task_omega",
                    "shared_model_end_depot_time",
                    "omega_return_minus_calculated_return"
            );

            for (int routeIndex = 0; routeIndex < crewRoutes.size(); routeIndex++) {
                List<Integer> route = crewRoutes.get(routeIndex);
                int lastTask = route.get(route.size() - 2);

                writeCsvRow(writer,
                        Integer.toString(routeIndex + 1),
                        route.toString(),
                        Integer.toString(lastTask),
                        formatCsvDouble(crewTravelTime[routeIndex]),
                        formatCsvDouble(crewRepairTime[routeIndex]),
                        formatCsvDouble(
                                crewTravelTime[routeIndex] + crewRepairTime[routeIndex]
                        ),
                        formatCsvDouble(calculatedCrewReturn[routeIndex]),
                        formatCsvDouble(omegaBasedCrewReturn[routeIndex]),
                        formatCsvDouble(sharedEndDepotTime),
                        formatCsvDouble(
                                omegaBasedCrewReturn[routeIndex]
                                        - calculatedCrewReturn[routeIndex]
                        )
                );
            }
        }
    }

    /**
     * Writes one row for every task-zone association. This is the most detailed output for
     * assessing whether a shared task is merely associated with several zones or is actually
     * delaying several zones in the current solution.
     */
    private void writeTaskZoneIncidenceCsv(
            Path file,
            List<List<Integer>> zonesByTask
    ) throws IOException, IloException, GRBException {

        try (BufferedWriter writer = Files.newBufferedWriter(
                file, StandardCharsets.UTF_8
        )) {
            writeCsvRow(writer,
                    "task_id",
                    "zone_id",
                    "task_zone_count",
                    "zone_task_count",
                    "is_shared_task",
                    "zone_is_singleton_task",
                    "service_time",
                    "task_completion_time",
                    "zone_max_task_completion_time",
                    "is_latest_task_for_zone",
                    "other_tasks_max_completion_time",
                    "leave_one_out_completion_gap",
                    "is_unique_completion_bottleneck",
                    "zone_weight",
                    "weighted_leave_one_out_completion_gap"
            );

            for (int task = 1; task < data.getTasks(); task++) {
                double taskCompletion = taskCompletionValue(task);
                for (Integer zone : zonesByTask.get(task)) {
                    List<Integer> zoneTasks = data.getZone2tasks()[zone];
                    double zoneMax = zoneMaxTaskCompletion(zone, -1);
                    double otherTasksMax = zoneMaxTaskCompletion(zone, task);
                    if (otherTasksMax == Double.NEGATIVE_INFINITY) {
                        otherTasksMax = 0.0;
                    }
                    double leaveOneOutGap = Math.max(0.0, taskCompletion - otherTasksMax);
                    double zoneWeight = data.getWeights()[zone];

                    writeCsvRow(writer,
                            Integer.toString(task),
                            Integer.toString(zone),
                            Integer.toString(zonesByTask.get(task).size()),
                            Integer.toString(zoneTasks.size()),
                            Boolean.toString(zonesByTask.get(task).size() > 1),
                            Boolean.toString(zoneTasks.size() == 1),
                            formatCsvDouble(data.getServiceTimeMatrix()[task]),
                            formatCsvDouble(taskCompletion),
                            formatCsvDouble(zoneMax),
                            Boolean.toString(Math.abs(taskCompletion - zoneMax) <= EPS),
                            formatCsvDouble(otherTasksMax),
                            formatCsvDouble(leaveOneOutGap),
                            Boolean.toString(leaveOneOutGap > EPS),
                            formatCsvDouble(zoneWeight),
                            formatCsvDouble(zoneWeight * leaveOneOutGap)
                    );
                }
            }
        }
    }

    /**
     * Writes a compact reviewer-facing summary. Structural metrics depend only on the
     * task-zone incidence data; solution-impact metrics additionally use task completion times.
     */
    private void writeTaskSharingSummaryCsv(
            Path file,
            List<List<Integer>> zonesByTask
    ) throws IOException, IloException, GRBException {

        int taskCount = data.getTasks() - 1;
        int zoneCount = data.getNodeNumber() - 1;
        int associationCount = 0;
        int sharedTaskCount = 0;
        int minZonesPerTask = Integer.MAX_VALUE;
        int maxZonesPerTask = 0;
        int minTasksPerZone = Integer.MAX_VALUE;
        int maxTasksPerZone = 0;
        int singletonZoneCount = 0;
        int zonesWithSharedTasks = 0;

        double totalServiceTime = 0.0;
        double sharedTaskServiceTime = 0.0;
        double zoneExpandedServiceTime = 0.0;

        for (int task = 1; task < data.getTasks(); task++) {
            int multiplicity = zonesByTask.get(task).size();
            double serviceTime = data.getServiceTimeMatrix()[task];

            associationCount += multiplicity;
            minZonesPerTask = Math.min(minZonesPerTask, multiplicity);
            maxZonesPerTask = Math.max(maxZonesPerTask, multiplicity);
            totalServiceTime += serviceTime;
            zoneExpandedServiceTime += serviceTime * multiplicity;
            if (multiplicity > 1) {
                sharedTaskCount++;
                sharedTaskServiceTime += serviceTime;
            }
        }

        for (int zone = 1; zone < data.getNodeNumber(); zone++) {
            List<Integer> zoneTasks = data.getZone2tasks()[zone];
            int zoneTaskCount = zoneTasks.size();
            minTasksPerZone = Math.min(minTasksPerZone, zoneTaskCount);
            maxTasksPerZone = Math.max(maxTasksPerZone, zoneTaskCount);
            if (zoneTaskCount == 1) {
                singletonZoneCount++;
            }

            boolean hasSharedTask = false;
            for (Integer task : zoneTasks) {
                if (zonesByTask.get(task).size() > 1) {
                    hasSharedTask = true;
                    break;
                }
            }
            if (hasSharedTask) {
                zonesWithSharedTasks++;
            }
        }

        int latestIncidenceCount = 0;
        int uniqueBottleneckIncidenceCount = 0;
        int sharedUniqueBottleneckIncidenceCount = 0;
        boolean[] taskIsUniqueBottleneck = new boolean[data.getTasks()];
        boolean[] zoneHasSharedUniqueBottleneck = new boolean[data.getNodeNumber()];
        double totalLeaveOneOutGap = 0.0;
        double weightedLeaveOneOutGap = 0.0;
        double sharedTaskLeaveOneOutGap = 0.0;
        double sharedTaskWeightedLeaveOneOutGap = 0.0;
        double weightedCompletionExposure = 0.0;
        double sharedTaskWeightedCompletionExposure = 0.0;

        for (int task = 1; task < data.getTasks(); task++) {
            boolean shared = zonesByTask.get(task).size() > 1;
            double taskCompletion = taskCompletionValue(task);

            for (Integer zone : zonesByTask.get(task)) {
                double zoneMax = zoneMaxTaskCompletion(zone, -1);
                double otherTasksMax = zoneMaxTaskCompletion(zone, task);
                if (otherTasksMax == Double.NEGATIVE_INFINITY) {
                    otherTasksMax = 0.0;
                }
                double gap = Math.max(0.0, taskCompletion - otherTasksMax);
                double weight = data.getWeights()[zone];

                if (Math.abs(taskCompletion - zoneMax) <= EPS) {
                    latestIncidenceCount++;
                }
                if (gap > EPS) {
                    uniqueBottleneckIncidenceCount++;
                    taskIsUniqueBottleneck[task] = true;
                    if (shared) {
                        sharedUniqueBottleneckIncidenceCount++;
                        zoneHasSharedUniqueBottleneck[zone] = true;
                    }
                }

                totalLeaveOneOutGap += gap;
                weightedLeaveOneOutGap += weight * gap;
                weightedCompletionExposure += weight * taskCompletion;
                if (shared) {
                    sharedTaskLeaveOneOutGap += gap;
                    sharedTaskWeightedLeaveOneOutGap += weight * gap;
                    sharedTaskWeightedCompletionExposure += weight * taskCompletion;
                }
            }
        }

        int uniqueBottleneckTaskCount = 0;
        int sharedUniqueBottleneckTaskCount = 0;
        for (int task = 1; task < data.getTasks(); task++) {
            if (taskIsUniqueBottleneck[task]) {
                uniqueBottleneckTaskCount++;
                if (zonesByTask.get(task).size() > 1) {
                    sharedUniqueBottleneckTaskCount++;
                }
            }
        }

        int zonesWithSharedUniqueBottleneck = 0;
        for (int zone = 1; zone < data.getNodeNumber(); zone++) {
            if (zoneHasSharedUniqueBottleneck[zone]) {
                zonesWithSharedUniqueBottleneck++;
            }
        }

        double duplicateServiceAvoided = zoneExpandedServiceTime - totalServiceTime;

        try (BufferedWriter writer = Files.newBufferedWriter(
                file, StandardCharsets.UTF_8
        )) {
            writeCsvRow(writer, "category", "metric", "value", "interpretation");

            writeSummaryMetric(writer, "structure", "task_count", taskCount,
                    "Number of actual tasks, excluding the depot.");
            writeSummaryMetric(writer, "structure", "zone_count", zoneCount,
                    "Number of actual zones, excluding the depot.");
            writeSummaryMetric(writer, "structure", "task_zone_association_count", associationCount,
                    "Total number of task-zone incidences.");
            writeSummaryMetric(writer, "structure", "extra_task_zone_associations",
                    associationCount - taskCount,
                    "Associations beyond one mandatory zone per task; a direct measure of sharing.");
            writeSummaryMetric(writer, "structure", "shared_task_count", sharedTaskCount,
                    "Tasks associated with more than one zone.");
            writeSummaryMetric(writer, "structure", "shared_task_fraction",
                    safeRatio(sharedTaskCount, taskCount),
                    "Fraction of tasks associated with more than one zone.");
            writeSummaryMetric(writer, "structure", "mean_zones_per_task",
                    safeRatio(associationCount, taskCount),
                    "Average number of zones associated with a task.");
            writeSummaryMetric(writer, "structure", "min_zones_per_task", minZonesPerTask,
                    "Minimum task multiplicity; expected to be at least one.");
            writeSummaryMetric(writer, "structure", "max_zones_per_task", maxZonesPerTask,
                    "Maximum number of zones sharing one task.");
            writeSummaryMetric(writer, "structure", "mean_tasks_per_zone",
                    safeRatio(associationCount, zoneCount),
                    "Average number of required tasks per zone.");
            writeSummaryMetric(writer, "structure", "min_tasks_per_zone", minTasksPerZone,
                    "Minimum number of required tasks in a zone; expected to be at least one.");
            writeSummaryMetric(writer, "structure", "max_tasks_per_zone", maxTasksPerZone,
                    "Maximum number of required tasks in a zone.");
            writeSummaryMetric(writer, "structure", "singleton_zone_count", singletonZoneCount,
                    "Zones whose restoration depends on exactly one task.");
            writeSummaryMetric(writer, "structure", "zones_with_shared_tasks", zonesWithSharedTasks,
                    "Zones that require at least one task also required by another zone.");
            writeSummaryMetric(writer, "structure", "zones_with_shared_tasks_fraction",
                    safeRatio(zonesWithSharedTasks, zoneCount),
                    "Fraction of zones exposed to at least one shared task.");

            writeSummaryMetric(writer, "service_leverage", "total_task_service_time", totalServiceTime,
                    "Actual repair time when each task is serviced once.");
            writeSummaryMetric(writer, "service_leverage", "shared_task_service_time",
                    sharedTaskServiceTime,
                    "Repair time belonging to tasks shared by multiple zones.");
            writeSummaryMetric(writer, "service_leverage", "shared_task_service_time_fraction",
                    safeRatio(sharedTaskServiceTime, totalServiceTime),
                    "Fraction of actual repair time spent on shared tasks.");
            writeSummaryMetric(writer, "service_leverage", "zone_expanded_service_time",
                    zoneExpandedServiceTime,
                    "Hypothetical service time if every task-zone incidence were repaired separately.");
            writeSummaryMetric(writer, "service_leverage",
                    "potential_duplicate_service_time_avoided", duplicateServiceAvoided,
                    "Repair time avoided because one repair can satisfy several zones.");
            writeSummaryMetric(writer, "service_leverage", "service_time_leverage_ratio",
                    safeRatio(zoneExpandedServiceTime, totalServiceTime),
                    "Zone-expanded service demand divided by actual once-per-task service time.");

            writeSummaryMetric(writer, "solution_impact", "latest_task_zone_incidence_count",
                    latestIncidenceCount,
                    "Task-zone incidences in which the task ties for the latest completion in its zone.");
            writeSummaryMetric(writer, "solution_impact", "unique_bottleneck_incidence_count",
                    uniqueBottleneckIncidenceCount,
                    "Task-zone incidences with a positive leave-one-out completion gap.");
            writeSummaryMetric(writer, "solution_impact",
                    "shared_unique_bottleneck_incidence_count",
                    sharedUniqueBottleneckIncidenceCount,
                    "Positive bottleneck incidences attributable to shared tasks.");
            writeSummaryMetric(writer, "solution_impact", "unique_bottleneck_task_count",
                    uniqueBottleneckTaskCount,
                    "Tasks that uniquely determine completion of at least one associated zone.");
            writeSummaryMetric(writer, "solution_impact", "shared_unique_bottleneck_task_count",
                    sharedUniqueBottleneckTaskCount,
                    "Shared tasks that uniquely determine completion of at least one zone.");
            writeSummaryMetric(writer, "solution_impact",
                    "zones_with_shared_unique_bottleneck", zonesWithSharedUniqueBottleneck,
                    "Zones whose unique latest task is shared with at least one other zone.");
            writeSummaryMetric(writer, "solution_impact", "total_leave_one_out_completion_gap",
                    totalLeaveOneOutGap,
                    "Unweighted sum of completion gaps over all task-zone incidences.");
            writeSummaryMetric(writer, "solution_impact", "weighted_leave_one_out_completion_gap",
                    weightedLeaveOneOutGap,
                    "Zone-weighted sum of completion gaps over all task-zone incidences.");
            writeSummaryMetric(writer, "solution_impact", "shared_task_leave_one_out_completion_gap",
                    sharedTaskLeaveOneOutGap,
                    "Unweighted completion-gap total attributable to shared tasks.");
            writeSummaryMetric(writer, "solution_impact",
                    "shared_task_weighted_leave_one_out_completion_gap",
                    sharedTaskWeightedLeaveOneOutGap,
                    "Zone-weighted completion-gap total attributable to shared tasks.");
            writeSummaryMetric(writer, "solution_impact",
                    "shared_fraction_of_weighted_leave_one_out_gap",
                    safeRatio(sharedTaskWeightedLeaveOneOutGap, weightedLeaveOneOutGap),
                    "Fraction of weighted bottleneck impact attributable to shared tasks.");
            writeSummaryMetric(writer, "solution_impact", "weighted_completion_exposure",
                    weightedCompletionExposure,
                    "Sum of task completion time times zone weight over all task-zone incidences.");
            writeSummaryMetric(writer, "solution_impact", "shared_task_weighted_completion_exposure",
                    sharedTaskWeightedCompletionExposure,
                    "Weighted completion exposure attributable to shared tasks.");
            writeSummaryMetric(writer, "solution_impact",
                    "shared_fraction_of_weighted_completion_exposure",
                    safeRatio(sharedTaskWeightedCompletionExposure, weightedCompletionExposure),
                    "Fraction of weighted completion exposure attributable to shared tasks.");
        }
    }

    private double zoneMaxTaskCompletion(int zone, int excludedTask)
            throws IloException, GRBException {
        double max = Double.NEGATIVE_INFINITY;
        for (Integer task : data.getZone2tasks()[zone]) {
            if (task == excludedTask) {
                continue;
            }
            max = Math.max(max, taskCompletionValue(task));
        }
        return max;
    }

    private static double safeRatio(double numerator, double denominator) {
        return Math.abs(denominator) <= EPS ? 0.0 : numerator / denominator;
    }

    private static void writeSummaryMetric(
            BufferedWriter writer,
            String category,
            String metric,
            int value,
            String interpretation
    ) throws IOException {
        writeCsvRow(writer, category, metric, Integer.toString(value), interpretation);
    }

    private static void writeSummaryMetric(
            BufferedWriter writer,
            String category,
            String metric,
            double value,
            String interpretation
    ) throws IOException {
        writeCsvRow(writer, category, metric, formatCsvDouble(value), interpretation);
    }

    private List<List<Integer>> extractSelectedRoutes(
            int[][] routeVariables,
            int depotEnd,
            int expectedRouteCount,
            boolean requireEveryInternalNode,
            String routeLabel
    ) throws IloException, GRBException {

        List<Integer> starts = new ArrayList<>();
        for (int node = 1; node < depotEnd; node++) {
            if (isSelected(routeVariables[0][node])) {
                starts.add(node);
            }
        }
        starts.sort(Integer::compareTo);

        if (starts.size() != expectedRouteCount) {
            throw new IllegalStateException(
                    "Expected " + expectedRouteCount + " " + routeLabel
                            + " routes from the depot, but found " + starts.size() + "."
            );
        }

        boolean[] reachedFromDepot = new boolean[depotEnd];
        List<List<Integer>> routes = new ArrayList<>();

        for (Integer start : starts) {
            List<Integer> route = new ArrayList<>();
            route.add(0);
            int current = start;

            while (current != depotEnd) {
                if (current <= 0 || current >= depotEnd) {
                    throw new IllegalStateException(
                            "Invalid node " + current + " in " + routeLabel + " route."
                    );
                }
                if (reachedFromDepot[current]) {
                    throw new IllegalStateException(
                            "Cycle or merged routes detected at " + routeLabel
                                    + " node " + current + "."
                    );
                }

                reachedFromDepot[current] = true;
                route.add(current);
                current = selectedSuccessor(
                        routeVariables,
                        current,
                        depotEnd,
                        routeLabel
                );

                if (route.size() > depotEnd + 1) {
                    throw new IllegalStateException(
                            "Cycle detected while reconstructing a " + routeLabel + " route."
                    );
                }
            }

            route.add(depotEnd);
            routes.add(route);
        }

        for (int node = 1; node < depotEnd; node++) {
            boolean hasSelectedOutgoing = hasSelectedOutgoingArc(
                    routeVariables, node, depotEnd
            );
            if (hasSelectedOutgoing && !reachedFromDepot[node]) {
                throw new IllegalStateException(
                        "Selected " + routeLabel + " subtour is not connected to the depot;"
                                + " node " + node + " is affected."
                );
            }
            if (requireEveryInternalNode && !reachedFromDepot[node]) {
                throw new IllegalStateException(
                        "Required " + routeLabel + " node " + node
                                + " is not on a depot-rooted route."
                );
            }
        }

        return routes;
    }

    private int selectedSuccessor(
            int[][] routeVariables,
            int from,
            int depotEnd,
            String routeLabel
    ) throws IloException, GRBException {

        int successor = -1;
        for (int to = 1; to <= depotEnd; to++) {
            if (to == from || !hasVar(routeVariables[from][to])) {
                continue;
            }
            if (isSelected(routeVariables[from][to])) {
                if (successor != -1) {
                    throw new IllegalStateException(
                            "More than one selected successor for " + routeLabel
                                    + " node " + from + "."
                    );
                }
                successor = to;
            }
        }

        if (successor == -1) {
            throw new IllegalStateException(
                    "No selected successor for " + routeLabel + " node " + from + "."
            );
        }
        return successor;
    }

    private boolean hasSelectedOutgoingArc(
            int[][] routeVariables,
            int from,
            int depotEnd
    ) throws IloException, GRBException {
        for (int to = 1; to <= depotEnd; to++) {
            if (to != from
                    && hasVar(routeVariables[from][to])
                    && isSelected(routeVariables[from][to])) {
                return true;
            }
        }
        return false;
    }

    private boolean isSelected(int variable) throws IloException, GRBException {
        return hasVar(variable) && solver.getVariableValue(variable) > 0.5;
    }

    private static String safeFileStem(String value) {
        String filename = Path.of(value).getFileName().toString();
        int extension = filename.lastIndexOf('.');
        String stem = extension > 0 ? filename.substring(0, extension) : filename;
        stem = stem.replaceAll("[^A-Za-z0-9._-]", "_");
        return stem.isEmpty() ? "solution-analysis" : stem;
    }

    private static String formatCsvDouble(double value) {
        return String.format(Locale.ROOT, "%.6f", value);
    }

    private static void writeCsvRow(BufferedWriter writer, String... values)
            throws IOException {
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                writer.write(',');
            }
            writer.write(escapeCsv(values[i]));
        }
        writer.newLine();
    }

    private static String escapeCsv(String value) {
        if (value == null) {
            return "";
        }
        boolean quote = value.indexOf(',') >= 0
                || value.indexOf('"') >= 0
                || value.indexOf('\n') >= 0
                || value.indexOf('\r') >= 0;
        if (!quote) {
            return value;
        }
        return '"' + value.replace("\"", "\"\"") + '"';
    }

    /**
     * Appends one experiment result to a shared CSV file.
     *
     * <p>The JVM-level monitor serializes threads in this process, while the operating-system
     * file lock serializes separate JVMs/processes writing to the same file. The header is
     * written exactly once when the file is empty.</p>
     *
     * <p>Call this after {@link #solveMILP()} and {@link #analyzeResults(boolean)}, and before
     * {@link #clear()}.</p>
     */
    public void appendExperimentResultCsv(Path outputFile) throws IOException {
        Path absoluteFile = outputFile.toAbsolutePath();
        Path parent = absoluteFile.getParent();
        if (parent != null) {
            Files.createDirectories(parent);
        }

        String header = csvLine("filename","Crew#",
                "Tower#",
                "algorithm",
                "BUDGET",
                "Alpha",
                "Root Node LB",
                "CPU",
                "Best Value",
                "Lower Bound at Termination",
                "Date",
                "Solver"
        );
        String row = csvLine(experimentFilename,Integer.toString(experimentCrewNumber),
                Integer.toString(experimentTowerNumber),
                "MILP",
                experimentBudget,
                experimentAlpha,
                formatOptionalCsvDouble(rootNodeLowerBound),
                formatOptionalCsvDouble(cpuSeconds),
                formatOptionalCsvDouble(objVal),
                formatOptionalCsvDouble(bestBound),
                currentDateTime,
                solverType.toString()
        );

        synchronized (EXPERIMENT_CSV_LOCK) {
            try (FileChannel channel = FileChannel.open(
                    absoluteFile,
                    StandardOpenOption.CREATE,
                    StandardOpenOption.READ,
                    StandardOpenOption.WRITE
            ); FileLock ignored = channel.lock()) {
                boolean writeHeader = channel.size() == 0L;
                channel.position(channel.size());

                if (writeHeader) {
                    writeFully(channel, header + System.lineSeparator());
                }
                writeFully(channel, row + System.lineSeparator());
                channel.force(true);
            }
        }
    }

    private double readRootNodeLowerBoundFromLog() {
        Path logPath = Path.of(mipLogFile);
        if (!Files.isRegularFile(logPath)) {
            return Double.NaN;
        }

        double rootBound = Double.NaN;
        try (BufferedReader reader = Files.newBufferedReader(logPath, StandardCharsets.UTF_8)) {
            String line;
            while ((line = reader.readLine()) != null) {
                if (solverType == SolverType.GUROBI) {
                    Matcher matcher = GUROBI_ROOT_RELAXATION_PATTERN.matcher(line);
                    if (matcher.find()) {
                        // Keep the last match in case the log file contains an older run.
                        rootBound = parseFiniteDouble(matcher.group(1));
                    }
                } else if (solverType == SolverType.CPLEX) {
                    Matcher matcher = CPLEX_ROOT_NODE_PATTERN.matcher(line);
                    if (matcher.find()) {
                        // Keep the last node-0 objective because CPLEX may print several
                        // root rows as cuts improve the root relaxation.
                        rootBound = parseFiniteDouble(matcher.group(1));
                    }
                }
            }
        } catch (IOException | NumberFormatException ex) {
            System.err.println(
                    "Could not read Root Node LB from " + logPath + ": " + ex.getMessage()
            );
            return Double.NaN;
        }
        return rootBound;
    }

    private static double parseFiniteDouble(String value) {
        double parsed = Double.parseDouble(value);
        return Double.isFinite(parsed) ? parsed : Double.NaN;
    }

    private static String formatOptionalCsvDouble(double value) {
        return Double.isFinite(value) ? formatCsvDouble(value) : "";
    }

    private static String csvLine(String... values) {
        StringBuilder row = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                row.append(',');
            }
            row.append(escapeCsv(values[i]));
        }
        return row.toString();
    }

    private static void writeFully(FileChannel channel, String text) throws IOException {
        ByteBuffer buffer = StandardCharsets.UTF_8.encode(text);
        while (buffer.hasRemaining()) {
            channel.write(buffer);
        }
    }

    /**
     * Returns the fraction of zones restored at each requested hour checkpoint.
     * Every service zone contributes equally, regardless of its data weight.
     */
    public double[] calculateZoneFractionRestored(double... checkpointHours)
            throws IloException, GRBException {
        requireSolutionForAnalysis();
        int totalZones = serviceZoneCount();
        double[] restoredFractions = new double[checkpointHours.length];

        for (int checkpoint = 0; checkpoint < checkpointHours.length; checkpoint++) {
            double checkpointHoursValue = checkpointHours[checkpoint];
            if (!Double.isFinite(checkpointHoursValue) || checkpointHoursValue < 0.0) {
                throw new IllegalArgumentException(
                        "Restoration checkpoint must be a finite, nonnegative number of hours: "
                                + checkpointHoursValue
                );
            }

            double checkpointMinutes = checkpointHoursValue * 60.0;
            int restoredZones = 0;

            for (int zone = 1; zone < data.getNodeNumber(); zone++) {
                if (solver.getVariableValue(varDelta[zone]) <= checkpointMinutes + EPS) {
                    restoredZones++;
                }
            }

            restoredFractions[checkpoint] = (double) restoredZones / totalZones;
        }

        return restoredFractions;
    }

    /**
     * Returns, in hours, the earliest zone-restoration time at which each requested fraction
     * of zones has been restored. Every service zone contributes equally.
     * Targets must be fractions in [0, 1].
     */
    public double[] calculateTimeToZoneRestoration(double... targets)
            throws IloException, GRBException {
        requireSolutionForAnalysis();
        int totalZones = serviceZoneCount();

        List<Integer> zones = new ArrayList<>(totalZones);
        double[] restorationMinutes = new double[data.getNodeNumber()];

        for (int zone = 1; zone < data.getNodeNumber(); zone++) {
            zones.add(zone);
            restorationMinutes[zone] = solver.getVariableValue(varDelta[zone]);
        }

        zones.sort(Comparator.comparingDouble(zone -> restorationMinutes[zone]));

        double[] times = new double[targets.length];
        Arrays.fill(times, Double.NaN);

        for (int targetIndex = 0; targetIndex < targets.length; targetIndex++) {
            double target = targets[targetIndex];
            if (!Double.isFinite(target) || target < 0.0 || target > 1.0) {
                throw new IllegalArgumentException(
                        "Zone-restoration target must be a finite fraction in [0, 1]: " + target
                );
            }

            // Zero percent of zones is restored at time zero by definition.
            if (target <= EPS) {
                times[targetIndex] = 0.0;
            }
        }

        int restoredZones = 0;
        for (int zone : zones) {
            restoredZones++;
            double restoredFraction = (double) restoredZones / totalZones;

            for (int targetIndex = 0; targetIndex < targets.length; targetIndex++) {
                if (Double.isNaN(times[targetIndex])
                        && restoredFraction + EPS >= targets[targetIndex]) {
                    times[targetIndex] = restorationMinutes[zone] / 60.0;
                }
            }
        }

        return times;
    }

    /**
     * Backward-compatible alias. This method now returns the unweighted fraction of zones
     * restored, not a weight-based service fraction.
     */
    @Deprecated
    public double[] calculateServiceAvailable(double... checkpointHours)
            throws IloException, GRBException {
        return calculateZoneFractionRestored(checkpointHours);
    }

    /**
     * Backward-compatible alias. This method now returns unweighted zone-restoration times.
     */
    @Deprecated
    public double[] calculateTimeToServiceRestoration(double... targets)
            throws IloException, GRBException {
        return calculateTimeToZoneRestoration(targets);
    }

    /** Appends this instance's two restoration summaries to shared, multi-instance CSV files. */
    public void appendSolutionAnalysisCsv(Path outputDirectory)
            throws IOException, IloException, GRBException {
        if (!Constants.SOLUTION_ANALYSIS) {
            return;
        }
        double[] serviceAvailable = calculateZoneFractionRestored(SERVICE_CHECKPOINT_HOURS);
        double[] restorationTimes = calculateTimeToZoneRestoration(RESTORATION_TARGETS);
        String[] identity = {
                experimentFilename,
                Integer.toString(experimentCrewNumber),
                Integer.toString(experimentTowerNumber)
        };

        appendAnalysisRow(
                outputDirectory.resolve("percent-restored.csv"),
                csvLine("instance", "crews", "towers", "after_2.5_hours", "after_5_hours",
                        "after_7.5_hours", "after_10_hours"),
                csvLine(identity[0], identity[1], identity[2],
                        formatCsvDouble(100.0 * serviceAvailable[0]),
                        formatCsvDouble(100.0 * serviceAvailable[1]),
                        formatCsvDouble(100.0 * serviceAvailable[2]),
                        formatCsvDouble(100.0 * serviceAvailable[3]))
        );
        appendAnalysisRow(
                outputDirectory.resolve("time-to-restoration.csv"),
                csvLine("instance", "crews", "towers", "time_to_20_percent_hours", "time_to_40_percent_hours",
                        "time_to_60_percent_hours","time_to_80_percent_hours", "time_to_100_percent_hours"),
                csvLine(identity[0], identity[1], identity[2],
                        formatOptionalCsvDouble(restorationTimes[0]),
                        formatOptionalCsvDouble(restorationTimes[1]),
                        formatOptionalCsvDouble(restorationTimes[2]),
                        formatOptionalCsvDouble(restorationTimes[3]),
                        formatOptionalCsvDouble(restorationTimes[4])
                        )
        );
    }

    private void requireSolutionForAnalysis() throws IloException, GRBException {
        if (!solver.hasFeasibleSolution()) {
            throw new IllegalStateException("No feasible solution is available for restoration analysis.");
        }
    }

    private int serviceZoneCount() {
        int zoneCount = data.getNodeNumber() - 1;
        if (zoneCount <= 0) {
            throw new IllegalStateException(
                    "At least one service zone is required for restoration analysis."
            );
        }
        return zoneCount;
    }

    private static void appendAnalysisRow(Path outputFile, String header, String row) throws IOException {
        Path absoluteFile = outputFile.toAbsolutePath();
        Files.createDirectories(absoluteFile.getParent());
        synchronized (SOLUTION_ANALYSIS_CSV_LOCK) {
            try (FileChannel channel = FileChannel.open(absoluteFile, StandardOpenOption.CREATE,
                    StandardOpenOption.READ, StandardOpenOption.WRITE);
                 FileLock ignored = channel.lock()) {
                boolean writeHeader = channel.size() == 0L;
                channel.position(channel.size());
                if (writeHeader) {
                    writeFully(channel, header + System.lineSeparator());
                }
                writeFully(channel, row + System.lineSeparator());
                channel.force(true);
            }
        }
    }

    private Path findSolutionFile(
            int towerNum,
            Path solutionDir,
            String instanceFilename
    ) throws IOException {

        String inputFilename = Path.of(instanceFilename)
                .getFileName()
                .toString();

        String instanceName = inputFilename.contains(".")
                ? inputFilename.substring(
                0,
                inputFilename.lastIndexOf('.')
        )
                : inputFilename;

        /*
         * Expected combined-solution filename:
         *
         * <instance-name>-<crew-count>-<tower-count>.txt
         *
         * Example:
         * yATL_12-30-16_c0-10-1.txt
         */
        Pattern pattern = Pattern.compile(
                Pattern.quote(instanceName)
                        + "-\\d+-"
                        + towerNum
                        + "\\.txt"
        );

        List<Path> matches;

        if (!Files.isDirectory(solutionDir)) {
            throw new IllegalArgumentException(
                    "Solution directory does not exist: "
                            + solutionDir.toAbsolutePath()
            );
        }
        System.out.println(
                "Searching solution directory: "
                        + solutionDir.toAbsolutePath()
        );

        System.out.println(
                "Expected solution filename pattern: "
                        + pattern.pattern()
        );
        try (Stream<Path> files = Files.list(solutionDir)) {
            System.out.println("Available solution files:");

            files.filter(Files::isRegularFile)
                    .map(path -> path.getFileName().toString())
                    .sorted()
                    .forEach(name ->
                            System.out.println("\t" + name)
                    );
        }
        try (Stream<Path> files = Files.list(solutionDir)) {
            matches = files
                    .filter(Files::isRegularFile)
                    .filter(path -> pattern
                            .matcher(
                                    path.getFileName().toString()
                            )
                            .matches())
                    .sorted(
                            Comparator.comparing(
                                    path -> path
                                            .getFileName()
                                            .toString()
                            )
                    )
                    .toList();
        }

        if (matches.isEmpty()) {
            throw new IllegalArgumentException(
                    "No solution file found for instance "
                            + instanceFilename
                            + " with tower count "
                            + towerNum
                            + " in folder "
                            + solutionDir.toAbsolutePath()
                            + ". Expected a filename matching: "
                            + pattern.pattern()
            );
        }

        if (matches.size() > 1) {
            throw new IllegalArgumentException(
                    "Multiple solution files found for instance "
                            + instanceFilename
                            + " with tower count "
                            + towerNum
                            + ": "
                            + matches
            );
        }

        return matches.get(0);
    }

    /**
     * Convenience overload for experiments: solve one instance and append its metrics to the
     * shared results CSV before clearing the solver model.
     */
    public void buildAndSolveMIP(Path sharedResultsCsv)
            throws IloException, GRBException, IOException {
        build();
        Path fixedSolutionPath;
        if(Constants.FIX_CREW_ROUTES){
            fixedSolutionPath = findSolutionFile(data.getTowerNumber(),
                    Path.of("fixCrewSolutions"),
                    LocalWriter.filename
            );
            fixCrewRoutesFromFile(fixedSolutionPath, false);
        } else if(Constants.FIX_CREW_AND_TOWER_ROUTES || Constants.SOLUTION_ANALYSIS){
            fixedSolutionPath = findSolutionFile(data.getTowerNumber(),
                    Path.of("summary/multithread/solutions"),
                    LocalWriter.filename
            );
            fixCrewRoutesFromFile(fixedSolutionPath, true);
            fixTowerRoutesFromFile(fixedSolutionPath, true);
        } else
            fixedSolutionPath = null;

        try {
            boolean feasible = solveMILP();
            analyzeResults(feasible);
            appendExperimentResultCsv(sharedResultsCsv);
            if (feasible && solver.hasFeasibleSolution()) {
                printNonzeroSolution();
            }

            if(Constants.FIX_CREW_AND_TOWER_ROUTES  && !Constants.SOLUTION_ANALYSIS) {
                if (feasible && hasFeasibleSolution()) {
                    List<Path> generatedFiles = writeSolutionAnalysisCsv(
                            Path.of("summary/mipSolutionAnalysis"),
                            fixedSolutionPath.getFileName().toString()
                    );

                    generatedFiles.forEach(path ->
                            System.out.println("Wrote analysis file: " + path)
                    );
                }
            }

            if(Constants.SOLUTION_ANALYSIS){
                if (feasible && solver.hasFeasibleSolution()) {
                    appendSolutionAnalysisCsv(Path.of("summary/analyses/percentRestored"));
                }
            }
        } finally {
            clear();
        }
    }

    public void solveRoot() throws IloException, GRBException {
        System.out.println("Starting to solve the root node.");
        this.rootNodeLowerBound = Double.NaN;

        double start = System.nanoTime();
        boolean feasible = solver.solveRootNode();
        double elapsed = (System.nanoTime() - start) * 1e-9;

        if (!feasible) {
            throw new IllegalStateException(
                    "The MIP model is infeasible at the root node."
            );
        }

        this.rootNodeLowerBound = solver.getBestBound();

        System.out.println(
                "Spent " + Operators.truncate(elapsed, 3)
                        + " seconds solving the root node. Relaxation's Bound = "
                        + rootNodeLowerBound
        );
    }

    public void buildAndSolveMIP() throws IloException, GRBException {
        build();
        boolean feasible = solveMILP();
        analyzeResults(feasible);
        if (feasible && solver.hasFeasibleSolution()) {
            printNonzeroSolution();
        }
        clear();
    }

    public void clear() throws IloException, GRBException {
        solver.clearModel();
    }

    public double getRootNodeLowerBound() {
        return rootNodeLowerBound;
    }

    public double getCpuSeconds() {
        return cpuSeconds;
    }

    public double getObjVal() {
        return objVal;
    }

    public double getBestBound() {
        return bestBound;
    }

    public double getMipGap() {
        return mipGap;
    }

    private double robustRepairTimeBigM(int fromTask, int toTask) {
        return data.getHorizon()
                + data.getCrewTravelTimeMatrix()[fromTask][toTask]
                + data.getServiceTimeMatrix()[toTask]
                + data.getServiceDeviation()[toTask];
    }

    private double taskCompletionValue(int task) throws IloException, GRBException {
        if (!robustRepairTimes) {
            return solver.getVariableValue(varOmega[task]);
        }

        double max = Double.NEGATIVE_INFINITY;
        for (int g = 0; g < repairTimeBudget; g++) {
            max = Math.max(max, solver.getVariableValue(varOmegaRobust[task][g]));
        }
        return max;
    }

    private double minCrewReturnTimeForZoneOrZero(int zone) {
        if (zone == 0) return 0.0;
        double min = Double.MAX_VALUE;
        for (Integer q : data.getZone2tasks()[zone]) {
            min = Math.min(min, data.getCrewTravelTimeMatrix()[q][crewDepotEnd]);
        }
        if (min == Double.MAX_VALUE) {
            throw new IllegalStateException("Zone " + zone + " has no associated tasks.");
        }
        return min;
    }

    private double towerIncrement(int from, int to) {
        if (to == towerDepotEnd) {
            return data.getTowerTravelTimeMatrix()[from][to];
        }
        return data.getTowerTravelTimeMatrix()[from][to] + data.getPositionTimeMatrix()[to];
    }

    private ArrayList<ArrayList<Integer>> readRoutes(String filePath) throws IOException {
        ArrayList<ArrayList<Integer>> routes = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;
                if (line.startsWith("#")) continue;
                if (line.startsWith("[") && line.endsWith("]")) {
                    line = line.substring(1, line.length() - 1);
                }

                ArrayList<Integer> route = new ArrayList<>();
                if (!line.isEmpty()) {
                    for (String token : line.split(",")) {
                        route.add(Integer.parseInt(token.trim()));
                    }
                }
                if (!route.isEmpty()) routes.add(route);
            }
        }
        return routes;
    }

    private void printNonzeroArcs(String label, int[][] vars, int rows, int cols)
            throws IloException, GRBException {
        System.out.println(label + " arcs:");
        for (int i = 0; i < rows; i++) {
            for (int j = 1; j < cols; j++) {
                if (i == j || !hasVar(vars[i][j])) continue;
                double value = solver.getVariableValue(vars[i][j]);
                if (Math.abs(value) > EPS) {
                    System.out.println("\t" + label + "[" + i + " -> " + j + "] = " + value);
                }
            }
        }
    }

    private void printIfNonZero(String name, int index, int variable)
            throws IloException, GRBException {
        if (!hasVar(variable)) return;
        double value = solver.getVariableValue(variable);
        if (Math.abs(value) > EPS) {
            System.out.println("\t" + name + "[" + index + "] = " + value);
        }
    }

    private void printIfNonZero(String name, int index1, int index2, int variable)
            throws IloException, GRBException {
        if (!hasVar(variable)) return;
        double value = solver.getVariableValue(variable);
        if (Math.abs(value) > EPS) {
            System.out.println(
                    "\t" + name + "[" + index1 + "," + index2 + "] = " + value
            );
        }
    }

    private void addIfPresent(double coefficient, int variable) throws IloException, GRBException {
        if (hasVar(variable)) {
            solver.addTerm(coefficient, variable);
        }
    }

    private boolean hasVar(int variable) {
        return variable != NO_VAR;
    }

    private void fill2D(int[][] array, int value) {
        for (int[] row : array) {
            Arrays.fill(row, value);
        }
    }

    private int[] filled1D(int length, int value) {
        int[] array = new int[length];
        Arrays.fill(array, value);
        return array;
    }

    private String name(String prefix, int... indices) {
        StringBuilder sb = new StringBuilder(prefix);
        for (int index : indices) {
            sb.append(',').append(index);
        }
        return sb.toString();
    }
}
