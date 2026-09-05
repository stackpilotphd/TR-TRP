package problem.milp.genericSolverTools;

public enum SolverType {
    CPLEX, GUROBI;

    public static SolverType fromString(String solver) {
        return switch (solver.toLowerCase()) {
            case "cplex" -> CPLEX;
            case "gurobi" -> GUROBI;
            default -> throw new IllegalArgumentException(
                    "Unknown solver: " + solver);
        };
    }
}
