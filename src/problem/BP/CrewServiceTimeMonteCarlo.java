package problem.BP;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

/**
 * Monte Carlo simulation for robustness analysis of crew-route solutions
 * under uncertain service times at nodes.
 *
 * A realization samples, for each uncertain node i:
 *
 *      s_i^omega ~ U[s_i, s_i + trunc(alpha_i * s_i)]
 *
 * A solution is infeasible in a scenario if at least one crew route violates
 * the route horizon, node time windows, or a user-defined feasibility check.
 *
 * This class assumes:
 *   - travel times are deterministic;
 *   - uncertainty affects only service times at non-depot nodes;
 *   - each Route stores its sequence in route.getPattern().
 */
public class CrewServiceTimeMonteCarlo {

    private static final double EPS = 1e-6;

    private final double[][] travelTime;
    private final double[] nominalServiceTime;
    private final double[] alphaService;
    private final double[] earliestStart;
    private final double[] latestStart;

    private final int startDepot;
    private final int endDepot;
    private final double routeHorizon;

    private final int simulations;
    private final double truncationUnit;
    private final Random random;

    private ExtraFeasibilityChecker extraFeasibilityChecker;

    /**
     * Optional hook for checking additional problem-specific constraints.
     *
     * For example, one can check precedence, synchronization, maximum workload,
     * crew-specific restrictions, etc.
     */
    @FunctionalInterface
    public interface ExtraFeasibilityChecker {
        boolean isFeasible(
                Route route,
                double[] sampledServiceTime,
                double[] arrivalTimes,
                double completionTime
        );
    }

    public static class SimulationResult {
        private final int simulations;
        private final int infeasibleScenarios;
        private final double risk;
        private final double averageCompletionTime;
        private final double worstCompletionTime;

        private SimulationResult(
                int simulations,
                int infeasibleScenarios,
                double averageCompletionTime,
                double worstCompletionTime
        ) {
            this.simulations = simulations;
            this.infeasibleScenarios = infeasibleScenarios;
            this.risk = simulations == 0 ? 0.0 : (double) infeasibleScenarios / simulations;
            this.averageCompletionTime = averageCompletionTime;
            this.worstCompletionTime = worstCompletionTime;
        }

        public int getSimulations() {
            return simulations;
        }

        public int getInfeasibleScenarios() {
            return infeasibleScenarios;
        }

        public double getRisk() {
            return risk;
        }

        public double getRiskPercent() {
            return 100.0 * risk;
        }

        public double getAverageCompletionTime() {
            return averageCompletionTime;
        }

        public double getWorstCompletionTime() {
            return worstCompletionTime;
        }

        @Override
        public String toString() {
            return "MonteCarloResult{" +
                    "simulations=" + simulations +
                    ", infeasibleScenarios=" + infeasibleScenarios +
                    ", risk=" + risk +
                    ", riskPercent=" + getRiskPercent() +
                    ", averageCompletionTime=" + averageCompletionTime +
                    ", worstCompletionTime=" + worstCompletionTime +
                    '}';
        }
    }

    private static class RouteEvaluation {
        final boolean feasible;
        final double completionTime;
        final double[] arrivalTimes;

        RouteEvaluation(boolean feasible, double completionTime, double[] arrivalTimes) {
            this.feasible = feasible;
            this.completionTime = completionTime;
            this.arrivalTimes = arrivalTimes;
        }
    }

    /**
     * Main constructor.
     *
     * @param travelTime deterministic travel-time matrix
     * @param nominalServiceTime nominal service time s_i for each node i
     * @param alphaService uncertainty factor alpha_i for each node i
     * @param earliestStart earliest service start time at each node, or null if unused
     * @param latestStart latest service start time at each node, or null if unused
     * @param startDepot start depot index
     * @param endDepot end depot index
     * @param routeHorizon maximum allowed route completion time
     * @param simulations number of Monte Carlo simulations, e.g., 10000
     * @param truncationUnit truncation unit. Use 1.0 for integer truncation;
     *                       use 0.1 for one-decimal truncation;
     *                       use 0.0 if no truncation is desired.
     * @param seed random seed
     */
    public CrewServiceTimeMonteCarlo(
            double[][] travelTime,
            double[] nominalServiceTime,
            double[] alphaService,
            double[] earliestStart,
            double[] latestStart,
            int startDepot,
            int endDepot,
            double routeHorizon,
            int simulations,
            double truncationUnit,
            long seed
    ) {
        this.travelTime = travelTime;
        this.nominalServiceTime = nominalServiceTime;
        this.alphaService = alphaService;
        this.earliestStart = earliestStart;
        this.latestStart = latestStart;
        this.startDepot = startDepot;
        this.endDepot = endDepot;
        this.routeHorizon = routeHorizon;
        this.simulations = simulations;
        this.truncationUnit = truncationUnit;
        this.random = new Random(seed);

        validateInput();
    }

    /**
     * Convenience constructor for a common scalar uncertainty factor alpha.
     */
    public CrewServiceTimeMonteCarlo(
            double[][] travelTime,
            double[] nominalServiceTime,
            double alphaService,
            double[] earliestStart,
            double[] latestStart,
            int startDepot,
            int endDepot,
            double routeHorizon,
            int simulations,
            double truncationUnit,
            long seed
    ) {
        this(
                travelTime,
                nominalServiceTime,
                createConstantAlpha(nominalServiceTime.length, alphaService),
                earliestStart,
                latestStart,
                startDepot,
                endDepot,
                routeHorizon,
                simulations,
                truncationUnit,
                seed
        );
    }

    public void setExtraFeasibilityChecker(ExtraFeasibilityChecker extraFeasibilityChecker) {
        this.extraFeasibilityChecker = extraFeasibilityChecker;
    }

    /**
     * Simulates a crew solution represented by selected crew routes.
     *
     * A scenario is counted as infeasible if at least one selected route is infeasible.
     */
    /**
     * Simulates a complete solution represented directly as an ArrayList of routes.
     *
     * A scenario is infeasible if at least one route in the solution is infeasible.
     *
     * @param completeSolution complete integer solution containing the selected routes
     * @return aggregated Monte Carlo simulation results
     */
    public SimulationResult simulate(ArrayList<Route> completeSolution) {
        if (completeSolution == null) {
            throw new IllegalArgumentException(
                    "Complete solution cannot be null."
            );
        }

        if (completeSolution.isEmpty()) {
            return new SimulationResult(
                    simulations,
                    0,
                    0.0,
                    0.0
            );
        }

        int infeasible = 0;
        double sumScenarioCompletion = 0.0;
        double worstScenarioCompletion = 0.0;

        for (int omega = 0; omega < simulations; omega++) {
            double[] sampledServiceTime = sampleServiceTimes();

            boolean scenarioFeasible = true;
            double scenarioCompletion = 0.0;

            for (Route route : completeSolution) {
                if (route == null) {
                    throw new IllegalArgumentException(
                            "Complete solution cannot contain null routes."
                    );
                }

                RouteEvaluation evaluation = evaluateRoute(
                        route,
                        sampledServiceTime
                );

                scenarioCompletion = Math.max(
                        scenarioCompletion,
                        evaluation.completionTime
                );

                if (!evaluation.feasible) {
                    scenarioFeasible = false;
                    break;
                }
            }

            if (!scenarioFeasible) {
                infeasible++;
            }

            sumScenarioCompletion += scenarioCompletion;

            worstScenarioCompletion = Math.max(
                    worstScenarioCompletion,
                    scenarioCompletion
            );
        }

        double averageCompletion =
                sumScenarioCompletion / simulations;

        return new SimulationResult(
                simulations,
                infeasible,
                averageCompletion,
                worstScenarioCompletion
        );
    }

    /**
     * Use this to simulate directly from active VarR objects.
     */
    public SimulationResult simulateFromVarR(
            Collection<VarR> activeCrewColumns,
            double threshold
    ) {
        ArrayList<Route> selectedRoutes = new ArrayList<>();

        if (activeCrewColumns != null) {
            for (VarR var : activeCrewColumns) {
                if (var == null || var.getRoute() == null) {
                    continue;
                }

                if (var.getValue() > threshold) {
                    selectedRoutes.add(var.getRoute());
                }
            }
        }

        return simulate(selectedRoutes);
    }

    /**
     * Price of robustness:
     *
     *      PoR = (z_robust - z_nominal) / z_nominal * 100%.
     */
    public static double computePriceOfRobustness(
            double robustObjective,
            double nominalObjective
    ) {
        if (Math.abs(nominalObjective) <= EPS) {
            throw new IllegalArgumentException(
                    "Nominal objective is zero; price of robustness is undefined."
            );
        }

        return 100.0 * (robustObjective - nominalObjective) / nominalObjective;
    }

    private RouteEvaluation evaluateRoute(
            Route route,
            double[] sampledServiceTime
    ) {
        List<Integer> pattern = route.getPattern();

        if (pattern == null || pattern.isEmpty()) {
            return new RouteEvaluation(true, 0.0, new double[0]);
        }

        double[] arrivals = new double[pattern.size()];
        double time = 0.0;

        int first = pattern.get(0);
        if (first != startDepot) {
            return new RouteEvaluation(false, 0.0, arrivals);
        }

        arrivals[0] = 0.0;

        for (int k = 1; k < pattern.size(); k++) {
            int previous = pattern.get(k - 1);
            int node = pattern.get(k);

            time += travelTime[previous][node];
            arrivals[k] = time;

            if (hasLatestStart(node) && time > latestStart[node] + EPS) {
                return new RouteEvaluation(false, time, arrivals);
            }

            if (hasEarliestStart(node) && time < earliestStart[node]) {
                time = earliestStart[node];
            }

            if (!isDepot(node)) {
                time += sampledServiceTime[node];
            }
        }

        int last = pattern.get(pattern.size() - 1);
        if (last != endDepot) {
            return new RouteEvaluation(false, time, arrivals);
        }

        if (time > routeHorizon + EPS) {
            return new RouteEvaluation(false, time, arrivals);
        }

        if (extraFeasibilityChecker != null) {
            boolean extraFeasible = extraFeasibilityChecker.isFeasible(
                    route,
                    sampledServiceTime,
                    arrivals,
                    time
            );

            if (!extraFeasible) {
                return new RouteEvaluation(false, time, arrivals);
            }
        }

        return new RouteEvaluation(true, time, arrivals);
    }

    private double[] sampleServiceTimes() {
        double[] sampled = nominalServiceTime.clone();

        for (int i = 0; i < sampled.length; i++) {
            if (isDepot(i)) {
                sampled[i] = 0.0;
                continue;
            }

            double deviation = computeDeviation(i);

            if (deviation <= EPS) {
                sampled[i] = nominalServiceTime[i];
            } else {
                sampled[i] = nominalServiceTime[i] + random.nextDouble() * deviation;
            }
        }

        return sampled;
    }

    private double computeDeviation(int i) {
        double rawDeviation = alphaService[i] * nominalServiceTime[i];

        if (rawDeviation <= EPS) {
            return 0.0;
        }

        if (truncationUnit <= EPS) {
            return rawDeviation;
        }

        return truncationUnit * Math.floor(rawDeviation / truncationUnit);
    }

    private boolean isDepot(int node) {
        return node == startDepot || node == endDepot;
    }

    private boolean hasEarliestStart(int node) {
        return earliestStart != null
                && node >= 0
                && node < earliestStart.length
                && !Double.isInfinite(earliestStart[node]);
    }

    private boolean hasLatestStart(int node) {
        return latestStart != null
                && node >= 0
                && node < latestStart.length
                && !Double.isInfinite(latestStart[node]);
    }

    private void validateInput() {
        if (travelTime == null || nominalServiceTime == null || alphaService == null) {
            throw new IllegalArgumentException("Travel, service, and alpha arrays cannot be null.");
        }

        if (nominalServiceTime.length != alphaService.length) {
            throw new IllegalArgumentException(
                    "nominalServiceTime and alphaService must have the same length."
            );
        }

        if (travelTime.length < nominalServiceTime.length) {
            throw new IllegalArgumentException(
                    "travelTime matrix is smaller than the service-time array."
            );
        }

        for (int i = 0; i < travelTime.length; i++) {
            if (travelTime[i] == null || travelTime[i].length < nominalServiceTime.length) {
                throw new IllegalArgumentException("travelTime must be a square-compatible matrix.");
            }
        }

        if (simulations <= 0) {
            throw new IllegalArgumentException("Number of simulations must be positive.");
        }

        if (routeHorizon < 0.0) {
            throw new IllegalArgumentException("Route horizon cannot be negative.");
        }
    }

    private static double[] createConstantAlpha(int length, double alpha) {
        double[] result = new double[length];

        for (int i = 0; i < length; i++) {
            result[i] = alpha;
        }

        return result;
    }
}