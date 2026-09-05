package problem.analyses;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.function.ToDoubleFunction;

/**
 * Accumulates statistics describing how closely exact-solution routes follow
 * a non-increasing task-priority order.
 *
 * Assumption:
 * A larger priority value represents a higher-priority task.
 */
public final class PriorityOrderStatistics {

    private static final Map<String, InstanceMetrics> INSTANCES =
            new LinkedHashMap<>();

    private PriorityOrderStatistics() {
    }

    public enum ObjectiveSense {
        MINIMIZE,
        MAXIMIZE
    }

    /**
     * Calculates a positive percentage when the heuristic is worse than the
     * exact solution.
     *
     * For minimization:
     *     100 * (heuristic - exact) / abs(exact)
     *
     * For maximization:
     *     100 * (exact - heuristic) / abs(exact)
     */
    public static double calculateGapPercent(
            double exactObjective,
            double heuristicObjective,
            ObjectiveSense objectiveSense
    ) {
        Objects.requireNonNull(objectiveSense, "objectiveSense");

        if (!Double.isFinite(exactObjective)
                || !Double.isFinite(heuristicObjective)) {
            throw new IllegalArgumentException(
                    "Objective values must be finite."
            );
        }

        if (exactObjective == 0.0) {
            throw new IllegalArgumentException(
                    "Cannot calculate a relative gap when the exact "
                            + "objective is zero."
            );
        }

        double absoluteDifference;

        if (objectiveSense == ObjectiveSense.MINIMIZE) {
            absoluteDifference = heuristicObjective - exactObjective;
        } else {
            absoluteDifference = exactObjective - heuristicObjective;
        }

        return 100.0 * absoluteDifference / Math.abs(exactObjective);
    }

    /**
     * Computes priority-order statistics for one route.
     */
    public static RouteMetrics analyzeRoute(
            List<Integer> route,
            int[] priority
    ) {
        Objects.requireNonNull(route, "route");
        Objects.requireNonNull(priority, "priority");

        /*
         * A route is assumed to contain:
         *
         * [start depot, task 1, ..., task n, end depot]
         */
        int taskCount = Math.max(0, route.size() - 2);

        if (taskCount == 0) {
            return new RouteMetrics(
                    0,
                    true,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0
            );
        }

        validateTaskIndices(route, priority);

        long adjacentPairCount = 0;
        long adjacentViolationCount = 0;
        long adjacentViolationMagnitude = 0;
        int maximumAdjacentViolationMagnitude = 0;

        /*
         * Adjacent violation:
         *
         * Current priority < next priority
         *
         * Example:
         * [8, 5, 7, 3]
         *
         * The transition 5 -> 7 is a violation with magnitude 2.
         */
        for (int routeIndex = 1;
             routeIndex < route.size() - 2;
             routeIndex++) {

            int currentTask = route.get(routeIndex);
            int nextTask = route.get(routeIndex + 1);

            int currentPriority = priority[currentTask];
            int nextPriority = priority[nextTask];

            adjacentPairCount++;

            if (currentPriority < nextPriority) {
                int magnitude = nextPriority - currentPriority;

                adjacentViolationCount++;
                adjacentViolationMagnitude += magnitude;

                maximumAdjacentViolationMagnitude = Math.max(
                        maximumAdjacentViolationMagnitude,
                        magnitude
                );
            }
        }

        long pairwisePairCount = 0;
        long pairwiseInversionCount = 0;
        long pairwiseInversionMagnitude = 0;
        int maximumPairwiseInversionMagnitude = 0;

        /*
         * Pairwise inversion:
         *
         * For every earlier-later task pair (i, j), i < j,
         * count an inversion when priority[i] < priority[j].
         */
        for (int earlierIndex = 1;
             earlierIndex < route.size() - 2;
             earlierIndex++) {

            int earlierTask = route.get(earlierIndex);
            int earlierPriority = priority[earlierTask];

            for (int laterIndex = earlierIndex + 1;
                 laterIndex < route.size() - 1;
                 laterIndex++) {

                int laterTask = route.get(laterIndex);
                int laterPriority = priority[laterTask];

                pairwisePairCount++;

                if (earlierPriority < laterPriority) {
                    int magnitude = laterPriority - earlierPriority;

                    pairwiseInversionCount++;
                    pairwiseInversionMagnitude += magnitude;

                    maximumPairwiseInversionMagnitude = Math.max(
                            maximumPairwiseInversionMagnitude,
                            magnitude
                    );
                }
            }
        }

        boolean nonIncreasing =
                adjacentViolationCount == 0;

        return new RouteMetrics(
                taskCount,
                nonIncreasing,
                adjacentPairCount,
                adjacentViolationCount,
                adjacentViolationMagnitude,
                maximumAdjacentViolationMagnitude,
                pairwisePairCount,
                pairwiseInversionCount,
                pairwiseInversionMagnitude,
                maximumPairwiseInversionMagnitude
        );
    }

    private static void validateTaskIndices(
            List<Integer> route,
            int[] priority
    ) {
        for (int routeIndex = 1;
             routeIndex < route.size() - 1;
             routeIndex++) {

            int task = route.get(routeIndex);

            if (task < 0 || task >= priority.length) {
                throw new IllegalArgumentException(
                        "Task index "
                                + task
                                + " is outside the priority array. "
                                + "Route: "
                                + route
                );
            }
        }
    }

    /**
     * Records or replaces the statistics for one instance.
     *
     * Replacing an existing instance prevents accidental double counting
     * when the same instance is analyzed twice.
     */
    public static synchronized void recordInstance(
            InstanceMetrics metrics
    ) {
        Objects.requireNonNull(metrics, "metrics");
        INSTANCES.put(metrics.getInstanceId(), metrics);
    }

    public static synchronized List<InstanceMetrics> getInstanceMetrics() {
        return new ArrayList<>(INSTANCES.values());
    }

    /**
     * Call once before analyzing a new complete experiment.
     */
    public static synchronized void reset() {
        INSTANCES.clear();
    }

    /**
     * Adds or replaces the heuristic gap after the instance has already
     * been analyzed.
     */
    public static synchronized void setHeuristicGapPercent(
            String instanceId,
            double heuristicGapPercent
    ) {
        InstanceMetrics existing = INSTANCES.get(instanceId);

        if (existing == null) {
            throw new IllegalArgumentException(
                    "No priority metrics recorded for instance "
                            + instanceId
            );
        }

        INSTANCES.put(
                instanceId,
                existing.withHeuristicGapPercent(heuristicGapPercent)
        );
    }

    public static void printSummary() {
        printSummary(System.out);
    }

    public static void printSummary(PrintStream output) {
        Objects.requireNonNull(output, "output");

        List<InstanceMetrics> instances = getInstanceMetrics();

        output.println();
        output.println("Priority-Order Statistics:");

        if (instances.isEmpty()) {
            output.println("\tNo instances have been recorded.");
            return;
        }

        long totalRoutes = 0;
        long totalTasks = 0;
        long totalNonIncreasingRoutes = 0;

        long totalAdjacentPairs = 0;
        long totalAdjacentViolations = 0;
        long totalAdjacentViolationMagnitude = 0;
        int maximumAdjacentViolationMagnitude = 0;

        long totalPairwisePairs = 0;
        long totalPairwiseInversions = 0;
        long totalPairwiseInversionMagnitude = 0;
        int maximumPairwiseInversionMagnitude = 0;

        double sumRouteAdjacentViolationRates = 0.0;
        long routesWithAdjacentPairs = 0;

        double sumRoutePairwiseInversionRates = 0.0;
        long routesWithPairwisePairs = 0;

        long totalSharedTasks = 0;
        int globalMaximumRouteLength = 0;

        for (InstanceMetrics instance : instances) {
            totalRoutes += instance.routeCount;
            totalTasks += instance.taskCount;
            totalNonIncreasingRoutes +=
                    instance.nonIncreasingRouteCount;

            totalAdjacentPairs += instance.adjacentPairCount;
            totalAdjacentViolations +=
                    instance.adjacentViolationCount;
            totalAdjacentViolationMagnitude +=
                    instance.adjacentViolationMagnitude;

            maximumAdjacentViolationMagnitude = Math.max(
                    maximumAdjacentViolationMagnitude,
                    instance.maximumAdjacentViolationMagnitude
            );

            totalPairwisePairs += instance.pairwisePairCount;
            totalPairwiseInversions +=
                    instance.pairwiseInversionCount;
            totalPairwiseInversionMagnitude +=
                    instance.pairwiseInversionMagnitude;

            maximumPairwiseInversionMagnitude = Math.max(
                    maximumPairwiseInversionMagnitude,
                    instance.maximumPairwiseInversionMagnitude
            );

            sumRouteAdjacentViolationRates +=
                    instance.sumRouteAdjacentViolationRates;

            routesWithAdjacentPairs +=
                    instance.routesWithAdjacentPairs;

            sumRoutePairwiseInversionRates +=
                    instance.sumRoutePairwiseInversionRates;

            routesWithPairwisePairs +=
                    instance.routesWithPairwisePairs;

            totalSharedTasks += instance.sharedTaskCount;

            globalMaximumRouteLength = Math.max(
                    globalMaximumRouteLength,
                    instance.maximumRouteLength
            );
        }

        long routesWithViolations =
                totalRoutes - totalNonIncreasingRoutes;

        double nonIncreasingRouteRate =
                safeRate(totalNonIncreasingRoutes, totalRoutes);

        double routeViolationRate =
                safeRate(routesWithViolations, totalRoutes);

        double pooledAdjacentViolationRate =
                safeRate(
                        totalAdjacentViolations,
                        totalAdjacentPairs
                );

        double meanRouteAdjacentViolationRate =
                safeRate(
                        sumRouteAdjacentViolationRates,
                        routesWithAdjacentPairs
                );

        double pooledPairwiseInversionRate =
                safeRate(
                        totalPairwiseInversions,
                        totalPairwisePairs
                );

        double meanRoutePairwiseInversionRate =
                safeRate(
                        sumRoutePairwiseInversionRates,
                        routesWithPairwisePairs
                );

        double averageAdjacentViolationMagnitude =
                safeRate(
                        totalAdjacentViolationMagnitude,
                        totalAdjacentViolations
                );

        double averagePairwiseInversionMagnitude =
                safeRate(
                        totalPairwiseInversionMagnitude,
                        totalPairwiseInversions
                );

        double averageRouteLength =
                safeRate(totalTasks, totalRoutes);

        output.printf(
                Locale.US,
                "\tInstances: %d%n",
                instances.size()
        );

        output.printf(
                Locale.US,
                "\tRoutes: %d%n",
                totalRoutes
        );

        output.printf(
                Locale.US,
                "\tTasks in routes: %d%n",
                totalTasks
        );

        output.printf(
                Locale.US,
                "\tAverage route length: %.4f tasks%n",
                averageRouteLength
        );

        output.printf(
                Locale.US,
                "\tMaximum route length: %d tasks%n",
                globalMaximumRouteLength
        );

        output.printf(
                Locale.US,
                "\tNon-increasing priority routes: "
                        + "%d/%d (%.2f%%)%n",
                totalNonIncreasingRoutes,
                totalRoutes,
                100.0 * nonIncreasingRouteRate
        );

        output.printf(
                Locale.US,
                "\tRoutes containing at least one priority violation: "
                        + "%d/%d (%.2f%%)%n",
                routesWithViolations,
                totalRoutes,
                100.0 * routeViolationRate
        );

        output.println();
        output.println("\tAdjacent priority violations:");

        output.printf(
                Locale.US,
                "\t\tOverall adjacent-pair violation rate: "
                        + "%d/%d (%.4f%%)%n",
                totalAdjacentViolations,
                totalAdjacentPairs,
                100.0 * pooledAdjacentViolationRate
        );

        output.printf(
                Locale.US,
                "\t\tMean route-level violation rate: %.4f%%%n",
                100.0 * meanRouteAdjacentViolationRate
        );

        output.printf(
                Locale.US,
                "\t\tAverage violation magnitude: %.4f%n",
                averageAdjacentViolationMagnitude
        );

        output.printf(
                Locale.US,
                "\t\tMaximum violation magnitude: %d%n",
                maximumAdjacentViolationMagnitude
        );

        output.println();
        output.println("\tPairwise priority inversions:");

        output.printf(
                Locale.US,
                "\t\tOverall pairwise inversion rate: "
                        + "%d/%d (%.4f%%)%n",
                totalPairwiseInversions,
                totalPairwisePairs,
                100.0 * pooledPairwiseInversionRate
        );

        output.printf(
                Locale.US,
                "\t\tMean route-level inversion rate: %.4f%%%n",
                100.0 * meanRoutePairwiseInversionRate
        );

        output.printf(
                Locale.US,
                "\t\tAverage inversion magnitude: %.4f%n",
                averagePairwiseInversionMagnitude
        );

        output.printf(
                Locale.US,
                "\t\tMaximum inversion magnitude: %d%n",
                maximumPairwiseInversionMagnitude
        );

        output.printf(
                Locale.US,
                "%n\tShared tasks across instances: %d%n",
                totalSharedTasks
        );

        printGapRelationships(output, instances);
    }

    private static void printGapRelationships(
            PrintStream output,
            List<InstanceMetrics> instances
    ) {
        List<InstanceMetrics> instancesWithGap =
                instances.stream()
                        .filter(InstanceMetrics::hasHeuristicGap)
                        .toList();

        output.println();
        output.println("\tHeuristic-gap relationships:");

        if (instancesWithGap.size() < 2) {
            output.println(
                    "\t\tAt least two instances with heuristic gaps "
                            + "are required."
            );
            return;
        }

        double adjacentCorrelation = pearsonCorrelation(
                instancesWithGap,
                InstanceMetrics::getHeuristicGapPercent,
                InstanceMetrics::getAdjacentViolationRate
        );

        double pairwiseCorrelation = pearsonCorrelation(
                instancesWithGap,
                InstanceMetrics::getHeuristicGapPercent,
                InstanceMetrics::getPairwiseInversionRate
        );

        double violatingRouteCorrelation = pearsonCorrelation(
                instancesWithGap,
                InstanceMetrics::getHeuristicGapPercent,
                InstanceMetrics::getRouteViolationRate
        );

        output.printf(
                Locale.US,
                "\t\tInstances with a recorded gap: %d%n",
                instancesWithGap.size()
        );

        printCorrelation(
                output,
                "Gap versus adjacent violation rate",
                adjacentCorrelation
        );

        printCorrelation(
                output,
                "Gap versus pairwise inversion rate",
                pairwiseCorrelation
        );

        printCorrelation(
                output,
                "Gap versus proportion of violating routes",
                violatingRouteCorrelation
        );

        output.println(
                "\t\tPearson correlation describes association, "
                        + "not causation."
        );
    }

    private static void printCorrelation(
            PrintStream output,
            String label,
            double correlation
    ) {
        if (Double.isNaN(correlation)) {
            output.printf(
                    "\t\t%s: undefined due to zero variation%n",
                    label
            );
        } else {
            output.printf(
                    Locale.US,
                    "\t\t%s: r = %.4f%n",
                    label,
                    correlation
            );
        }
    }

    private static double pearsonCorrelation(
            List<InstanceMetrics> instances,
            ToDoubleFunction<InstanceMetrics> xFunction,
            ToDoubleFunction<InstanceMetrics> yFunction
    ) {
        List<Double> xValues = new ArrayList<>();
        List<Double> yValues = new ArrayList<>();

        for (InstanceMetrics instance : instances) {
            double x = xFunction.applyAsDouble(instance);
            double y = yFunction.applyAsDouble(instance);

            if (Double.isFinite(x) && Double.isFinite(y)) {
                xValues.add(x);
                yValues.add(y);
            }
        }

        int observationCount = xValues.size();

        if (observationCount < 2) {
            return Double.NaN;
        }

        double xMean = xValues.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);

        double yMean = yValues.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(Double.NaN);

        double numerator = 0.0;
        double xSquaredDeviation = 0.0;
        double ySquaredDeviation = 0.0;

        for (int index = 0; index < observationCount; index++) {
            double xDeviation = xValues.get(index) - xMean;
            double yDeviation = yValues.get(index) - yMean;

            numerator += xDeviation * yDeviation;
            xSquaredDeviation += xDeviation * xDeviation;
            ySquaredDeviation += yDeviation * yDeviation;
        }

        double denominator = Math.sqrt(
                xSquaredDeviation * ySquaredDeviation
        );

        if (denominator == 0.0) {
            return Double.NaN;
        }

        return numerator / denominator;
    }

    /**
     * Writes one row per instance so the relationship between heuristic gap
     * and priority violations can be examined externally.
     */
    public static void writeInstanceCsv(Path outputFile)
            throws IOException {

        Objects.requireNonNull(outputFile, "outputFile");

        Path parent = outputFile.getParent();

        if (parent != null) {
            Files.createDirectories(parent);
        }

        List<InstanceMetrics> instances = getInstanceMetrics();

        instances.sort(
                Comparator.comparing(
                        InstanceMetrics::getInstanceId
                )
        );

        try (
                BufferedWriter writer =
                        Files.newBufferedWriter(outputFile);
                PrintWriter output =
                        new PrintWriter(writer)
        ) {
            output.println(
                    "instance_id,"
                            + "heuristic_gap_percent,"
                            + "route_count,"
                            + "task_count,"
                            + "non_increasing_route_count,"
                            + "non_increasing_route_percent,"
                            + "violating_route_percent,"
                            + "adjacent_pair_count,"
                            + "adjacent_violation_count,"
                            + "adjacent_violation_rate_percent,"
                            + "mean_route_adjacent_violation_rate_percent,"
                            + "average_adjacent_violation_magnitude,"
                            + "maximum_adjacent_violation_magnitude,"
                            + "pairwise_pair_count,"
                            + "pairwise_inversion_count,"
                            + "pairwise_inversion_rate_percent,"
                            + "mean_route_pairwise_inversion_rate_percent,"
                            + "average_pairwise_inversion_magnitude,"
                            + "maximum_pairwise_inversion_magnitude,"
                            + "shared_task_count,"
                            + "average_route_length,"
                            + "maximum_route_length"
            );

            for (InstanceMetrics instance : instances) {
                output.print(escapeCsv(instance.instanceId));
                output.print(',');

                if (instance.hasHeuristicGap()) {
                    output.print(format(instance.heuristicGapPercent));
                }

                output.print(',');
                output.print(instance.routeCount);
                output.print(',');
                output.print(instance.taskCount);
                output.print(',');
                output.print(instance.nonIncreasingRouteCount);
                output.print(',');
                output.print(format(
                        100.0 * instance.getNonIncreasingRouteRate()
                ));
                output.print(',');
                output.print(format(
                        100.0 * instance.getRouteViolationRate()
                ));
                output.print(',');
                output.print(instance.adjacentPairCount);
                output.print(',');
                output.print(instance.adjacentViolationCount);
                output.print(',');
                output.print(format(
                        100.0 * instance.getAdjacentViolationRate()
                ));
                output.print(',');
                output.print(format(
                        100.0
                                * instance
                                .getMeanRouteAdjacentViolationRate()
                ));
                output.print(',');
                output.print(format(
                        instance.getAverageAdjacentViolationMagnitude()
                ));
                output.print(',');
                output.print(
                        instance.maximumAdjacentViolationMagnitude
                );
                output.print(',');
                output.print(instance.pairwisePairCount);
                output.print(',');
                output.print(instance.pairwiseInversionCount);
                output.print(',');
                output.print(format(
                        100.0 * instance.getPairwiseInversionRate()
                ));
                output.print(',');
                output.print(format(
                        100.0
                                * instance
                                .getMeanRoutePairwiseInversionRate()
                ));
                output.print(',');
                output.print(format(
                        instance.getAveragePairwiseInversionMagnitude()
                ));
                output.print(',');
                output.print(
                        instance.maximumPairwiseInversionMagnitude
                );
                output.print(',');
                output.print(instance.sharedTaskCount);
                output.print(',');
                output.print(format(instance.averageRouteLength));
                output.print(',');
                output.println(instance.maximumRouteLength);
            }
        }
    }

    private static String format(double value) {
        if (!Double.isFinite(value)) {
            return "";
        }

        return String.format(Locale.US, "%.8f", value);
    }

    private static String escapeCsv(String value) {
        if (value == null) {
            return "";
        }

        if (value.contains(",")
                || value.contains("\"")
                || value.contains("\n")) {

            return "\""
                    + value.replace("\"", "\"\"")
                    + "\"";
        }

        return value;
    }

    private static double safeRate(
            long numerator,
            long denominator
    ) {
        if (denominator == 0) {
            return 0.0;
        }

        return (double) numerator / denominator;
    }

    private static double safeRate(
            double numerator,
            long denominator
    ) {
        if (denominator == 0) {
            return 0.0;
        }

        return numerator / denominator;
    }

    /**
     * Route-level statistics.
     */
    public static final class RouteMetrics {

        private final int taskCount;
        private final boolean nonIncreasing;

        private final long adjacentPairCount;
        private final long adjacentViolationCount;
        private final long adjacentViolationMagnitude;
        private final int maximumAdjacentViolationMagnitude;

        private final long pairwisePairCount;
        private final long pairwiseInversionCount;
        private final long pairwiseInversionMagnitude;
        private final int maximumPairwiseInversionMagnitude;

        private RouteMetrics(
                int taskCount,
                boolean nonIncreasing,
                long adjacentPairCount,
                long adjacentViolationCount,
                long adjacentViolationMagnitude,
                int maximumAdjacentViolationMagnitude,
                long pairwisePairCount,
                long pairwiseInversionCount,
                long pairwiseInversionMagnitude,
                int maximumPairwiseInversionMagnitude
        ) {
            this.taskCount = taskCount;
            this.nonIncreasing = nonIncreasing;
            this.adjacentPairCount = adjacentPairCount;
            this.adjacentViolationCount =
                    adjacentViolationCount;
            this.adjacentViolationMagnitude =
                    adjacentViolationMagnitude;
            this.maximumAdjacentViolationMagnitude =
                    maximumAdjacentViolationMagnitude;
            this.pairwisePairCount = pairwisePairCount;
            this.pairwiseInversionCount =
                    pairwiseInversionCount;
            this.pairwiseInversionMagnitude =
                    pairwiseInversionMagnitude;
            this.maximumPairwiseInversionMagnitude =
                    maximumPairwiseInversionMagnitude;
        }

        public int getTaskCount() {
            return taskCount;
        }

        public boolean isNonIncreasing() {
            return nonIncreasing;
        }

        public long getAdjacentPairCount() {
            return adjacentPairCount;
        }

        public long getAdjacentViolationCount() {
            return adjacentViolationCount;
        }

        public long getAdjacentViolationMagnitude() {
            return adjacentViolationMagnitude;
        }

        public int getMaximumAdjacentViolationMagnitude() {
            return maximumAdjacentViolationMagnitude;
        }

        public long getPairwisePairCount() {
            return pairwisePairCount;
        }

        public long getPairwiseInversionCount() {
            return pairwiseInversionCount;
        }

        public long getPairwiseInversionMagnitude() {
            return pairwiseInversionMagnitude;
        }

        public int getMaximumPairwiseInversionMagnitude() {
            return maximumPairwiseInversionMagnitude;
        }

        public double getAdjacentViolationRate() {
            return safeRate(
                    adjacentViolationCount,
                    adjacentPairCount
            );
        }

        public double getPairwiseInversionRate() {
            return safeRate(
                    pairwiseInversionCount,
                    pairwisePairCount
            );
        }
    }

    /**
     * Builds the statistics for one instance while its routes are read.
     */
    public static final class InstanceBuilder {

        private final String instanceId;
        private final int sharedTaskCount;
        private final Double heuristicGapPercent;

        private int routeCount;
        private long taskCount;
        private int nonIncreasingRouteCount;

        private long adjacentPairCount;
        private long adjacentViolationCount;
        private long adjacentViolationMagnitude;
        private int maximumAdjacentViolationMagnitude;

        private long pairwisePairCount;
        private long pairwiseInversionCount;
        private long pairwiseInversionMagnitude;
        private int maximumPairwiseInversionMagnitude;

        private double sumRouteAdjacentViolationRates;
        private long routesWithAdjacentPairs;

        private double sumRoutePairwiseInversionRates;
        private long routesWithPairwisePairs;

        private int maximumRouteLength;

        public InstanceBuilder(
                String instanceId,
                int sharedTaskCount,
                Double heuristicGapPercent
        ) {
            this.instanceId =
                    Objects.requireNonNull(
                            instanceId,
                            "instanceId"
                    );

            this.sharedTaskCount = sharedTaskCount;

            if (heuristicGapPercent != null
                    && !Double.isFinite(heuristicGapPercent)) {
                throw new IllegalArgumentException(
                        "Heuristic gap must be finite."
                );
            }

            this.heuristicGapPercent = heuristicGapPercent;
        }

        public void addRoute(RouteMetrics routeMetrics) {
            Objects.requireNonNull(
                    routeMetrics,
                    "routeMetrics"
            );

            /*
             * Empty depot-to-depot routes are not treated as task routes.
             */
            if (routeMetrics.taskCount == 0) {
                return;
            }

            routeCount++;
            taskCount += routeMetrics.taskCount;

            maximumRouteLength = Math.max(
                    maximumRouteLength,
                    routeMetrics.taskCount
            );

            if (routeMetrics.nonIncreasing) {
                nonIncreasingRouteCount++;
            }

            adjacentPairCount +=
                    routeMetrics.adjacentPairCount;

            adjacentViolationCount +=
                    routeMetrics.adjacentViolationCount;

            adjacentViolationMagnitude +=
                    routeMetrics.adjacentViolationMagnitude;

            maximumAdjacentViolationMagnitude = Math.max(
                    maximumAdjacentViolationMagnitude,
                    routeMetrics.maximumAdjacentViolationMagnitude
            );

            pairwisePairCount +=
                    routeMetrics.pairwisePairCount;

            pairwiseInversionCount +=
                    routeMetrics.pairwiseInversionCount;

            pairwiseInversionMagnitude +=
                    routeMetrics.pairwiseInversionMagnitude;

            maximumPairwiseInversionMagnitude = Math.max(
                    maximumPairwiseInversionMagnitude,
                    routeMetrics.maximumPairwiseInversionMagnitude
            );

            if (routeMetrics.adjacentPairCount > 0) {
                routesWithAdjacentPairs++;

                sumRouteAdjacentViolationRates +=
                        routeMetrics.getAdjacentViolationRate();
            }

            if (routeMetrics.pairwisePairCount > 0) {
                routesWithPairwisePairs++;

                sumRoutePairwiseInversionRates +=
                        routeMetrics.getPairwiseInversionRate();
            }
        }

        public InstanceMetrics build() {
            return new InstanceMetrics(this);
        }
    }

    /**
     * Immutable instance-level statistics.
     */
    public static final class InstanceMetrics {

        private final String instanceId;
        private final Double heuristicGapPercent;

        private final int routeCount;
        private final long taskCount;
        private final int nonIncreasingRouteCount;

        private final long adjacentPairCount;
        private final long adjacentViolationCount;
        private final long adjacentViolationMagnitude;
        private final int maximumAdjacentViolationMagnitude;

        private final long pairwisePairCount;
        private final long pairwiseInversionCount;
        private final long pairwiseInversionMagnitude;
        private final int maximumPairwiseInversionMagnitude;

        private final double sumRouteAdjacentViolationRates;
        private final long routesWithAdjacentPairs;

        private final double sumRoutePairwiseInversionRates;
        private final long routesWithPairwisePairs;

        private final int sharedTaskCount;
        private final double averageRouteLength;
        private final int maximumRouteLength;

        private InstanceMetrics(InstanceBuilder builder) {
            this.instanceId = builder.instanceId;
            this.heuristicGapPercent =
                    builder.heuristicGapPercent;

            this.routeCount = builder.routeCount;
            this.taskCount = builder.taskCount;
            this.nonIncreasingRouteCount =
                    builder.nonIncreasingRouteCount;

            this.adjacentPairCount =
                    builder.adjacentPairCount;
            this.adjacentViolationCount =
                    builder.adjacentViolationCount;
            this.adjacentViolationMagnitude =
                    builder.adjacentViolationMagnitude;
            this.maximumAdjacentViolationMagnitude =
                    builder.maximumAdjacentViolationMagnitude;

            this.pairwisePairCount =
                    builder.pairwisePairCount;
            this.pairwiseInversionCount =
                    builder.pairwiseInversionCount;
            this.pairwiseInversionMagnitude =
                    builder.pairwiseInversionMagnitude;
            this.maximumPairwiseInversionMagnitude =
                    builder.maximumPairwiseInversionMagnitude;

            this.sumRouteAdjacentViolationRates =
                    builder.sumRouteAdjacentViolationRates;
            this.routesWithAdjacentPairs =
                    builder.routesWithAdjacentPairs;

            this.sumRoutePairwiseInversionRates =
                    builder.sumRoutePairwiseInversionRates;
            this.routesWithPairwisePairs =
                    builder.routesWithPairwisePairs;

            this.sharedTaskCount =
                    builder.sharedTaskCount;

            this.averageRouteLength =
                    safeRate(taskCount, routeCount);

            this.maximumRouteLength =
                    builder.maximumRouteLength;
        }

        private InstanceMetrics(
                InstanceMetrics source,
                double heuristicGapPercent
        ) {
            this.instanceId = source.instanceId;
            this.heuristicGapPercent = heuristicGapPercent;
            this.routeCount = source.routeCount;
            this.taskCount = source.taskCount;
            this.nonIncreasingRouteCount =
                    source.nonIncreasingRouteCount;
            this.adjacentPairCount =
                    source.adjacentPairCount;
            this.adjacentViolationCount =
                    source.adjacentViolationCount;
            this.adjacentViolationMagnitude =
                    source.adjacentViolationMagnitude;
            this.maximumAdjacentViolationMagnitude =
                    source.maximumAdjacentViolationMagnitude;
            this.pairwisePairCount =
                    source.pairwisePairCount;
            this.pairwiseInversionCount =
                    source.pairwiseInversionCount;
            this.pairwiseInversionMagnitude =
                    source.pairwiseInversionMagnitude;
            this.maximumPairwiseInversionMagnitude =
                    source.maximumPairwiseInversionMagnitude;
            this.sumRouteAdjacentViolationRates =
                    source.sumRouteAdjacentViolationRates;
            this.routesWithAdjacentPairs =
                    source.routesWithAdjacentPairs;
            this.sumRoutePairwiseInversionRates =
                    source.sumRoutePairwiseInversionRates;
            this.routesWithPairwisePairs =
                    source.routesWithPairwisePairs;
            this.sharedTaskCount =
                    source.sharedTaskCount;
            this.averageRouteLength =
                    source.averageRouteLength;
            this.maximumRouteLength =
                    source.maximumRouteLength;
        }

        private InstanceMetrics withHeuristicGapPercent(
                double gapPercent
        ) {
            if (!Double.isFinite(gapPercent)) {
                throw new IllegalArgumentException(
                        "Heuristic gap must be finite."
                );
            }

            return new InstanceMetrics(this, gapPercent);
        }

        public String getInstanceId() {
            return instanceId;
        }

        public boolean hasHeuristicGap() {
            return heuristicGapPercent != null;
        }

        public double getHeuristicGapPercent() {
            return heuristicGapPercent == null
                    ? Double.NaN
                    : heuristicGapPercent;
        }

        public int getRouteCount() {
            return routeCount;
        }

        public long getTaskCount() {
            return taskCount;
        }

        public int getNonIncreasingRouteCount() {
            return nonIncreasingRouteCount;
        }

        public int getSharedTaskCount() {
            return sharedTaskCount;
        }

        public double getAverageRouteLength() {
            return averageRouteLength;
        }

        public int getMaximumRouteLength() {
            return maximumRouteLength;
        }

        public double getNonIncreasingRouteRate() {
            return safeRate(
                    nonIncreasingRouteCount,
                    routeCount
            );
        }

        public double getRouteViolationRate() {
            return 1.0 - getNonIncreasingRouteRate();
        }

        public double getAdjacentViolationRate() {
            return safeRate(
                    adjacentViolationCount,
                    adjacentPairCount
            );
        }

        public double getMeanRouteAdjacentViolationRate() {
            return safeRate(
                    sumRouteAdjacentViolationRates,
                    routesWithAdjacentPairs
            );
        }

        public double getAverageAdjacentViolationMagnitude() {
            return safeRate(
                    adjacentViolationMagnitude,
                    adjacentViolationCount
            );
        }

        public double getPairwiseInversionRate() {
            return safeRate(
                    pairwiseInversionCount,
                    pairwisePairCount
            );
        }

        public double getMeanRoutePairwiseInversionRate() {
            return safeRate(
                    sumRoutePairwiseInversionRates,
                    routesWithPairwisePairs
            );
        }

        public double getAveragePairwiseInversionMagnitude() {
            return safeRate(
                    pairwiseInversionMagnitude,
                    pairwiseInversionCount
            );
        }
    }
}