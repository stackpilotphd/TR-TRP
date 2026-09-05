package problem.analyses;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

public final class PositionalStatistics {

    /*
     * Positions 1 through 9 are supported.
     * Index 0 is unused because route position numbering starts at 1.
     */
    private static final int ABSOLUTE_POSITION_LIMIT = 10;

    /*
     * Five normalized route sections:
     *
     * [0%, 20%)
     * [20%, 40%)
     * [40%, 60%)
     * [60%, 80%)
     * [80%, 100%]
     */
    private static final int NORMALIZED_BIN_COUNT = 5;

    private static final String[] NORMALIZED_BIN_LABELS = {
            "0%-20%",
            "20%-40%",
            "40%-60%",
            "60%-80%",
            "80%-100%"
    };

    private static DistributionAccumulator[] weightByPosition =
            createAccumulators(ABSOLUTE_POSITION_LIMIT);

    private static DistributionAccumulator[] serviceByPosition =
            createAccumulators(ABSOLUTE_POSITION_LIMIT);

    private static DistributionAccumulator[] priorityByPosition =
            createAccumulators(ABSOLUTE_POSITION_LIMIT);

    private static long[] taskCountByPosition =
            new long[ABSOLUTE_POSITION_LIMIT];

    private static long[] sharedTaskCountByPosition =
            new long[ABSOLUTE_POSITION_LIMIT];

    private static DistributionAccumulator[] weightByNormalizedPosition =
            createAccumulators(NORMALIZED_BIN_COUNT);

    private static DistributionAccumulator[] serviceByNormalizedPosition =
            createAccumulators(NORMALIZED_BIN_COUNT);

    private static DistributionAccumulator[] priorityByNormalizedPosition =
            createAccumulators(NORMALIZED_BIN_COUNT);

    private static long[] taskCountByNormalizedPosition =
            new long[NORMALIZED_BIN_COUNT];

    private static long[] sharedTaskCountByNormalizedPosition =
            new long[NORMALIZED_BIN_COUNT];

    /*
     * A task in a single-task route has no meaningful normalized position,
     * because (position - 1) / (routeLength - 1) would divide by zero.
     */
    private static DistributionAccumulator singleTaskRouteWeights =
            new DistributionAccumulator();

    private static DistributionAccumulator singleTaskRouteServices =
            new DistributionAccumulator();

    private static DistributionAccumulator singleTaskRoutePriorities =
            new DistributionAccumulator();

    private static long singleTaskRouteTaskCount;
    private static long singleTaskRouteSharedTaskCount;

    private PositionalStatistics() {
    }

    /**
     * Records one task occurrence.
     *
     * @param absolutePosition task position in the route, beginning at 1
     * @param routeTaskCount   number of tasks in the route, excluding depots
     * @param weight           task weight
     * @param serviceDuration  task service duration
     * @param priority         task priority
     * @param shared           whether the task is shared
     */
    public static synchronized void recordTask(
            int absolutePosition,
            int routeTaskCount,
            double weight,
            double serviceDuration,
            int priority,
            boolean shared
    ) {
        validateInput(
                absolutePosition,
                routeTaskCount,
                weight,
                serviceDuration
        );

        recordAbsolutePosition(
                absolutePosition,
                weight,
                serviceDuration,
                priority,
                shared
        );

        recordNormalizedPosition(
                absolutePosition,
                routeTaskCount,
                weight,
                serviceDuration,
                priority,
                shared
        );
    }

    private static void recordAbsolutePosition(
            int position,
            double weight,
            double serviceDuration,
            int priority,
            boolean shared
    ) {
        weightByPosition[position].add(weight);
        serviceByPosition[position].add(serviceDuration);
        priorityByPosition[position].add(priority);

        taskCountByPosition[position]++;

        if (shared) {
            sharedTaskCountByPosition[position]++;
        }
    }

    private static void recordNormalizedPosition(
            int position,
            int routeTaskCount,
            double weight,
            double serviceDuration,
            int priority,
            boolean shared
    ) {
        if (routeTaskCount == 1) {
            singleTaskRouteWeights.add(weight);
            singleTaskRouteServices.add(serviceDuration);
            singleTaskRoutePriorities.add(priority);

            singleTaskRouteTaskCount++;

            if (shared) {
                singleTaskRouteSharedTaskCount++;
            }

            return;
        }

        /*
         * First task: normalized position 0.0
         * Last task:  normalized position 1.0
         */
        double normalizedPosition =
                (double) (position - 1)
                        / (routeTaskCount - 1);

        int bin = (int) Math.floor(
                normalizedPosition * NORMALIZED_BIN_COUNT
        );

        /*
         * The last task has normalized position 1.0, which would produce
         * bin 5. It belongs in the final bin, whose index is 4.
         */
        bin = Math.min(bin, NORMALIZED_BIN_COUNT - 1);

        weightByNormalizedPosition[bin].add(weight);
        serviceByNormalizedPosition[bin].add(serviceDuration);
        priorityByNormalizedPosition[bin].add(priority);

        taskCountByNormalizedPosition[bin]++;

        if (shared) {
            sharedTaskCountByNormalizedPosition[bin]++;
        }
    }

    private static void validateInput(
            int absolutePosition,
            int routeTaskCount,
            double weight,
            double serviceDuration
    ) {
        if (routeTaskCount < 1) {
            throw new IllegalArgumentException(
                    "Route task count must be at least 1."
            );
        }

        if (absolutePosition < 1
                || absolutePosition > routeTaskCount) {
            throw new IllegalArgumentException(
                    "Position "
                            + absolutePosition
                            + " is invalid for a route containing "
                            + routeTaskCount
                            + " tasks."
            );
        }

        if (absolutePosition >= ABSOLUTE_POSITION_LIMIT) {
            throw new IllegalArgumentException(
                    "Position "
                            + absolutePosition
                            + " exceeds the configured limit of "
                            + (ABSOLUTE_POSITION_LIMIT - 1)
                            + "."
            );
        }

        if (!Double.isFinite(weight)) {
            throw new IllegalArgumentException(
                    "Task weight must be finite."
            );
        }

        if (!Double.isFinite(serviceDuration)) {
            throw new IllegalArgumentException(
                    "Service duration must be finite."
            );
        }
    }

    public static synchronized void printAbsolutePositionStatistics() {
        printAbsolutePositionStatistics(System.out);
    }

    public static synchronized void printAbsolutePositionStatistics(
            PrintStream output
    ) {
        Objects.requireNonNull(output, "output");

        output.println();
        output.println(
                "Detailed Statistics by Absolute Route Position:"
        );

        printAbsoluteMetric(
                output,
                "Task Weight",
                weightByPosition
        );

        printAbsoluteMetric(
                output,
                "Service Duration",
                serviceByPosition
        );

        printAbsoluteMetric(
                output,
                "Task Priority",
                priorityByPosition
        );

        output.println();
        output.println("Shared-task rate by absolute position:");

        for (int position = 1;
             position < ABSOLUTE_POSITION_LIMIT;
             position++) {

            long taskCount = taskCountByPosition[position];

            if (taskCount == 0) {
                continue;
            }

            long sharedCount =
                    sharedTaskCountByPosition[position];

            double sharedRate =
                    (double) sharedCount / taskCount;

            output.printf(
                    Locale.US,
                    "\tPosition %d: %.2f%% [%d/%d]%n",
                    position,
                    100.0 * sharedRate,
                    sharedCount,
                    taskCount
            );
        }
    }

    private static void printAbsoluteMetric(
            PrintStream output,
            String metricName,
            DistributionAccumulator[] accumulators
    ) {
        output.println();
        output.println(metricName + ":");

        for (int position = 1;
             position < ABSOLUTE_POSITION_LIMIT;
             position++) {

            Summary statistics =
                    accumulators[position].summarize();

            if (statistics.count() == 0) {
                continue;
            }

            printSummary(
                    output,
                    "Position " + position,
                    statistics
            );
        }
    }

    public static synchronized void printNormalizedPositionStatistics() {
        printNormalizedPositionStatistics(System.out);
    }

    public static synchronized void printNormalizedPositionStatistics(
            PrintStream output
    ) {
        Objects.requireNonNull(output, "output");

        output.println();
        output.println(
                "Statistics by Normalized Route Position:"
        );

        output.println(
                "\tNormalized position is calculated as "
                        + "(position - 1) / (route task count - 1)."
        );

        printNormalizedMetric(
                output,
                "Task Weight",
                weightByNormalizedPosition,
                singleTaskRouteWeights
        );

        printNormalizedMetric(
                output,
                "Service Duration",
                serviceByNormalizedPosition,
                singleTaskRouteServices
        );

        printNormalizedMetric(
                output,
                "Task Priority",
                priorityByNormalizedPosition,
                singleTaskRoutePriorities
        );

        output.println();
        output.println("Shared-task rate by normalized position:");

        for (int bin = 0;
             bin < NORMALIZED_BIN_COUNT;
             bin++) {

            long taskCount =
                    taskCountByNormalizedPosition[bin];

            if (taskCount == 0) {
                continue;
            }

            long sharedCount =
                    sharedTaskCountByNormalizedPosition[bin];

            double sharedRate =
                    (double) sharedCount / taskCount;

            output.printf(
                    Locale.US,
                    "\t%s: %.2f%% [%d/%d]%n",
                    NORMALIZED_BIN_LABELS[bin],
                    100.0 * sharedRate,
                    sharedCount,
                    taskCount
            );
        }

        if (singleTaskRouteTaskCount > 0) {
            double sharedRate =
                    (double) singleTaskRouteSharedTaskCount
                            / singleTaskRouteTaskCount;

            output.printf(
                    Locale.US,
                    "\tSingle-task routes: %.2f%% [%d/%d]%n",
                    100.0 * sharedRate,
                    singleTaskRouteSharedTaskCount,
                    singleTaskRouteTaskCount
            );
        }
    }

    private static void printNormalizedMetric(
            PrintStream output,
            String metricName,
            DistributionAccumulator[] accumulators,
            DistributionAccumulator singleTaskAccumulator
    ) {
        output.println();
        output.println(metricName + ":");

        for (int bin = 0;
             bin < NORMALIZED_BIN_COUNT;
             bin++) {

            Summary statistics =
                    accumulators[bin].summarize();

            if (statistics.count() == 0) {
                continue;
            }

            printSummary(
                    output,
                    NORMALIZED_BIN_LABELS[bin],
                    statistics
            );
        }

        Summary singleTaskStatistics =
                singleTaskAccumulator.summarize();

        if (singleTaskStatistics.count() > 0) {
            printSummary(
                    output,
                    "Single-task routes",
                    singleTaskStatistics
            );
        }
    }

    private static void printSummary(
            PrintStream output,
            String label,
            Summary statistics
    ) {
        output.printf(
                Locale.US,
                "\t%s: n=%d, mean=%.4f, stdDev=%.4f, "
                        + "min=%.4f, Q1=%.4f, median=%.4f, "
                        + "Q3=%.4f, max=%.4f%n",
                label,
                statistics.count(),
                statistics.mean(),
                statistics.populationStandardDeviation(),
                statistics.minimum(),
                statistics.firstQuartile(),
                statistics.median(),
                statistics.thirdQuartile(),
                statistics.maximum()
        );
    }

    /**
     * Call once before starting a new complete experiment.
     */
    public static synchronized void reset() {
        weightByPosition =
                createAccumulators(ABSOLUTE_POSITION_LIMIT);

        serviceByPosition =
                createAccumulators(ABSOLUTE_POSITION_LIMIT);

        priorityByPosition =
                createAccumulators(ABSOLUTE_POSITION_LIMIT);

        taskCountByPosition =
                new long[ABSOLUTE_POSITION_LIMIT];

        sharedTaskCountByPosition =
                new long[ABSOLUTE_POSITION_LIMIT];

        weightByNormalizedPosition =
                createAccumulators(NORMALIZED_BIN_COUNT);

        serviceByNormalizedPosition =
                createAccumulators(NORMALIZED_BIN_COUNT);

        priorityByNormalizedPosition =
                createAccumulators(NORMALIZED_BIN_COUNT);

        taskCountByNormalizedPosition =
                new long[NORMALIZED_BIN_COUNT];

        sharedTaskCountByNormalizedPosition =
                new long[NORMALIZED_BIN_COUNT];

        singleTaskRouteWeights =
                new DistributionAccumulator();

        singleTaskRouteServices =
                new DistributionAccumulator();

        singleTaskRoutePriorities =
                new DistributionAccumulator();

        singleTaskRouteTaskCount = 0;
        singleTaskRouteSharedTaskCount = 0;
    }

    private static DistributionAccumulator[] createAccumulators(
            int size
    ) {
        DistributionAccumulator[] result =
                new DistributionAccumulator[size];

        for (int index = 0; index < size; index++) {
            result[index] = new DistributionAccumulator();
        }

        return result;
    }

    /**
     * Uses the population standard deviation:
     *
     * sqrt(sum((x - mean)^2) / N)
     *
     * Quartiles use linear interpolation at index:
     *
     * (N - 1) * probability
     */
    private static final class DistributionAccumulator {

        private final List<Double> values = new ArrayList<>();

        private void add(double value) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(
                        "Statistic value must be finite."
                );
            }

            values.add(value);
        }

        private Summary summarize() {
            if (values.isEmpty()) {
                return Summary.empty();
            }

            double[] sortedValues = values.stream()
                    .mapToDouble(Double::doubleValue)
                    .sorted()
                    .toArray();

            double sum = Arrays.stream(sortedValues).sum();
            double mean = sum / sortedValues.length;

            double squaredDeviationSum = 0.0;

            for (double value : sortedValues) {
                double difference = value - mean;
                squaredDeviationSum += difference * difference;
            }

            double populationStandardDeviation =
                    Math.sqrt(
                            squaredDeviationSum
                                    / sortedValues.length
                    );

            return new Summary(
                    sortedValues.length,
                    mean,
                    populationStandardDeviation,
                    sortedValues[0],
                    quantile(sortedValues, 0.25),
                    quantile(sortedValues, 0.50),
                    quantile(sortedValues, 0.75),
                    sortedValues[sortedValues.length - 1]
            );
        }

        private double quantile(
                double[] sortedValues,
                double probability
        ) {
            if (sortedValues.length == 1) {
                return sortedValues[0];
            }

            double index =
                    probability * (sortedValues.length - 1);

            int lowerIndex = (int) Math.floor(index);
            int upperIndex = (int) Math.ceil(index);

            if (lowerIndex == upperIndex) {
                return sortedValues[lowerIndex];
            }

            double interpolationWeight =
                    index - lowerIndex;

            return sortedValues[lowerIndex]
                    + interpolationWeight
                    * (
                    sortedValues[upperIndex]
                            - sortedValues[lowerIndex]
            );
        }
    }

    public record Summary(
            long count,
            double mean,
            double populationStandardDeviation,
            double minimum,
            double firstQuartile,
            double median,
            double thirdQuartile,
            double maximum
    ) {
        private static Summary empty() {
            return new Summary(
                    0,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN
            );
        }
    }
}