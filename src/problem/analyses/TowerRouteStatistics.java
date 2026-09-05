package problem.analyses;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Collects descriptive statistics for tower routes.
 *
 * A tower route is supplied as the ordered list of visited zone identifiers;
 * depot identifiers must already have been removed.
 *
 * Assumption: larger numerical values represent higher zone priorities.
 */
public final class TowerRouteStatistics {

    private static final Map<String, InstanceMetrics> INSTANCES =
            new LinkedHashMap<>();

    private TowerRouteStatistics() {
    }

    public enum Segment {
        BEGINNING("Beginning (0%-33%)"),
        MIDDLE("Middle (33%-67%)"),
        END("End (67%-100%)"),
        SINGLE("Single-zone routes");

        private final String label;

        Segment(String label) {
            this.label = label;
        }

        public String getLabel() {
            return label;
        }
    }

    public static final class InstanceBuilder {

        private final String instanceId;

        private final DistributionAccumulator[] weightBySegment =
                createAccumulators();

        private final DistributionAccumulator[] priorityBySegment =
                createAccumulators();

        private long routeCount;
        private long zoneVisitCount;
        private int maximumRouteLength;

        private long transitionCount;
        private long upwardTransitionCount;
        private long downwardTransitionCount;
        private long equalPriorityTransitionCount;

        private long upwardTransitionMagnitudeSum;
        private int maximumUpwardTransitionMagnitude;

        private long downwardTransitionMagnitudeSum;
        private int maximumDownwardTransitionMagnitude;

        private long routesWithTransitions;
        private long routesWithUpwardTransition;
        private double sumRouteUpwardTransitionRates;

        private long pairwisePairCount;
        private long pairwiseInversionCount;
        private long pairwiseInversionMagnitudeSum;
        private int maximumPairwiseInversionMagnitude;
        private long routesWithPairwisePairs;
        private double sumRoutePairwiseInversionRates;

        private long routesEndingAtHigherPriority;
        private long routesEndingAtLowerPriority;
        private long routesEndingAtEqualPriority;
        private long netPriorityChangeSum;

        private long upwardWeightTransitionCount;
        private long downwardWeightTransitionCount;
        private long equalWeightTransitionCount;

        private long firstZoneHasMaximumPriorityCount;
        private final long[] firstMaximumPrioritySegmentCount =
                new long[Segment.values().length];

        private long routesWithRepeatedZone;
        private long repeatedZoneVisitCount;

        public InstanceBuilder(String instanceId) {
            this.instanceId = Objects.requireNonNull(
                    instanceId,
                    "instanceId"
            );
        }

        public void addRoute(
                List<Integer> visitedZones,
                ZoneAttributeProvider attributes
        ) {
            Objects.requireNonNull(visitedZones, "visitedZones");
            Objects.requireNonNull(attributes, "attributes");

            if (visitedZones.isEmpty()) {
                return;
            }

            int zoneCount = visitedZones.size();
            int[] priorities = new int[zoneCount];
            double[] weights = new double[zoneCount];

            routeCount++;
            zoneVisitCount += zoneCount;
            maximumRouteLength = Math.max(maximumRouteLength, zoneCount);

            Set<Integer> seenZones = new HashSet<>();
            boolean routeHasRepeatedZone = false;

            int maximumPriority = Integer.MIN_VALUE;
            int firstMaximumPriorityIndex = -1;

            for (int index = 0; index < zoneCount; index++) {
                int zoneId = visitedZones.get(index);
                double weight = attributes.weightOf(zoneId);
                int priority = attributes.priorityOf(zoneId);

                if (!Double.isFinite(weight)) {
                    throw new IllegalArgumentException(
                            "Non-finite weight for zone " + zoneId
                    );
                }

                weights[index] = weight;
                priorities[index] = priority;

                Segment segment = classifySegment(index, zoneCount);
                weightBySegment[segment.ordinal()].add(weight);
                priorityBySegment[segment.ordinal()].add(priority);

                if (!seenZones.add(zoneId)) {
                    repeatedZoneVisitCount++;
                    routeHasRepeatedZone = true;
                }

                if (priority > maximumPriority) {
                    maximumPriority = priority;
                    firstMaximumPriorityIndex = index;
                }
            }

            if (routeHasRepeatedZone) {
                routesWithRepeatedZone++;
            }

            if (priorities[0] == maximumPriority) {
                firstZoneHasMaximumPriorityCount++;
            }

            Segment firstMaximumSegment = classifySegment(
                    firstMaximumPriorityIndex,
                    zoneCount
            );
            firstMaximumPrioritySegmentCount[
                    firstMaximumSegment.ordinal()
                    ]++;

            int firstPriority = priorities[0];
            int lastPriority = priorities[zoneCount - 1];
            int netPriorityChange = lastPriority - firstPriority;
            netPriorityChangeSum += netPriorityChange;

            if (netPriorityChange > 0) {
                routesEndingAtHigherPriority++;
            } else if (netPriorityChange < 0) {
                routesEndingAtLowerPriority++;
            } else {
                routesEndingAtEqualPriority++;
            }

            if (zoneCount > 1) {
                routesWithTransitions++;

                long routeUpwardTransitions = 0;

                for (int index = 0; index < zoneCount - 1; index++) {
                    int currentPriority = priorities[index];
                    int nextPriority = priorities[index + 1];
                    double currentWeight = weights[index];
                    double nextWeight = weights[index + 1];

                    transitionCount++;

                    if (nextPriority > currentPriority) {
                        int magnitude = nextPriority - currentPriority;
                        upwardTransitionCount++;
                        routeUpwardTransitions++;
                        upwardTransitionMagnitudeSum += magnitude;
                        maximumUpwardTransitionMagnitude = Math.max(
                                maximumUpwardTransitionMagnitude,
                                magnitude
                        );
                    } else if (nextPriority < currentPriority) {
                        int magnitude = currentPriority - nextPriority;
                        downwardTransitionCount++;
                        downwardTransitionMagnitudeSum += magnitude;
                        maximumDownwardTransitionMagnitude = Math.max(
                                maximumDownwardTransitionMagnitude,
                                magnitude
                        );
                    } else {
                        equalPriorityTransitionCount++;
                    }

                    int weightComparison = Double.compare(
                            nextWeight,
                            currentWeight
                    );

                    if (weightComparison > 0) {
                        upwardWeightTransitionCount++;
                    } else if (weightComparison < 0) {
                        downwardWeightTransitionCount++;
                    } else {
                        equalWeightTransitionCount++;
                    }
                }

                if (routeUpwardTransitions > 0) {
                    routesWithUpwardTransition++;
                }

                sumRouteUpwardTransitionRates +=
                        (double) routeUpwardTransitions / (zoneCount - 1);
            }

            if (zoneCount > 1) {
                routesWithPairwisePairs++;
                long routePairwisePairs = 0;
                long routePairwiseInversions = 0;

                for (int earlier = 0; earlier < zoneCount - 1; earlier++) {
                    for (int later = earlier + 1;
                         later < zoneCount;
                         later++) {

                        routePairwisePairs++;
                        pairwisePairCount++;

                        if (priorities[earlier] < priorities[later]) {
                            int magnitude =
                                    priorities[later] - priorities[earlier];

                            routePairwiseInversions++;
                            pairwiseInversionCount++;
                            pairwiseInversionMagnitudeSum += magnitude;
                            maximumPairwiseInversionMagnitude = Math.max(
                                    maximumPairwiseInversionMagnitude,
                                    magnitude
                            );
                        }
                    }
                }

                sumRoutePairwiseInversionRates +=
                        (double) routePairwiseInversions
                                / routePairwisePairs;
            }
        }

        public InstanceMetrics build() {
            return new InstanceMetrics(this);
        }
    }

    public static synchronized void recordInstance(
            InstanceMetrics metrics
    ) {
        Objects.requireNonNull(metrics, "metrics");
        INSTANCES.put(metrics.instanceId, metrics);
    }

    public static synchronized void reset() {
        INSTANCES.clear();
    }

    public static synchronized List<InstanceMetrics> getInstanceMetrics() {
        return new ArrayList<>(INSTANCES.values());
    }

    public static void printSummary() {
        printSummary(System.out);
    }

    public static void printSummary(PrintStream output) {
        Objects.requireNonNull(output, "output");

        List<InstanceMetrics> instances = getInstanceMetrics();

        output.println();
        output.println("Tower-Route Statistics:");

        if (instances.isEmpty()) {
            output.println("\tNo tower routes have been recorded.");
            return;
        }

        Aggregate aggregate = aggregate(instances);

        output.printf(Locale.US, "\tInstances: %d%n", instances.size());
        output.printf(Locale.US, "\tTower routes: %d%n", aggregate.routeCount);
        output.printf(Locale.US, "\tZone visits: %d%n", aggregate.zoneVisitCount);
        output.printf(
                Locale.US,
                "\tAverage visited zones per route: %.4f%n",
                safeRate(aggregate.zoneVisitCount, aggregate.routeCount)
        );
        output.printf(
                Locale.US,
                "\tMaximum visited zones in a route: %d%n",
                aggregate.maximumRouteLength
        );

        output.printf(
                Locale.US,
                "\tRoutes revisiting at least one zone: %d/%d (%.2f%%)%n",
                aggregate.routesWithRepeatedZone,
                aggregate.routeCount,
                100.0 * safeRate(
                        aggregate.routesWithRepeatedZone,
                        aggregate.routeCount
                )
        );

        output.printf(
                Locale.US,
                "\tRepeated zone visits: %d/%d (%.2f%% of visits)%n",
                aggregate.repeatedZoneVisitCount,
                aggregate.zoneVisitCount,
                100.0 * safeRate(
                        aggregate.repeatedZoneVisitCount,
                        aggregate.zoneVisitCount
                )
        );

        printSegmentDistributions(output, aggregate);
        printPriorityRepositioning(output, aggregate);
        printWeightMovement(output, aggregate);
        printMaximumPriorityPlacement(output, aggregate);
    }

    private static void printSegmentDistributions(
            PrintStream output,
            Aggregate aggregate
    ) {
        output.println();
        output.println(
                "\tZone attributes by normalized tower-route segment:"
        );
        output.println(
                "\t\tBeginning: [0%,33%), Middle: [33%,67%], "
                        + "End: (67%,100%]."
        );

        output.println("\t\tZone weight:");
        for (Segment segment : Segment.values()) {
            printDistribution(
                    output,
                    segment.getLabel(),
                    aggregate.weightBySegment[segment.ordinal()]
            );
        }

        output.println("\t\tZone priority:");
        for (Segment segment : Segment.values()) {
            printDistribution(
                    output,
                    segment.getLabel(),
                    aggregate.priorityBySegment[segment.ordinal()]
            );
        }
    }

    private static void printPriorityRepositioning(
            PrintStream output,
            Aggregate aggregate
    ) {
        output.println();
        output.println("\tPriority changes between consecutive zones:");

        output.printf(
                Locale.US,
                "\t\tLower-to-higher repositioning: %d/%d (%.4f%%)%n",
                aggregate.upwardTransitionCount,
                aggregate.transitionCount,
                100.0 * safeRate(
                        aggregate.upwardTransitionCount,
                        aggregate.transitionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tHigher-to-lower repositioning: %d/%d (%.4f%%)%n",
                aggregate.downwardTransitionCount,
                aggregate.transitionCount,
                100.0 * safeRate(
                        aggregate.downwardTransitionCount,
                        aggregate.transitionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tEqual-priority repositioning: %d/%d (%.4f%%)%n",
                aggregate.equalPriorityTransitionCount,
                aggregate.transitionCount,
                100.0 * safeRate(
                        aggregate.equalPriorityTransitionCount,
                        aggregate.transitionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tRoutes with at least one lower-to-higher move: "
                        + "%d/%d (%.2f%%)%n",
                aggregate.routesWithUpwardTransition,
                aggregate.routeCount,
                100.0 * safeRate(
                        aggregate.routesWithUpwardTransition,
                        aggregate.routeCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tMean route-level lower-to-higher rate: %.4f%%%n",
                100.0 * safeRate(
                        aggregate.sumRouteUpwardTransitionRates,
                        aggregate.routesWithTransitions
                )
        );

        output.printf(
                Locale.US,
                "\t\tAverage upward priority jump: %.4f%n",
                safeRate(
                        aggregate.upwardTransitionMagnitudeSum,
                        aggregate.upwardTransitionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tMaximum upward priority jump: %d%n",
                aggregate.maximumUpwardTransitionMagnitude
        );

        output.printf(
                Locale.US,
                "\t\tAverage downward priority drop: %.4f%n",
                safeRate(
                        aggregate.downwardTransitionMagnitudeSum,
                        aggregate.downwardTransitionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tOverall pairwise inversion rate: %d/%d (%.4f%%)%n",
                aggregate.pairwiseInversionCount,
                aggregate.pairwisePairCount,
                100.0 * safeRate(
                        aggregate.pairwiseInversionCount,
                        aggregate.pairwisePairCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tMean route-level pairwise inversion rate: %.4f%%%n",
                100.0 * safeRate(
                        aggregate.sumRoutePairwiseInversionRates,
                        aggregate.routesWithPairwisePairs
                )
        );

        output.printf(
                Locale.US,
                "\t\tAverage pairwise inversion magnitude: %.4f%n",
                safeRate(
                        aggregate.pairwiseInversionMagnitudeSum,
                        aggregate.pairwiseInversionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tMaximum pairwise inversion magnitude: %d%n",
                aggregate.maximumPairwiseInversionMagnitude
        );

        output.println();
        output.println("\tPriority change from first to last visited zone:");

        output.printf(
                Locale.US,
                "\t\tEnds at higher priority: %d/%d (%.2f%%)%n",
                aggregate.routesEndingAtHigherPriority,
                aggregate.routeCount,
                100.0 * safeRate(
                        aggregate.routesEndingAtHigherPriority,
                        aggregate.routeCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tEnds at lower priority: %d/%d (%.2f%%)%n",
                aggregate.routesEndingAtLowerPriority,
                aggregate.routeCount,
                100.0 * safeRate(
                        aggregate.routesEndingAtLowerPriority,
                        aggregate.routeCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tEnds at equal priority: %d/%d (%.2f%%)%n",
                aggregate.routesEndingAtEqualPriority,
                aggregate.routeCount,
                100.0 * safeRate(
                        aggregate.routesEndingAtEqualPriority,
                        aggregate.routeCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tMean last-minus-first priority: %.4f%n",
                safeRate(
                        aggregate.netPriorityChangeSum,
                        aggregate.routeCount
                )
        );
    }

    private static void printWeightMovement(
            PrintStream output,
            Aggregate aggregate
    ) {
        output.println();
        output.println("\tWeight changes between consecutive zones:");

        output.printf(
                Locale.US,
                "\t\tMoves to a higher-weight zone: %d/%d (%.4f%%)%n",
                aggregate.upwardWeightTransitionCount,
                aggregate.transitionCount,
                100.0 * safeRate(
                        aggregate.upwardWeightTransitionCount,
                        aggregate.transitionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tMoves to a lower-weight zone: %d/%d (%.4f%%)%n",
                aggregate.downwardWeightTransitionCount,
                aggregate.transitionCount,
                100.0 * safeRate(
                        aggregate.downwardWeightTransitionCount,
                        aggregate.transitionCount
                )
        );

        output.printf(
                Locale.US,
                "\t\tMoves to an equal-weight zone: %d/%d (%.4f%%)%n",
                aggregate.equalWeightTransitionCount,
                aggregate.transitionCount,
                100.0 * safeRate(
                        aggregate.equalWeightTransitionCount,
                        aggregate.transitionCount
                )
        );
    }

    private static void printMaximumPriorityPlacement(
            PrintStream output,
            Aggregate aggregate
    ) {
        output.println();
        output.println("\tPlacement of each route's maximum-priority zone:");

        output.printf(
                Locale.US,
                "\t\tFirst visited zone has maximum route priority: "
                        + "%d/%d (%.2f%%)%n",
                aggregate.firstZoneHasMaximumPriorityCount,
                aggregate.routeCount,
                100.0 * safeRate(
                        aggregate.firstZoneHasMaximumPriorityCount,
                        aggregate.routeCount
                )
        );

        for (Segment segment : Segment.values()) {
            long count = aggregate.firstMaximumPrioritySegmentCount[
                    segment.ordinal()
                    ];

            output.printf(
                    Locale.US,
                    "\t\tFirst occurrence in %s: %d/%d (%.2f%%)%n",
                    segment.getLabel(),
                    count,
                    aggregate.routeCount,
                    100.0 * safeRate(count, aggregate.routeCount)
            );
        }
    }

    private static void printDistribution(
            PrintStream output,
            String label,
            DistributionAccumulator accumulator
    ) {
        DistributionSummary summary = accumulator.summarize();

        if (summary.count == 0) {
            return;
        }

        output.printf(
                Locale.US,
                "\t\t\t%s: n=%d, mean=%.4f, stdDev=%.4f, "
                        + "min=%.4f, Q1=%.4f, median=%.4f, "
                        + "Q3=%.4f, max=%.4f%n",
                label,
                summary.count,
                summary.mean,
                summary.standardDeviation,
                summary.minimum,
                summary.firstQuartile,
                summary.median,
                summary.thirdQuartile,
                summary.maximum
        );
    }

    public static void writeInstanceCsv(Path outputFile)
            throws IOException {

        Objects.requireNonNull(outputFile, "outputFile");
        Path parent = outputFile.getParent();

        if (parent != null) {
            Files.createDirectories(parent);
        }

        List<InstanceMetrics> instances = getInstanceMetrics();
        instances.sort(
                Comparator.comparing(metrics -> metrics.instanceId)
        );

        try (
                BufferedWriter writer = Files.newBufferedWriter(outputFile);
                PrintWriter output = new PrintWriter(writer)
        ) {
            output.println(
                    "instance_id,route_count,zone_visit_count,"
                            + "average_route_length,maximum_route_length,"
                            + "upward_transition_count,transition_count,"
                            + "upward_transition_rate_percent,"
                            + "mean_route_upward_transition_rate_percent,"
                            + "routes_with_upward_transition_percent,"
                            + "average_upward_priority_jump,"
                            + "maximum_upward_priority_jump,"
                            + "pairwise_inversion_rate_percent,"
                            + "mean_route_pairwise_inversion_rate_percent,"
                            + "average_pairwise_inversion_magnitude,"
                            + "routes_ending_at_higher_priority_percent,"
                            + "mean_last_minus_first_priority,"
                            + "first_zone_has_maximum_priority_percent,"
                            + "routes_with_repeated_zone_percent"
            );

            for (InstanceMetrics metrics : instances) {
                output.printf(
                        Locale.US,
                        "%s,%d,%d,%.8f,%d,%d,%d,%.8f,%.8f,%.8f,"
                                + "%.8f,%d,%.8f,%.8f,%.8f,%.8f,"
                                + "%.8f,%.8f,%.8f%n",
                        escapeCsv(metrics.instanceId),
                        metrics.routeCount,
                        metrics.zoneVisitCount,
                        safeRate(metrics.zoneVisitCount, metrics.routeCount),
                        metrics.maximumRouteLength,
                        metrics.upwardTransitionCount,
                        metrics.transitionCount,
                        100.0 * safeRate(
                                metrics.upwardTransitionCount,
                                metrics.transitionCount
                        ),
                        100.0 * safeRate(
                                metrics.sumRouteUpwardTransitionRates,
                                metrics.routesWithTransitions
                        ),
                        100.0 * safeRate(
                                metrics.routesWithUpwardTransition,
                                metrics.routeCount
                        ),
                        safeRate(
                                metrics.upwardTransitionMagnitudeSum,
                                metrics.upwardTransitionCount
                        ),
                        metrics.maximumUpwardTransitionMagnitude,
                        100.0 * safeRate(
                                metrics.pairwiseInversionCount,
                                metrics.pairwisePairCount
                        ),
                        100.0 * safeRate(
                                metrics.sumRoutePairwiseInversionRates,
                                metrics.routesWithPairwisePairs
                        ),
                        safeRate(
                                metrics.pairwiseInversionMagnitudeSum,
                                metrics.pairwiseInversionCount
                        ),
                        100.0 * safeRate(
                                metrics.routesEndingAtHigherPriority,
                                metrics.routeCount
                        ),
                        safeRate(
                                metrics.netPriorityChangeSum,
                                metrics.routeCount
                        ),
                        100.0 * safeRate(
                                metrics.firstZoneHasMaximumPriorityCount,
                                metrics.routeCount
                        ),
                        100.0 * safeRate(
                                metrics.routesWithRepeatedZone,
                                metrics.routeCount
                        )
                );
            }
        }
    }

    private static Aggregate aggregate(
            List<InstanceMetrics> instances
    ) {
        Aggregate aggregate = new Aggregate();

        for (InstanceMetrics instance : instances) {
            aggregate.routeCount += instance.routeCount;
            aggregate.zoneVisitCount += instance.zoneVisitCount;
            aggregate.maximumRouteLength = Math.max(
                    aggregate.maximumRouteLength,
                    instance.maximumRouteLength
            );

            aggregate.transitionCount += instance.transitionCount;
            aggregate.upwardTransitionCount +=
                    instance.upwardTransitionCount;
            aggregate.downwardTransitionCount +=
                    instance.downwardTransitionCount;
            aggregate.equalPriorityTransitionCount +=
                    instance.equalPriorityTransitionCount;
            aggregate.upwardTransitionMagnitudeSum +=
                    instance.upwardTransitionMagnitudeSum;
            aggregate.maximumUpwardTransitionMagnitude = Math.max(
                    aggregate.maximumUpwardTransitionMagnitude,
                    instance.maximumUpwardTransitionMagnitude
            );
            aggregate.downwardTransitionMagnitudeSum +=
                    instance.downwardTransitionMagnitudeSum;
            aggregate.maximumDownwardTransitionMagnitude = Math.max(
                    aggregate.maximumDownwardTransitionMagnitude,
                    instance.maximumDownwardTransitionMagnitude
            );
            aggregate.routesWithTransitions +=
                    instance.routesWithTransitions;
            aggregate.routesWithUpwardTransition +=
                    instance.routesWithUpwardTransition;
            aggregate.sumRouteUpwardTransitionRates +=
                    instance.sumRouteUpwardTransitionRates;

            aggregate.pairwisePairCount += instance.pairwisePairCount;
            aggregate.pairwiseInversionCount +=
                    instance.pairwiseInversionCount;
            aggregate.pairwiseInversionMagnitudeSum +=
                    instance.pairwiseInversionMagnitudeSum;
            aggregate.maximumPairwiseInversionMagnitude = Math.max(
                    aggregate.maximumPairwiseInversionMagnitude,
                    instance.maximumPairwiseInversionMagnitude
            );
            aggregate.routesWithPairwisePairs +=
                    instance.routesWithPairwisePairs;
            aggregate.sumRoutePairwiseInversionRates +=
                    instance.sumRoutePairwiseInversionRates;

            aggregate.routesEndingAtHigherPriority +=
                    instance.routesEndingAtHigherPriority;
            aggregate.routesEndingAtLowerPriority +=
                    instance.routesEndingAtLowerPriority;
            aggregate.routesEndingAtEqualPriority +=
                    instance.routesEndingAtEqualPriority;
            aggregate.netPriorityChangeSum +=
                    instance.netPriorityChangeSum;

            aggregate.upwardWeightTransitionCount +=
                    instance.upwardWeightTransitionCount;
            aggregate.downwardWeightTransitionCount +=
                    instance.downwardWeightTransitionCount;
            aggregate.equalWeightTransitionCount +=
                    instance.equalWeightTransitionCount;

            aggregate.firstZoneHasMaximumPriorityCount +=
                    instance.firstZoneHasMaximumPriorityCount;

            for (Segment segment : Segment.values()) {
                int index = segment.ordinal();
                aggregate.weightBySegment[index].addAll(
                        instance.weightValuesBySegment[index]
                );
                aggregate.priorityBySegment[index].addAll(
                        instance.priorityValuesBySegment[index]
                );
                aggregate.firstMaximumPrioritySegmentCount[index] +=
                        instance.firstMaximumPrioritySegmentCount[index];
            }

            aggregate.routesWithRepeatedZone +=
                    instance.routesWithRepeatedZone;
            aggregate.repeatedZoneVisitCount +=
                    instance.repeatedZoneVisitCount;
        }

        return aggregate;
    }

    private static Segment classifySegment(
            int zeroBasedPosition,
            int routeLength
    ) {
        if (routeLength == 1) {
            return Segment.SINGLE;
        }

        double normalizedPosition =
                (double) zeroBasedPosition / (routeLength - 1);

        if (normalizedPosition < 1.0 / 3.0) {
            return Segment.BEGINNING;
        }

        if (normalizedPosition <= 2.0 / 3.0) {
            return Segment.MIDDLE;
        }

        return Segment.END;
    }

    private static DistributionAccumulator[] createAccumulators() {
        DistributionAccumulator[] result =
                new DistributionAccumulator[Segment.values().length];

        for (int index = 0; index < result.length; index++) {
            result[index] = new DistributionAccumulator();
        }

        return result;
    }

    private static double safeRate(long numerator, long denominator) {
        return denominator == 0
                ? 0.0
                : (double) numerator / denominator;
    }

    private static double safeRate(double numerator, long denominator) {
        return denominator == 0
                ? 0.0
                : numerator / denominator;
    }

    private static String escapeCsv(String value) {
        if (value.contains(",")
                || value.contains("\"")
                || value.contains("\n")) {

            return "\"" + value.replace("\"", "\"\"") + "\"";
        }

        return value;
    }

    public static final class InstanceMetrics {

        private final String instanceId;
        private final List<Double>[] weightValuesBySegment;
        private final List<Double>[] priorityValuesBySegment;

        private final long routeCount;
        private final long zoneVisitCount;
        private final int maximumRouteLength;

        private final long transitionCount;
        private final long upwardTransitionCount;
        private final long downwardTransitionCount;
        private final long equalPriorityTransitionCount;
        private final long upwardTransitionMagnitudeSum;
        private final int maximumUpwardTransitionMagnitude;
        private final long downwardTransitionMagnitudeSum;
        private final int maximumDownwardTransitionMagnitude;
        private final long routesWithTransitions;
        private final long routesWithUpwardTransition;
        private final double sumRouteUpwardTransitionRates;

        private final long pairwisePairCount;
        private final long pairwiseInversionCount;
        private final long pairwiseInversionMagnitudeSum;
        private final int maximumPairwiseInversionMagnitude;
        private final long routesWithPairwisePairs;
        private final double sumRoutePairwiseInversionRates;

        private final long routesEndingAtHigherPriority;
        private final long routesEndingAtLowerPriority;
        private final long routesEndingAtEqualPriority;
        private final long netPriorityChangeSum;

        private final long upwardWeightTransitionCount;
        private final long downwardWeightTransitionCount;
        private final long equalWeightTransitionCount;

        private final long firstZoneHasMaximumPriorityCount;
        private final long[] firstMaximumPrioritySegmentCount;

        private final long routesWithRepeatedZone;
        private final long repeatedZoneVisitCount;

        @SuppressWarnings("unchecked")
        private InstanceMetrics(InstanceBuilder builder) {
            this.instanceId = builder.instanceId;

            weightValuesBySegment = new List[Segment.values().length];
            priorityValuesBySegment = new List[Segment.values().length];

            for (Segment segment : Segment.values()) {
                int index = segment.ordinal();
                weightValuesBySegment[index] =
                        builder.weightBySegment[index].copyValues();
                priorityValuesBySegment[index] =
                        builder.priorityBySegment[index].copyValues();
            }

            routeCount = builder.routeCount;
            zoneVisitCount = builder.zoneVisitCount;
            maximumRouteLength = builder.maximumRouteLength;
            transitionCount = builder.transitionCount;
            upwardTransitionCount = builder.upwardTransitionCount;
            downwardTransitionCount = builder.downwardTransitionCount;
            equalPriorityTransitionCount =
                    builder.equalPriorityTransitionCount;
            upwardTransitionMagnitudeSum =
                    builder.upwardTransitionMagnitudeSum;
            maximumUpwardTransitionMagnitude =
                    builder.maximumUpwardTransitionMagnitude;
            downwardTransitionMagnitudeSum =
                    builder.downwardTransitionMagnitudeSum;
            maximumDownwardTransitionMagnitude =
                    builder.maximumDownwardTransitionMagnitude;
            routesWithTransitions = builder.routesWithTransitions;
            routesWithUpwardTransition =
                    builder.routesWithUpwardTransition;
            sumRouteUpwardTransitionRates =
                    builder.sumRouteUpwardTransitionRates;
            pairwisePairCount = builder.pairwisePairCount;
            pairwiseInversionCount = builder.pairwiseInversionCount;
            pairwiseInversionMagnitudeSum =
                    builder.pairwiseInversionMagnitudeSum;
            maximumPairwiseInversionMagnitude =
                    builder.maximumPairwiseInversionMagnitude;
            routesWithPairwisePairs = builder.routesWithPairwisePairs;
            sumRoutePairwiseInversionRates =
                    builder.sumRoutePairwiseInversionRates;
            routesEndingAtHigherPriority =
                    builder.routesEndingAtHigherPriority;
            routesEndingAtLowerPriority =
                    builder.routesEndingAtLowerPriority;
            routesEndingAtEqualPriority =
                    builder.routesEndingAtEqualPriority;
            netPriorityChangeSum = builder.netPriorityChangeSum;
            upwardWeightTransitionCount =
                    builder.upwardWeightTransitionCount;
            downwardWeightTransitionCount =
                    builder.downwardWeightTransitionCount;
            equalWeightTransitionCount =
                    builder.equalWeightTransitionCount;
            firstZoneHasMaximumPriorityCount =
                    builder.firstZoneHasMaximumPriorityCount;
            firstMaximumPrioritySegmentCount =
                    builder.firstMaximumPrioritySegmentCount.clone();
            routesWithRepeatedZone = builder.routesWithRepeatedZone;
            repeatedZoneVisitCount = builder.repeatedZoneVisitCount;
        }
    }

    private static final class Aggregate {
        private final DistributionAccumulator[] weightBySegment =
                createAccumulators();
        private final DistributionAccumulator[] priorityBySegment =
                createAccumulators();

        private long routeCount;
        private long zoneVisitCount;
        private int maximumRouteLength;
        private long transitionCount;
        private long upwardTransitionCount;
        private long downwardTransitionCount;
        private long equalPriorityTransitionCount;
        private long upwardTransitionMagnitudeSum;
        private int maximumUpwardTransitionMagnitude;
        private long downwardTransitionMagnitudeSum;
        private int maximumDownwardTransitionMagnitude;
        private long routesWithTransitions;
        private long routesWithUpwardTransition;
        private double sumRouteUpwardTransitionRates;
        private long pairwisePairCount;
        private long pairwiseInversionCount;
        private long pairwiseInversionMagnitudeSum;
        private int maximumPairwiseInversionMagnitude;
        private long routesWithPairwisePairs;
        private double sumRoutePairwiseInversionRates;
        private long routesEndingAtHigherPriority;
        private long routesEndingAtLowerPriority;
        private long routesEndingAtEqualPriority;
        private long netPriorityChangeSum;
        private long upwardWeightTransitionCount;
        private long downwardWeightTransitionCount;
        private long equalWeightTransitionCount;
        private long firstZoneHasMaximumPriorityCount;
        private final long[] firstMaximumPrioritySegmentCount =
                new long[Segment.values().length];
        private long routesWithRepeatedZone;
        private long repeatedZoneVisitCount;
    }

    private static final class DistributionAccumulator {
        private final List<Double> values = new ArrayList<>();

        private void add(double value) {
            values.add(value);
        }

        private void addAll(List<Double> source) {
            values.addAll(source);
        }

        private List<Double> copyValues() {
            return new ArrayList<>(values);
        }

        private DistributionSummary summarize() {
            if (values.isEmpty()) {
                return DistributionSummary.empty();
            }

            double[] sorted = values.stream()
                    .mapToDouble(Double::doubleValue)
                    .sorted()
                    .toArray();

            double mean = Arrays.stream(sorted).average().orElse(0.0);
            double squaredDeviationSum = 0.0;

            for (double value : sorted) {
                double deviation = value - mean;
                squaredDeviationSum += deviation * deviation;
            }

            double standardDeviation = Math.sqrt(
                    squaredDeviationSum / sorted.length
            );

            return new DistributionSummary(
                    sorted.length,
                    mean,
                    standardDeviation,
                    sorted[0],
                    quantile(sorted, 0.25),
                    quantile(sorted, 0.50),
                    quantile(sorted, 0.75),
                    sorted[sorted.length - 1]
            );
        }

        private static double quantile(
                double[] sorted,
                double probability
        ) {
            if (sorted.length == 1) {
                return sorted[0];
            }

            double position = probability * (sorted.length - 1);
            int lower = (int) Math.floor(position);
            int upper = (int) Math.ceil(position);

            if (lower == upper) {
                return sorted[lower];
            }

            double weight = position - lower;
            return sorted[lower]
                    + weight * (sorted[upper] - sorted[lower]);
        }
    }

    private static final class DistributionSummary {
        private final long count;
        private final double mean;
        private final double standardDeviation;
        private final double minimum;
        private final double firstQuartile;
        private final double median;
        private final double thirdQuartile;
        private final double maximum;

        private DistributionSummary(
                long count,
                double mean,
                double standardDeviation,
                double minimum,
                double firstQuartile,
                double median,
                double thirdQuartile,
                double maximum
        ) {
            this.count = count;
            this.mean = mean;
            this.standardDeviation = standardDeviation;
            this.minimum = minimum;
            this.firstQuartile = firstQuartile;
            this.median = median;
            this.thirdQuartile = thirdQuartile;
            this.maximum = maximum;
        }

        private static DistributionSummary empty() {
            return new DistributionSummary(
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
