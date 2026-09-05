package problem.analyses;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Locale;

public class OutputHandler {

    private static final int POSITION_LIMIT = 10;

    public void writeOutput() {
        printTotalTasks();
        printAverageWeight();
        printAverageServiceDuration();
        printAveragePriority();
        printSharedTaskProbabilityByPosition();
        printSharedTaskPositionDistribution();

        PriorityOrderStatistics.printSummary();

        PositionalStatistics.printAbsolutePositionStatistics();
        PositionalStatistics.printNormalizedPositionStatistics();

        TowerRouteStatistics.printSummary();
    }

    private void printTotalTasks() {
        System.out.println("Total tasks at Position:");

        for (int position = 1;
             position < POSITION_LIMIT;
             position++) {

            int taskCount =
                    StaticCounters
                            .numberOfTasksAtPosition[position];

            if (taskCount > 0) {
                System.out.println(
                        "\t"
                                + position
                                + ":#"
                                + taskCount
                );
            }
        }
    }

    private void printAverageWeight() {
        System.out.println("Average Weight at Position:");

        for (int position = 1;
             position < POSITION_LIMIT;
             position++) {

            int taskCount =
                    StaticCounters
                            .numberOfTasksAtPosition[position];

            if (taskCount > 0) {
                double totalWeight =
                        StaticCounters
                                .totalWeightAtPosition[position];

                double averageWeight =
                        totalWeight / taskCount;

                System.out.println(
                        "\t"
                                + position
                                + ":#"
                                + averageWeight
                );
            }
        }
    }

    private void printAverageServiceDuration() {
        System.out.println(
                "Average Service Duration at Position:"
        );

        for (int position = 1;
             position < POSITION_LIMIT;
             position++) {

            int taskCount =
                    StaticCounters
                            .numberOfTasksAtPosition[position];

            if (taskCount > 0) {
                double totalService =
                        StaticCounters
                                .totalServiceAtPosition[position];

                double averageService =
                        totalService / taskCount;

                System.out.println(
                        "\t"
                                + position
                                + ":#"
                                + averageService
                );
            }
        }
    }

    private void printAveragePriority() {
        System.out.println("Average Priority at Position:");

        for (int position = 1;
             position < POSITION_LIMIT;
             position++) {

            int taskCount =
                    StaticCounters
                            .numberOfTasksAtPosition[position];

            if (taskCount > 0) {
                double totalPriority =
                        StaticCounters
                                .priorityCounterAtPosition[position];

                double averagePriority =
                        totalPriority / taskCount;

                System.out.println(
                        "\t"
                                + position
                                + ":#"
                                + averagePriority
                );
            }
        }
    }

    /**
     * Estimates:
     *
     * P(shared task | task appears at position p,
     *                 instance contains shared tasks)
     */
    private void printSharedTaskProbabilityByPosition() {
        System.out.println(
                "Proportion of Shared Tasks at Position "
                        + "(among instances containing shared tasks):"
        );

        for (int position = 1;
             position < POSITION_LIMIT;
             position++) {

            int totalTasksAtPosition =
                    StaticCounters
                            .numberOfTasksAtPositionInSharedInstances[
                            position
                            ];

            if (totalTasksAtPosition > 0) {
                int sharedTasksAtPosition =
                        StaticCounters
                                .numberSharedTasksAtPosition[
                                position
                                ];

                double proportion =
                        (double) sharedTasksAtPosition
                                / totalTasksAtPosition;

                System.out.printf(
                        Locale.US,
                        "\t%d: %.4f (%.2f%%) [%d/%d]%n",
                        position,
                        proportion,
                        100.0 * proportion,
                        sharedTasksAtPosition,
                        totalTasksAtPosition
                );
            }
        }
    }

    /**
     * Estimates:
     *
     * P(position p | task is shared)
     */
    private void printSharedTaskPositionDistribution() {
        int totalSharedTaskOccurrences = 0;

        for (int position = 1;
             position < POSITION_LIMIT;
             position++) {

            totalSharedTaskOccurrences +=
                    StaticCounters
                            .numberSharedTasksAtPosition[position];
        }

        System.out.println(
                "Among all shared tasks, percentage appearing "
                        + "at Position:"
        );

        if (totalSharedTaskOccurrences == 0) {
            System.out.println(
                    "\tNo shared-task occurrences were observed."
            );
            return;
        }

        for (int position = 1;
             position < POSITION_LIMIT;
             position++) {

            int sharedTasksAtPosition =
                    StaticCounters
                            .numberSharedTasksAtPosition[position];

            double percentage =
                    100.0
                            * sharedTasksAtPosition
                            / totalSharedTaskOccurrences;

            System.out.printf(
                    Locale.US,
                    "\t%d: %.2f%% [%d/%d]%n",
                    position,
                    percentage,
                    sharedTasksAtPosition,
                    totalSharedTaskOccurrences
            );
        }
    }

    /**
     * Writes detailed instance-level metrics for statistical analysis.
     */
    public void writePriorityMetricsCsv(Path outputFile)
            throws IOException {

        PriorityOrderStatistics.writeInstanceCsv(outputFile);
    }

    public void writeTowerMetricsCsv(Path outputFile)
            throws IOException {

        TowerRouteStatistics.writeInstanceCsv(outputFile);
    }
}