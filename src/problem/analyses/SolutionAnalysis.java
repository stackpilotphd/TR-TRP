package problem.analyses;

import problem.graph.Data;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.regex.Pattern;
import java.util.stream.Stream;

public class SolutionAnalysis {

    private final String pathLink =
            "summary/multithread/solutions";

    private final String filename;
    private final Double heuristicGapPercent;
    private final ZoneAttributeProvider zoneAttributes;
    private final TowerRouteFormat towerRouteFormat;

    /**
     * Describes how a tower-route line is encoded in the solution file.
     */
    public enum TowerRouteFormat {
        /** Example: [zone1, zone2, zone3]. */
        ZONES_ONLY,

        /** Example: [depot, zone1, zone2, zone3, depot]. */
        DEPOT_AT_BOTH_ENDS,

        /** Example: [depot, zone1, zone2, zone3]. */
        DEPOT_AT_START_ONLY,

        /** Example: [zone1, zone2, zone3, depot]. */
        DEPOT_AT_END_ONLY
    }

    /**
     * Existing crew-only constructor.
     */
    public SolutionAnalysis(String filename) {
        this(
                filename,
                null,
                null,
                TowerRouteFormat.DEPOT_AT_BOTH_ENDS
        );
    }

    /**
     * Existing crew-only constructor with a known heuristic gap.
     */
    public SolutionAnalysis(
            String filename,
            Double heuristicGapPercent
    ) {
        this(
                filename,
                heuristicGapPercent,
                null,
                TowerRouteFormat.DEPOT_AT_BOTH_ENDS
        );
    }

    /**
     * Constructor that enables tower-route analysis.
     */
    public SolutionAnalysis(
            String filename,
            ZoneAttributeProvider zoneAttributes,
            TowerRouteFormat towerRouteFormat
    ) {
        this(
                filename,
                null,
                zoneAttributes,
                towerRouteFormat
        );
    }

    /**
     * Constructor that enables both heuristic-gap and tower-route analysis.
     */
    public SolutionAnalysis(
            String filename,
            Double heuristicGapPercent,
            ZoneAttributeProvider zoneAttributes,
            TowerRouteFormat towerRouteFormat
    ) {
        this.filename = Objects.requireNonNull(filename, "filename");
        this.heuristicGapPercent = heuristicGapPercent;
        this.zoneAttributes = zoneAttributes;
        this.towerRouteFormat = Objects.requireNonNull(
                towerRouteFormat,
                "towerRouteFormat"
        );
    }

    /**
     * Existing crew-only constructor when objective values are available.
     */
    public SolutionAnalysis(
            String filename,
            double exactObjective,
            double heuristicObjective,
            PriorityOrderStatistics.ObjectiveSense objectiveSense
    ) {
        this(
                filename,
                PriorityOrderStatistics.calculateGapPercent(
                        exactObjective,
                        heuristicObjective,
                        objectiveSense
                ),
                null,
                TowerRouteFormat.DEPOT_AT_BOTH_ENDS
        );
    }

    /**
     * Constructor for objective-gap and tower-route analysis together.
     */
    public SolutionAnalysis(
            String filename,
            double exactObjective,
            double heuristicObjective,
            PriorityOrderStatistics.ObjectiveSense objectiveSense,
            ZoneAttributeProvider zoneAttributes,
            TowerRouteFormat towerRouteFormat
    ) {
        this(
                filename,
                PriorityOrderStatistics.calculateGapPercent(
                        exactObjective,
                        heuristicObjective,
                        objectiveSense
                ),
                zoneAttributes,
                towerRouteFormat
        );
    }

    public void analyze() throws IOException {
        Data data = Data.getInstance();

        double[] taskWeight = data.getTaskWeight();
        double[] repairTime = data.getServiceTimeMatrix();
        boolean[] isShared = data.getIsShared();
        int[] priority = data.getTaskPriority();
        int towerNum = data.getTowerNumber();

        boolean instanceHasSharedTask = false;
        int numberOfSharedTasksInInstance = 0;

        for (boolean shared : isShared) {
            if (shared) {
                instanceHasSharedTask = true;
                numberOfSharedTasksInInstance++;
            }
        }

        Path filePath = findSolutionFile(
                towerNum,
                Path.of(pathLink),
                filename
        );

        String instanceId =
                Path.of(filename).getFileName()
                        + "|tower="
                        + towerNum;

        PriorityOrderStatistics.InstanceBuilder priorityBuilder =
                new PriorityOrderStatistics.InstanceBuilder(
                        instanceId,
                        numberOfSharedTasksInInstance,
                        heuristicGapPercent
                );

        TowerRouteStatistics.InstanceBuilder towerBuilder =
                zoneAttributes == null
                        ? null
                        : new TowerRouteStatistics.InstanceBuilder(
                        instanceId
                );

        boolean readingTowerRoutes = false;
        boolean towerRouteSectionFound = false;

        for (String originalLine : Files.readAllLines(filePath)) {
            String line = originalLine.trim();

            if (line.isEmpty()) {
                continue;
            }

            if (line.equals("Technician Plans")) {
                readingTowerRoutes = false;
                continue;
            }

            if (line.equals("Tower Routes")) {
                readingTowerRoutes = true;
                towerRouteSectionFound = true;
                continue;
            }

            List<Integer> route = parseRoute(line);

            if (readingTowerRoutes) {
                if (towerBuilder == null) {
                    continue;
                }

                List<Integer> visitedZones = extractVisitedZones(
                        route,
                        towerRouteFormat
                );

                towerBuilder.addRoute(
                        visitedZones,
                        zoneAttributes
                );

                continue;
            }

            /*
             * Technician route:
             * [start depot, task 1, ..., task n, end depot]
             */
            if (route.size() < 3) {
                continue;
            }

            updatePositionalStatistics(
                    route,
                    taskWeight,
                    repairTime,
                    isShared,
                    priority,
                    instanceHasSharedTask
            );

            PriorityOrderStatistics.RouteMetrics routeMetrics =
                    PriorityOrderStatistics.analyzeRoute(
                            route,
                            priority
                    );

            priorityBuilder.addRoute(routeMetrics);
        }

        PriorityOrderStatistics.recordInstance(
                priorityBuilder.build()
        );

        if (towerBuilder != null && towerRouteSectionFound) {
            TowerRouteStatistics.recordInstance(
                    towerBuilder.build()
            );
        }
    }

    /**
     * Parses either a plain list such as "[1, 2, 3]" or a labelled line
     * such as "Tower 1: [1, 2, 3]".
     */
    private List<Integer> parseRoute(String line) {
        String routeText = line;

        int openingBracket = line.indexOf('[');
        int closingBracket = line.lastIndexOf(']');

        if (openingBracket >= 0
                && closingBracket > openingBracket) {
            routeText = line.substring(
                    openingBracket + 1,
                    closingBracket
            );
        }

        routeText = routeText.trim();

        if (routeText.isEmpty()) {
            return List.of();
        }

        String[] tokens = routeText.split(",");
        List<Integer> route = new ArrayList<>(tokens.length);

        for (String token : tokens) {
            route.add(Integer.parseInt(token.trim()));
        }

        return route;
    }

    private List<Integer> extractVisitedZones(
            List<Integer> encodedRoute,
            TowerRouteFormat format
    ) {
        if (encodedRoute.isEmpty()) {
            return List.of();
        }

        int fromIndex = 0;
        int toIndex = encodedRoute.size();

        switch (format) {
            case ZONES_ONLY:
                break;

            case DEPOT_AT_BOTH_ENDS:
                fromIndex = 1;
                toIndex = encodedRoute.size() - 1;
                break;

            case DEPOT_AT_START_ONLY:
                fromIndex = 1;
                break;

            case DEPOT_AT_END_ONLY:
                toIndex = encodedRoute.size() - 1;
                break;

            default:
                throw new IllegalStateException(
                        "Unsupported tower-route format: " + format
                );
        }

        if (fromIndex >= toIndex) {
            return List.of();
        }

        return new ArrayList<>(
                encodedRoute.subList(fromIndex, toIndex)
        );
    }

    private void updatePositionalStatistics(
            List<Integer> route,
            double[] taskWeight,
            double[] repairTime,
            boolean[] isShared,
            int[] priority,
            boolean instanceHasSharedTask
    ) {
        int routeTaskCount = route.size() - 2;

        for (int position = 1;
             position < route.size() - 1;
             position++) {

            int task = route.get(position);

            double weight = taskWeight[task];
            double serviceDuration = repairTime[task];
            int taskPriority = priority[task];
            boolean shared = isShared[task];

            StaticCounters.numberOfTasksAtPosition[position]++;
            StaticCounters.totalWeightAtPosition[position] += weight;
            StaticCounters.totalServiceAtPosition[position] +=
                    serviceDuration;
            StaticCounters.priorityCounterAtPosition[position] +=
                    taskPriority;

            if (instanceHasSharedTask) {
                StaticCounters
                        .numberOfTasksAtPositionInSharedInstances[position]++;

                if (shared) {
                    StaticCounters
                            .numberSharedTasksAtPosition[position]++;
                }
            }

            PositionalStatistics.recordTask(
                    position,
                    routeTaskCount,
                    weight,
                    serviceDuration,
                    taskPriority,
                    shared
            );
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
}
