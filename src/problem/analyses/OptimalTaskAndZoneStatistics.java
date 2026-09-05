package problem.analyses;

import problem.graph.Data;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

public final class OptimalTaskAndZoneStatistics {

    private static final Path OPTIMAL_SUMMARY =
            Paths.get("isOptimal", "opt_summary.csv");

    private static final Path CREW_ONLY_SOLUTIONS =
            Paths.get("summary", "crewOnly", "solutions");

    private static final Path MULTITHREAD_SOLUTIONS =
            Paths.get("summary", "multithread", "solutions");

    private static final Path OUTPUT_CSV =
            Paths.get(
                    "gantChartExperiments",
                    "allOptimal",
                    "taskAndZoneStatistics.csv");

    private static final int HORIZON = 600;
    private static final int INTERVAL = 100;

    private static final String TECHNICIAN_SECTION = "Technician Plans";
    private static final String TOWER_SECTION = "Tower Routes";

    private static final Pattern ROUTE_PATTERN =
            Pattern.compile("\\[(.*?)]");

    private OptimalTaskAndZoneStatistics() {
    }

    /**
     * Checks whether (filename, towerNumber) is optimal according to
     * isOptimal/opt_summary.csv.
     *
     * If it is not optimal, returns false immediately and does nothing else.
     *
     * If it is optimal:
     *   1. locates the corresponding solution file;
     *   2. reconstructs technician task completion times;
     *   3. reconstructs zone completion times;
     *   4. appends the statistics to
     *      gantChartExperiments/allOptimal/taskAndZoneStatistics.csv.
     *
     * Data.getInstance() must already have been populated by the instance reader.
     *
     * @return true if the instance is optimal and its statistics were exported;
     *         false if the instance is marked non-optimal.
     */
    public static boolean collect(String filename, int towerNumber)
            throws IOException {

        if (filename == null || filename.trim().isEmpty()) {
            throw new IllegalArgumentException("filename must not be blank.");
        }

        if (towerNumber < 0) {
            throw new IllegalArgumentException(
                    "towerNumber must be >= 0.");
        }

        /*
         * IMPORTANT:
         * Do this first. If the instance is not optimal, nothing else happens.
         */
        boolean isOptimal =
                readOptimality(filename, towerNumber);

        if (!isOptimal) {
            return false;
        }

        /*
         * Only obtain/use Data after we know the solution is optimal.
         */
        Data data = Data.getInstance();
        validateData(data);

        Path solutionDirectory =
                towerNumber == 0
                        ? CREW_ONLY_SOLUTIONS
                        : MULTITHREAD_SOLUTIONS;

        Path solutionFile =
                findSolutionFile(solutionDirectory, filename,towerNumber);

        List<List<Integer>> technicianPlans =
                readTechnicianPlans(solutionFile);

        if (technicianPlans.isEmpty()) {
            throw new IllegalArgumentException(
                    "No technician plans found in solution file: "
                            + solutionFile);
        }

        /*
         * Reconstruct completion time of every task.
         */
        Map<Integer, Double> taskCompletion =
                computeTaskCompletionTimes(data, technicianPlans);

        /*
         * A zone completes when all tasks belonging to that zone
         * have completed.
         */
        Map<Integer, Double> zoneCompletion =
                computeZoneCompletionTimes(
                        data.getZone2tasks(),
                        taskCompletion);

        appendStatistics(
                filename,
                towerNumber,
                true,
                taskCompletion,
                zoneCompletion);

        return true;
    }

    // ========================================================================
    // OPTIMALITY CSV
    // ========================================================================

    /**
     * Finds the row in isOptimal/opt_summary.csv having both the requested
     * filename and towerNumber.
     *
     * If duplicate rows exist, the LAST matching row is used, since the
     * optimality CSV is append-based.
     */
    private static boolean readOptimality(
            String filename,
            int towerNumber) throws IOException {

        if (!Files.isRegularFile(OPTIMAL_SUMMARY)) {
            throw new IllegalArgumentException(
                    "Optimality summary does not exist: "
                            + OPTIMAL_SUMMARY.toAbsolutePath());
        }

        try (BufferedReader reader =
                     Files.newBufferedReader(
                             OPTIMAL_SUMMARY,
                             StandardCharsets.UTF_8)) {

            String headerLine = reader.readLine();

            if (headerLine == null) {
                throw new IllegalArgumentException(
                        "Optimality summary is empty: "
                                + OPTIMAL_SUMMARY.toAbsolutePath());
            }

            List<String> header =
                    parseCsvLine(headerLine);

            int filenameIndex =
                    findColumn(header, "filename");

            int towerIndex =
                    findColumn(header, "towerNumber");

            int optimalIndex =
                    findColumn(header, "isOptimal");

            Boolean result = null;

            String line;
            int lineNumber = 1;

            while ((line = reader.readLine()) != null) {
                lineNumber++;

                if (line.trim().isEmpty()) {
                    continue;
                }

                List<String> fields =
                        parseCsvLine(line);

                int requiredSize =
                        Math.max(
                                filenameIndex,
                                Math.max(
                                        towerIndex,
                                        optimalIndex))
                                + 1;

                if (fields.size() < requiredSize) {
                    throw new IllegalArgumentException(
                            "Malformed row " + lineNumber
                                    + " in " + OPTIMAL_SUMMARY
                                    + ": " + line);
                }

                String rowFilename =
                        fields.get(filenameIndex).trim();

                String rowTower =
                        fields.get(towerIndex).trim();

                if (!rowFilename.equals(filename)) {
                    continue;
                }

                int rowTowerNumber;

                try {
                    rowTowerNumber =
                            Integer.parseInt(rowTower);
                } catch (NumberFormatException e) {
                    throw new IllegalArgumentException(
                            "Invalid towerNumber at row "
                                    + lineNumber + ": " + rowTower,
                            e);
                }

                if (rowTowerNumber != towerNumber) {
                    continue;
                }

                String value =
                        fields.get(optimalIndex).trim();

                if (value.equalsIgnoreCase("true")) {
                    result = Boolean.TRUE;
                } else if (value.equalsIgnoreCase("false")) {
                    result = Boolean.FALSE;
                } else {
                    throw new IllegalArgumentException(
                            "Invalid isOptimal value at row "
                                    + lineNumber + ": " + value);
                }
            }

            if (result == null) {
                throw new IllegalArgumentException(
                        "No optimality entry found for filename='"
                                + filename
                                + "', towerNumber="
                                + towerNumber
                                + " in "
                                + OPTIMAL_SUMMARY.toAbsolutePath());
            }

            return result.booleanValue();
        }
    }

    private static int findColumn(
            List<String> header,
            String wanted) {

        for (int i = 0; i < header.size(); i++) {
            if (header.get(i).trim().equalsIgnoreCase(wanted)) {
                return i;
            }
        }

        throw new IllegalArgumentException(
                "CSV column not found: " + wanted);
    }

    // ========================================================================
    // SOLUTION FILE
    // ========================================================================
    /**
     * Finds the solution file corresponding to the given instance and tower count.
     *
     * The solution filename has the form:
     *
     *     <instanceBase>-<crewCount>-<towerNumber>.txt
     *
     * Examples:
     *
     *     tATL_6-20-9.txt, towerNumber=0
     *         -> tATL_6-20-9-6-0.txt
     *
     *     tCLT_6-20-1.txt, towerNumber=0
     *         -> tCLT_6-20-1-7-0.txt
     *
     * The crew count is NOT inferred from the instance filename. Instead, any
     * integer crew count is accepted, provided exactly one matching solution
     * exists.
     */
    private static Path findSolutionFile(
            Path solutionDirectory,
            String filename,
            int towerNumber) throws IOException {

        if (!Files.isDirectory(solutionDirectory)) {
            throw new IllegalArgumentException(
                    "Solution directory does not exist: "
                            + solutionDirectory.toAbsolutePath());
        }

        if (filename == null || filename.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "filename must not be blank.");
        }

        if (towerNumber < 0) {
            throw new IllegalArgumentException(
                    "towerNumber must be >= 0.");
        }

        /*
         * Remove the extension:
         *
         * tCLT_6-20-1.txt
         * ->
         * tCLT_6-20-1
         */
        String baseName = filename;

        int dotIndex = baseName.lastIndexOf('.');

        if (dotIndex > 0) {
            baseName = baseName.substring(0, dotIndex);
        }

        /*
         * Match:
         *
         * baseName-<crewCount>-<towerNumber>.txt
         *
         * Example:
         *
         * tCLT_6-20-1-7-0.txt
         */
        Pattern solutionPattern = Pattern.compile(
                "^"
                        + Pattern.quote(baseName)
                        + "-(\\d+)-"
                        + towerNumber
                        + "\\.txt$"
        );

        List<Path> matches = new ArrayList<Path>();

        try (Stream<Path> paths = Files.walk(solutionDirectory)) {

            paths.filter(Files::isRegularFile)
                    .filter(path -> {
                        String candidate =
                                path.getFileName().toString();

                        return solutionPattern
                                .matcher(candidate)
                                .matches();
                    })
                    .forEach(matches::add);
        }

        if (matches.isEmpty()) {
            throw new IllegalArgumentException(
                    "Could not find solution file matching '"
                            + baseName
                            + "-<crewCount>-"
                            + towerNumber
                            + ".txt' below "
                            + solutionDirectory.toAbsolutePath());
        }

        if (matches.size() > 1) {
            throw new IllegalArgumentException(
                    "More than one solution file matched instance '"
                            + filename
                            + "' with towerNumber="
                            + towerNumber
                            + ": "
                            + matches);
        }

        Path solutionFile = matches.get(0);

        System.out.println(
                "Matched solution file: "
                        + solutionFile.getFileName());

        return solutionFile;
    }

    // ========================================================================
    // TECHNICIAN PLANS
    // ========================================================================

    /**
     * Reads:
     *
     * Technician Plans
     * [0, 20, 21]
     * [0, 6, 18, 21]
     * ...
     * Tower Routes
     *
     * Only the routes between Technician Plans and Tower Routes are read.
     */
    private static List<List<Integer>> readTechnicianPlans(
            Path solutionFile) throws IOException {

        List<List<Integer>> routes =
                new ArrayList<List<Integer>>();

        boolean inTechnicianSection = false;

        try (BufferedReader reader =
                     Files.newBufferedReader(
                             solutionFile,
                             StandardCharsets.UTF_8)) {

            String line;

            while ((line = reader.readLine()) != null) {

                String trimmed = line.trim();

                if (isHeading(
                        trimmed,
                        TECHNICIAN_SECTION)) {

                    inTechnicianSection = true;
                    continue;
                }

                if (inTechnicianSection
                        && isHeading(
                        trimmed,
                        TOWER_SECTION)) {
                    break;
                }

                if (!inTechnicianSection
                        || trimmed.isEmpty()) {
                    continue;
                }

                Matcher matcher =
                        ROUTE_PATTERN.matcher(trimmed);

                if (!matcher.find()) {
                    continue;
                }

                List<Integer> nodes =
                        parseIntegerList(
                                matcher.group(1));

                /*
                 * Expected:
                 * depot 0 -> at least one task -> dummy sink.
                 */
                if (nodes.size() < 3) {
                    throw new IllegalArgumentException(
                            "Technician route must contain depot, "
                                    + "at least one task, and terminal node: "
                                    + line);
                }

                if (nodes.get(0) != 0) {
                    throw new IllegalArgumentException(
                            "Technician route does not begin at depot 0: "
                                    + line);
                }

                routes.add(nodes);
            }
        }

        return routes;
    }

    private static boolean isHeading(
            String line,
            String heading) {

        return line.equalsIgnoreCase(heading)
                || line.equalsIgnoreCase(heading + ":");
    }

    // ========================================================================
    // TASK COMPLETION TIMES
    // ========================================================================

    /**
     * Reconstructs each technician's schedule.
     *
     * Every technician begins at time 0.
     *
     * For each real task:
     *
     * current time
     *   + travel(previousNode, task)
     *   + service(task)
     *   = task completion time
     *
     * The final node in each route is ignored because it is the dummy sink.
     */
    private static Map<Integer, Double> computeTaskCompletionTimes(
            Data data,
            List<List<Integer>> routes) {

        double[][] travelTimes =
                data.getCrewTravelTimeMatrix();

        ServiceTimeAccessor serviceTimes =
                ServiceTimeAccessor.forRoutes(
                        data.getServiceTimeMatrix(),
                        routes);

        Map<Integer, Double> completionTimes =
                new HashMap<Integer, Double>();

        for (List<Integer> route : routes) {

            double currentTime = 0.0;
            int previousNode = 0;

            /*
             * route[0] = depot
             * route[last] = dummy sink
             *
             * Therefore real tasks are 1 .. size-2.
             */
            for (int i = 1;
                 i < route.size() - 1;
                 i++) {

                int taskId =
                        route.get(i);

                double travel =
                        travelTime(
                                travelTimes,
                                previousNode,
                                taskId);

                currentTime += travel;

                double service =
                        serviceTimes.get(taskId);

                currentTime += service;

                Double previousCompletion =
                        completionTimes.get(taskId);

                /*
                 * Normally each task appears exactly once.
                 * max() makes this safe if a task unexpectedly
                 * occurs more than once.
                 */
                if (previousCompletion == null
                        || currentTime > previousCompletion) {

                    completionTimes.put(
                            taskId,
                            currentTime);
                }

                previousNode = taskId;
            }
        }

        return completionTimes;
    }

    // ========================================================================
    // ZONE COMPLETION TIMES
    // ========================================================================

    /**
     * A zone is complete at the completion time of its latest task.
     *
     * If one of the tasks belonging to a zone does not appear in the
     * technician schedule, that zone receives +infinity and is therefore
     * counted as incomplete at all finite milestones.
     *
     * This mirrors the zone logic in CrewScheduleCsvExporter.
     */
    private static Map<Integer, Double> computeZoneCompletionTimes(
            ArrayList<Integer>[] zone2tasks,
            Map<Integer, Double> taskCompletion) {

        if (zone2tasks == null) {
            throw new IllegalStateException(
                    "Data.getZone2tasks() is null; "
                            + "zone statistics cannot be computed.");
        }

        Map<Integer, Double> result =
                new LinkedHashMap<Integer, Double>();

        /*
         * Supports:
         *
         * direct indexing:
         *   zone2tasks[zoneId]
         *   with index 0 unused
         *
         * compact indexing:
         *   zone2tasks[zoneId - 1]
         */
        boolean directIndexing =
                zone2tasks.length > 1
                        && (zone2tasks[0] == null
                        || zone2tasks[0].isEmpty());

        int firstIndex =
                directIndexing ? 1 : 0;

        for (int arrayIndex = firstIndex;
             arrayIndex < zone2tasks.length;
             arrayIndex++) {

            ArrayList<Integer> tasks =
                    zone2tasks[arrayIndex];

            if (tasks == null || tasks.isEmpty()) {
                continue;
            }

            int zoneId =
                    directIndexing
                            ? arrayIndex
                            : arrayIndex + 1;

            double zoneCompletion = 0.0;
            boolean missingTask = false;

            for (Integer taskId : tasks) {

                if (taskId == null) {
                    continue;
                }

                Double taskTime =
                        taskCompletion.get(taskId);

                if (taskTime == null) {
                    missingTask = true;
                    break;
                }

                zoneCompletion =
                        Math.max(
                                zoneCompletion,
                                taskTime);
            }

            result.put(
                    zoneId,
                    missingTask
                            ? Double.POSITIVE_INFINITY
                            : zoneCompletion);
        }

        return result;
    }

    // ========================================================================
    // OUTPUT CSV
    // ========================================================================

    private static void appendStatistics(
            String filename,
            int towerNumber,
            boolean isOptimal,
            Map<Integer, Double> taskCompletion,
            Map<Integer, Double> zoneCompletion)
            throws IOException {

        Path parent =
                OUTPUT_CSV.toAbsolutePath().getParent();

        if (parent != null) {
            Files.createDirectories(parent);
        }

        boolean writeHeader =
                !Files.exists(OUTPUT_CSV)
                        || Files.size(OUTPUT_CSV) == 0;

        try (BufferedWriter writer =
                     Files.newBufferedWriter(
                             OUTPUT_CSV,
                             StandardCharsets.UTF_8,
                             StandardOpenOption.CREATE,
                             StandardOpenOption.APPEND)) {

            if (writeHeader) {
                writeHeader(writer);
            }

            StringBuilder row =
                    new StringBuilder();

            row.append(csv(filename))
                    .append(',')
                    .append(towerNumber)
                    .append(',')
                    .append(isOptimal);

            /*
             * TasksBy100 ... TasksBy600
             */
            for (int t = INTERVAL;
                 t <= HORIZON;
                 t += INTERVAL) {

                int completed =
                        countCompleted(
                                taskCompletion.values(),
                                t);

                row.append(',')
                        .append(
                                statisticCell(
                                        completed,
                                        taskCompletion.size()));
            }

            /*
             * ZonesBy100 ... ZonesBy600
             */
            for (int t = INTERVAL;
                 t <= HORIZON;
                 t += INTERVAL) {

                int completed =
                        countCompleted(
                                zoneCompletion.values(),
                                t);

                row.append(',')
                        .append(
                                statisticCell(
                                        completed,
                                        zoneCompletion.size()));
            }

            /*
             * Makespan = time at which the last task is completed.
             */
            double makespan =
                    computeMakespan(taskCompletion);

            row.append(',')
                    .append(
                            String.format(
                                    Locale.ROOT,
                                    "%.6f",
                                    makespan));

            writer.write(row.toString());
            writer.newLine();
        }
    }

    private static void writeHeader(
            BufferedWriter writer) throws IOException {

        StringBuilder header =
                new StringBuilder();

        header.append(
                "filename,towerNumber,isOptimal");

        for (int t = INTERVAL;
             t <= HORIZON;
             t += INTERVAL) {

            header.append(",TasksBy")
                    .append(t);
        }

        for (int t = INTERVAL;
             t <= HORIZON;
             t += INTERVAL) {

            header.append(",ZonesBy")
                    .append(t);
        }
        header.append(",Makespan");
        writer.write(header.toString());
        writer.newLine();
    }

    private static double computeMakespan(
            Map<Integer, Double> taskCompletion) {

        if (taskCompletion == null
                || taskCompletion.isEmpty()) {
            return Double.NaN;
        }

        double makespan =
                Double.NEGATIVE_INFINITY;

        for (Double completionTime
                : taskCompletion.values()) {

            if (completionTime != null
                    && Double.isFinite(completionTime)) {

                makespan =
                        Math.max(
                                makespan,
                                completionTime);
            }
        }

        return makespan == Double.NEGATIVE_INFINITY
                ? Double.NaN
                : makespan;
    }

    /**
     * Keeps the exact column structure requested while preserving both
     * count and proportion.
     *
     * Example:
     *
     * 12/20 (0.600000)
     */
    private static String statisticCell(
            int completed,
            int total) {

        if (total == 0) {
            return "0";
        }

        double percentage =
                100.0 * completed / total;

        String result =
                String.format(
                        Locale.ROOT,
                        "%.6f",
                        percentage);

        // Remove unnecessary trailing zeros.
        // Examples:
        // 35.000000   -> 35
        // 50.000000   -> 50
        // 16.666667   -> 16.666667
        result = result.replaceAll("0+$", "")
                .replaceAll("\\.$", "");

        return result;
    }
    private static int countCompleted(
            Iterable<Double> completionTimes,
            double threshold) {

        int count = 0;

        for (Double time : completionTimes) {

            if (time != null
                    && Double.isFinite(time)
                    && time <= threshold + 1e-9) {

                count++;
            }
        }

        return count;
    }

    // ========================================================================
    // DATA / MATRIX HELPERS
    // ========================================================================

    private static void validateData(Data data) {

        if (data == null) {
            throw new IllegalArgumentException(
                    "Data is null.");
        }

        if (data.getServiceTimeMatrix() == null) {
            throw new IllegalStateException(
                    "Data.getServiceTimeMatrix() is null. "
                            + "Run the instance reader first.");
        }

        if (data.getCrewTravelTimeMatrix() == null) {
            throw new IllegalStateException(
                    "Data.getCrewTravelTimeMatrix() is null. "
                            + "Run the instance reader first.");
        }

        if (data.getZone2tasks() == null) {
            throw new IllegalStateException(
                    "Data.getZone2tasks() is null. "
                            + "Run the instance reader first.");
        }
    }

    private static double travelTime(
            double[][] matrix,
            int fromNode,
            int toNode) {

        if (fromNode < 0
                || fromNode >= matrix.length
                || matrix[fromNode] == null
                || toNode < 0
                || toNode >= matrix[fromNode].length) {

            throw new IllegalArgumentException(
                    "Crew travel matrix has no entry for "
                            + fromNode
                            + " -> "
                            + toNode);
        }

        return matrix[fromNode][toNode];
    }

    /**
     * Same service-array interpretation used by CrewScheduleCsvExporter:
     *
     *  A) service[taskId]
     *  B) service[taskId - 1]
     */
    private static final class ServiceTimeAccessor {

        private final double[] service;
        private final boolean directNodeIndexing;

        private ServiceTimeAccessor(
                double[] service,
                boolean directNodeIndexing) {

            this.service = service;
            this.directNodeIndexing =
                    directNodeIndexing;
        }

        private static ServiceTimeAccessor forRoutes(
                double[] service,
                List<List<Integer>> routes) {

            int maxTaskId = 0;

            for (List<Integer> route : routes) {

                for (int i = 1;
                     i < route.size() - 1;
                     i++) {

                    maxTaskId =
                            Math.max(
                                    maxTaskId,
                                    route.get(i));
                }
            }

            if (service.length > maxTaskId) {
                return new ServiceTimeAccessor(
                        service,
                        true);
            }

            if (service.length >= maxTaskId) {
                return new ServiceTimeAccessor(
                        service,
                        false);
            }

            throw new IllegalArgumentException(
                    "Service-time array is too short "
                            + "for task ID "
                            + maxTaskId
                            + ". Length="
                            + service.length);
        }

        private double get(int taskId) {

            int index =
                    directNodeIndexing
                            ? taskId
                            : taskId - 1;

            if (index < 0
                    || index >= service.length) {

                throw new IllegalArgumentException(
                        "No service time for task "
                                + taskId
                                + " (index "
                                + index
                                + ")");
            }

            return service[index];
        }
    }

    // ========================================================================
    // PARSING HELPERS
    // ========================================================================

    private static List<Integer> parseIntegerList(
            String body) {

        List<Integer> values =
                new ArrayList<Integer>();

        String[] parts =
                body.split(",");

        for (String part : parts) {

            String value =
                    part.trim();

            if (!value.isEmpty()) {
                values.add(
                        Integer.parseInt(value));
            }
        }

        return values;
    }

    /**
     * Small CSV parser supporting quoted fields and escaped quotes.
     */
    private static List<String> parseCsvLine(
            String line) {

        List<String> fields =
                new ArrayList<String>();

        StringBuilder current =
                new StringBuilder();

        boolean inQuotes = false;

        for (int i = 0; i < line.length(); i++) {

            char c = line.charAt(i);

            if (c == '"') {

                if (inQuotes
                        && i + 1 < line.length()
                        && line.charAt(i + 1) == '"') {

                    current.append('"');
                    i++;

                } else {
                    inQuotes = !inQuotes;
                }

            } else if (c == ','
                    && !inQuotes) {

                fields.add(
                        current.toString());

                current.setLength(0);

            } else {
                current.append(c);
            }
        }

        fields.add(
                current.toString());

        return fields;
    }

    private static String csv(String value) {

        if (value == null) {
            return "";
        }

        boolean quote =
                value.indexOf(',') >= 0
                        || value.indexOf('"') >= 0
                        || value.indexOf('\n') >= 0
                        || value.indexOf('\r') >= 0;

        String escaped =
                value.replace(
                        "\"",
                        "\"\"");

        return quote
                ? "\"" + escaped + "\""
                : escaped;
    }
}