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
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Collects, for optimal solutions only, the time required to complete
 * prescribed proportions of tasks and to restore prescribed proportions
 * of outage zones.
 *
 * Tower decisions are ignored.  All times are reconstructed exclusively
 * from the technician routes stored in the optimal solution file.
 *
 * Data.getInstance() must already contain the instance being processed.
 */
public final class OptimalServiceRestoredStatistics {

    private static final Path OPTIMAL_SUMMARY =
            Paths.get("isOptimal", "opt_summary.csv");

    private static final Path CREW_ONLY_SOLUTIONS =
            Paths.get("summary", "crewOnly", "solutions");

    private static final Path MULTITHREAD_SOLUTIONS =
            Paths.get("summary", "multithread", "solutions");

    private static final Path OUTPUT_CSV =
            Paths.get(
                    "optimalObjectiveStatistics",
                    "serviceRestored.csv");

    private static final double HORIZON = 600.0;
    private static final double EPS = 1e-9;

    private static final int[] PROPORTIONS = {
            10, 20, 25, 30, 40, 50, 60,
            70, 75, 80, 90, 95, 100
    };

    private static final String TECHNICIAN_SECTION = "Technician Plans";
    private static final String TOWER_SECTION = "Tower Routes";

    private static final Pattern ROUTE_PATTERN =
            Pattern.compile("\\[(.*?)]");

    private OptimalServiceRestoredStatistics() {
    }

    /**
     * Collects one CSV row for an optimal (filename, towerNumber) pair.
     *
     * For a requested proportion p and N objects, the reported value is the
     * earliest time at which at least p percent of the objects are complete.
     * Equivalently, after sorting completion times increasingly, it is the
     * ceil(p*N/100)-th completion time.
     *
     * @return true if the pair is marked optimal and a row was written;
     *         false if the pair is marked non-optimal.
     */
    public static boolean collect(String filename, int towerNumber)
            throws IOException {

        if (filename == null || filename.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "filename must not be blank.");
        }

        if (towerNumber < 0) {
            throw new IllegalArgumentException(
                    "towerNumber must be >= 0.");
        }

        boolean isOptimal =
                readOptimality(filename, towerNumber);

        if (!isOptimal) {
            return false;
        }

        Data data = Data.getInstance();
        validateData(data);

        Path solutionDirectory =
                towerNumber == 0
                        ? CREW_ONLY_SOLUTIONS
                        : MULTITHREAD_SOLUTIONS;

        Path solutionFile =
                findSolutionFile(
                        solutionDirectory,
                        filename,
                        towerNumber);

        List<List<Integer>> technicianPlans =
                readTechnicianPlans(solutionFile);

        if (technicianPlans.isEmpty()) {
            throw new IllegalArgumentException(
                    "No technician plans found in solution file: "
                            + solutionFile);
        }

        Map<Integer, Double> taskCompletion =
                computeTaskCompletionTimes(
                        data,
                        technicianPlans);

        Map<Integer, Double> zoneCompletion =
                computeZoneCompletionTimes(
                        data.getZone2tasks(),
                        taskCompletion);

        validateCompletionTimes(
                "task",
                taskCompletion);

        validateCompletionTimes(
                "zone",
                zoneCompletion);

        double[] taskTimes =
                timesForProportions(
                        taskCompletion.values(),
                        "tasks");

        double[] zoneTimes =
                timesForProportions(
                        zoneCompletion.values(),
                        "zones");

        appendStatistics(
                filename,
                towerNumber,
                taskTimes,
                zoneTimes);

        return true;
    }

    // =====================================================================
    // PROPORTION STATISTICS
    // =====================================================================

    /**
     * Returns the earliest time at which at least each requested proportion
     * has been completed/restored.
     *
     * Example: for 30 tasks and p=25%, ceil(0.25*30)=8, so the returned
     * value is the 8th-smallest task completion time.
     */
    private static double[] timesForProportions(
            Iterable<Double> completionTimes,
            String label) {

        List<Double> sorted = new ArrayList<Double>();

        for (Double value : completionTimes) {
            if (value == null || !Double.isFinite(value)) {
                throw new IllegalArgumentException(
                        "Non-finite completion time found for " + label + ".");
            }
            sorted.add(value);
        }

        if (sorted.isEmpty()) {
            throw new IllegalArgumentException(
                    "No completion times available for " + label + ".");
        }

        Collections.sort(sorted);

        double[] result =
                new double[PROPORTIONS.length];

        int total = sorted.size();

        for (int i = 0; i < PROPORTIONS.length; i++) {
            int percent = PROPORTIONS[i];

            int requiredCount =
                    (int) Math.ceil(
                            (percent / 100.0) * total - EPS);

            /*
             * PROPORTIONS contains only positive values, but keep the
             * bounds explicit in case the list is changed later.
             */
            requiredCount =
                    Math.max(1, Math.min(total, requiredCount));

            result[i] =
                    sorted.get(requiredCount - 1);
        }

        return result;
    }

    // =====================================================================
    // OPTIMALITY CSV
    // =====================================================================

    /**
     * Uses the last matching row if the optimality CSV contains duplicates.
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

                if (!rowFilename.equals(filename)) {
                    continue;
                }

                int rowTowerNumber;

                try {
                    rowTowerNumber =
                            Integer.parseInt(
                                    fields.get(towerIndex).trim());
                } catch (NumberFormatException e) {
                    throw new IllegalArgumentException(
                            "Invalid towerNumber at row "
                                    + lineNumber,
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

    // =====================================================================
    // SOLUTION FILE
    // =====================================================================

    /**
     * Finds a solution named
     * <instanceBase>-<crewCount>-<towerNumber>.txt
     * below the selected solution directory.
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

        String baseName = filename;
        int dotIndex = baseName.lastIndexOf('.');

        if (dotIndex > 0) {
            baseName = baseName.substring(0, dotIndex);
        }

        Pattern solutionPattern =
                Pattern.compile(
                        "^"
                                + Pattern.quote(baseName)
                                + "-(\\d+)-"
                                + towerNumber
                                + "\\.txt$");

        List<Path> matches =
                new ArrayList<Path>();

        try (Stream<Path> paths = Files.walk(solutionDirectory)) {
            paths.filter(Files::isRegularFile)
                    .filter(path ->
                            solutionPattern
                                    .matcher(
                                            path.getFileName().toString())
                                    .matches())
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

        return matches.get(0);
    }

    // =====================================================================
    // TECHNICIAN ROUTES
    // =====================================================================

    /**
     * Reads only the routes between "Technician Plans" and "Tower Routes".
     * Tower routes themselves are deliberately ignored.
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

                if (nodes.size() < 3) {
                    throw new IllegalArgumentException(
                            "Technician route must contain depot, "
                                    + "at least one task, and terminal node: "
                                    + line);
                }

                if (nodes.get(0).intValue() != 0) {
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

    // =====================================================================
    // TASK COMPLETION TIMES
    // =====================================================================

    /**
     * Reconstructs task completion times solely from the technician routes:
     * previous completion time + travel time + task service time.
     *
     * The first route node is the depot and the last node is the dummy sink,
     * so only route positions 1 through size-2 are real tasks.
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

            for (int i = 1;
                 i < route.size() - 1;
                 i++) {

                int taskId = route.get(i);

                currentTime +=
                        travelTime(
                                travelTimes,
                                previousNode,
                                taskId);

                currentTime +=
                        serviceTimes.get(taskId);

                Double previousCompletion =
                        completionTimes.get(taskId);

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

    // =====================================================================
    // ZONE RESTORATION TIMES
    // =====================================================================

    /**
     * A zone is restored when all tasks associated with that zone are complete.
     * Therefore its restoration time is the maximum completion time among its
     * associated tasks.
     *
     * Shared tasks need no special treatment: the same task completion time is
     * simply used in every zone that contains that task.
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

    // =====================================================================
    // OUTPUT CSV
    // =====================================================================

    private static void appendStatistics(
            String filename,
            int towerNumber,
            double[] taskTimes,
            double[] zoneTimes) throws IOException {

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
                    .append("TRUE");

            /* First all task-proportion times. */
            for (double value : taskTimes) {
                row.append(',')
                        .append(formatTime(value));
            }

            /* Then all zone-proportion times. */
            for (double value : zoneTimes) {
                row.append(',')
                        .append(formatTime(value));
            }

            writer.write(row.toString());
            writer.newLine();
        }
    }

    private static void writeHeader(
            BufferedWriter writer) throws IOException {

        StringBuilder header =
                new StringBuilder(
                        "filename,towerNumber,isOptimal");

        for (int percent : PROPORTIONS) {
            header.append(",Tasks")
                    .append(percent)
                    .append("Pct");
        }

        for (int percent : PROPORTIONS) {
            header.append(",Zones")
                    .append(percent)
                    .append("Pct");
        }

        writer.write(header.toString());
        writer.newLine();
    }

    private static String formatTime(double value) {
        String result =
                String.format(
                        Locale.ROOT,
                        "%.6f",
                        value);

        return result.replaceAll("0+$", "")
                .replaceAll("\\.$", "");
    }

    // =====================================================================
    // VALIDATION / DATA HELPERS
    // =====================================================================

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

    private static void validateCompletionTimes(
            String label,
            Map<Integer, Double> completionTimes) {

        if (completionTimes == null || completionTimes.isEmpty()) {
            throw new IllegalArgumentException(
                    "No " + label + " completion times were reconstructed.");
        }

        for (Map.Entry<Integer, Double> entry
                : completionTimes.entrySet()) {

            Double time = entry.getValue();

            if (time == null || !Double.isFinite(time)) {
                throw new IllegalArgumentException(
                        "Invalid " + label
                                + " completion time for id "
                                + entry.getKey()
                                + ": " + time);
            }

            if (time < -EPS || time > HORIZON + EPS) {
                throw new IllegalArgumentException(
                        label + " completion time for id "
                                + entry.getKey()
                                + " is outside the planning horizon [0, "
                                + HORIZON + "]: " + time);
            }
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
     * Supports both common service-time layouts:
     * service[taskId] and service[taskId - 1].
     */
    private static final class ServiceTimeAccessor {

        private final double[] service;
        private final boolean directNodeIndexing;

        private ServiceTimeAccessor(
                double[] service,
                boolean directNodeIndexing) {

            this.service = service;
            this.directNodeIndexing = directNodeIndexing;
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
                    "Service-time array is too short for task ID "
                            + maxTaskId
                            + ". Length="
                            + service.length);
        }

        private double get(int taskId) {
            int index =
                    directNodeIndexing
                            ? taskId
                            : taskId - 1;

            if (index < 0 || index >= service.length) {
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

    // =====================================================================
    // PARSING HELPERS
    // =====================================================================

    private static List<Integer> parseIntegerList(
            String body) {

        List<Integer> values =
                new ArrayList<Integer>();

        String[] parts = body.split(",");

        for (String part : parts) {
            String value = part.trim();

            if (!value.isEmpty()) {
                values.add(
                        Integer.parseInt(value));
            }
        }

        return values;
    }

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
            } else if (c == ',' && !inQuotes) {
                fields.add(current.toString());
                current.setLength(0);
            } else {
                current.append(c);
            }
        }

        fields.add(current.toString());
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
                value.replace("\"", "\"\"");

        return quote
                ? "\"" + escaped + "\""
                : escaped;
    }
}
