package problem.analyses;

import problem.graph.Data;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileVisitOption;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Reads the crew routes of ONE scenario from a solution file, sorts the routes
 * by the repair/service duration of their first real task (longest first),
 * computes the sequential crew schedule, and exports one CSV row per task visit.
 *
 * Assumptions matching the supplied experiment files:
 *   - every crew route starts at depot node 0;
 *   - the final route node is a dummy terminal/sink and is not repaired;
 *   - crews do not wait: repair starts immediately on arrival;
 *   - the next travel leg starts when the previous repair finishes;
 *   - task and travel times are already loaded into Data.
 *
 * The core entry point is export(...). Call it after  Instance reader has populated Data.getInstance().
 */
public final class CrewScheduleCsvExporter {

    private static final String CREW_SECTION = "Optimal Repair Crews:";
    private static final String CREW_SECTION_END = "Optimal Repair Schedule:";
    private static final Pattern ROUTE_PATTERN = Pattern.compile("\\[(.*?)]");

    private CrewScheduleCsvExporter() {
    }

    /**
     * Convenience method: writes to <experimentRoot>/ganttCsv/<scenario>_crew_schedule.csv.
     */
    public static Path export(Data data, Path experimentRoot, String scenario) throws IOException {
        Path output = experimentRoot
                .resolve("ganttCsv")
                .resolve(safeFileName(scenario) + "_crew_schedule.csv");
        export(data, experimentRoot, scenario, output);
        return output;
    }

    /**
     * Main integration method. Data must already be initialized by the caller.
     */
    public static void export(
            Data data,
            Path experimentRoot,
            String scenario,
            Path outputCsv) throws IOException {

        validateData(data);

        Path solutionFile = findSolutionFile(experimentRoot, scenario);
        List<CrewRoute> routes = readCrewRoutes(solutionFile);
        if (routes.isEmpty()) {
            throw new IllegalArgumentException(
                    "No crew routes found in solution file: " + solutionFile);
        }

        final ServiceTimeAccessor serviceTimes =
                ServiceTimeAccessor.forRoutes(data.getServiceTimeMatrix(), routes);

        // Requested ordering: longest service duration of the first task first.
        // Deterministic tie-breakers: first task ID, then original route order.
        Collections.sort(routes, new Comparator<CrewRoute>() {
            @Override
            public int compare(CrewRoute a, CrewRoute b) {
                double da = serviceTimes.get(a.firstTask());
                double db = serviceTimes.get(b.firstTask());
                int byDuration = Double.compare(db, da); // descending
                if (byDuration != 0) {
                    return byDuration;
                }
                int byTask = Integer.compare(a.firstTask(), b.firstTask());
                if (byTask != 0) {
                    return byTask;
                }
                return Integer.compare(a.originalRouteIndex, b.originalRouteIndex);
            }
        });

        List<ScheduleRow> rows = buildSchedule(
                scenario,
                routes,
                data.getCrewTravelTimeMatrix(),
                serviceTimes);

        writeCsv(outputCsv, rows);
        printScheduleStatistics(data, rows);

        System.out.println("Scenario       : " + scenario);
        System.out.println("Solution file  : " + solutionFile.toAbsolutePath());
        System.out.println("Crew routes    : " + routes.size());
        System.out.println("CSV output     : " + outputCsv.toAbsolutePath());
    }

    /**
     * Finds scenarios/<scenario>/sol below experimentRoot.
     * Matching ignores case, spaces, '-' and '_' so, for example,
     * "crew_only" matches folder "crewOnly".
     */
    public static Path findSolutionFile(Path experimentRoot, String scenario) throws IOException {
        if (experimentRoot == null || !Files.exists(experimentRoot)) {
            throw new IllegalArgumentException("Experiment root does not exist: " + experimentRoot);
        }
        if (scenario == null || scenario.trim().isEmpty()) {
            throw new IllegalArgumentException("Scenario must not be blank.");
        }

        String wanted = normalizeScenarioName(scenario);

        // Fast path for an exact folder name.
        Path direct = experimentRoot.resolve("scenarios").resolve(scenario).resolve("sol");
        if (Files.isRegularFile(direct)) {
            return direct;
        }

        // Flexible recursive search, useful if experimentRoot is one level above
        // the actual gantChartExperiments directory.
        List<Path> matches = new ArrayList<Path>();
        try (Stream<Path> paths = Files.walk(
                experimentRoot,
                Integer.MAX_VALUE,
                FileVisitOption.FOLLOW_LINKS)) {
            paths.filter(Files::isRegularFile)
                    .filter(p -> p.getFileName().toString().equals("sol"))
                    .forEach(p -> {
                        Path parent = p.getParent();
                        if (parent != null
                                && normalizeScenarioName(parent.getFileName().toString()).equals(wanted)) {
                            matches.add(p);
                        }
                    });
        }

        if (matches.isEmpty()) {
            throw new IllegalArgumentException(
                    "Could not find a solution file for scenario '" + scenario
                            + "' below " + experimentRoot.toAbsolutePath());
        }
        if (matches.size() > 1) {
            throw new IllegalArgumentException(
                    "More than one solution matched scenario '" + scenario + "': " + matches);
        }
        return matches.get(0);
    }

    /** Extracts only the route lines inside the Optimal Repair Crews section. */
    public static List<CrewRoute> readCrewRoutes(Path solutionFile) throws IOException {
        List<CrewRoute> routes = new ArrayList<CrewRoute>();
        boolean inCrewSection = false;
        int originalIndex = 0;

        try (BufferedReader reader = Files.newBufferedReader(solutionFile, StandardCharsets.UTF_8)) {
            String line;
            while ((line = reader.readLine()) != null) {
                String trimmed = line.trim();

                if (trimmed.startsWith(CREW_SECTION)) {
                    inCrewSection = true;
                    continue;
                }
                if (inCrewSection && trimmed.startsWith(CREW_SECTION_END)) {
                    break;
                }
                if (!inCrewSection || trimmed.isEmpty()) {
                    continue;
                }

                Matcher matcher = ROUTE_PATTERN.matcher(trimmed);
                if (!matcher.find()) {
                    continue;
                }

                List<Integer> nodes = parseIntegerList(matcher.group(1));
                if (nodes.size() < 3) {
                    throw new IllegalArgumentException(
                            "Crew route must contain start depot, at least one task, and terminal node: " + line);
                }
                if (nodes.get(0) != 0) {
                    throw new IllegalArgumentException("Crew route does not start at depot 0: " + line);
                }

                originalIndex++;
                routes.add(new CrewRoute(originalIndex, nodes));
            }
        }

        return routes;
    }

    private static List<ScheduleRow> buildSchedule(
            String scenario,
            List<CrewRoute> sortedRoutes,
            double[][] travelTimes,
            ServiceTimeAccessor serviceTimes) {

        validateTravelMatrix(travelTimes);
        List<ScheduleRow> rows = new ArrayList<ScheduleRow>();

        for (int crewRank = 0; crewRank < sortedRoutes.size(); crewRank++) {
            CrewRoute route = sortedRoutes.get(crewRank);
            int previousNode = 0;
            double currentTime = 0.0;
            String crewId = "Crew-" + (crewRank + 1);
            String routeText = routeAsText(route.nodes);
            double firstTaskDuration = serviceTimes.get(route.firstTask());

            // Last node is the dummy sink. Real tasks are indices 1..size-2.
            for (int routePosition = 1; routePosition < route.nodes.size() - 1; routePosition++) {
                int taskId = route.nodes.get(routePosition);

                double travelDuration = travelTime(travelTimes, previousNode, taskId);
                double travelStart = currentTime;
                double travelEnd = travelStart + travelDuration;
                double arrival = travelEnd;

                double repairDuration = serviceTimes.get(taskId);
                double repairStart = arrival;
                double repairEnd = repairStart + repairDuration;
                double completion = repairEnd;

                rows.add(new ScheduleRow(
                        scenario,
                        crewRank + 1,
                        crewId,
                        route.originalRouteIndex,
                        route.firstTask(),
                        firstTaskDuration,
                        routeText,
                        routePosition,
                        previousNode,
                        taskId,
                        travelStart,
                        travelEnd,
                        travelDuration,
                        arrival,
                        repairStart,
                        repairEnd,
                        repairDuration,
                        completion));

                currentTime = completion;
                previousNode = taskId;
            }
        }

        return rows;
    }

    private static void writeCsv(Path outputCsv, List<ScheduleRow> rows) throws IOException {
        Path parent = outputCsv.toAbsolutePath().getParent();
        if (parent != null) {
            Files.createDirectories(parent);
        }

        try (BufferedWriter writer = Files.newBufferedWriter(outputCsv, StandardCharsets.UTF_8)) {
            writer.write(
                    "scenario,crew_rank,crew_id,original_route_index,first_task,"
                            + "first_task_service_duration,route,sequence,previous_node,task_id,"
                            + "travel_start,travel_end,travel_duration,arrival_time,"
                            + "repair_start,repair_end,repair_duration,completion_time");
            writer.newLine();

            for (ScheduleRow row : rows) {
                writer.write(csv(row.scenario));
                writer.write(','); writer.write(Integer.toString(row.crewRank));
                writer.write(','); writer.write(csv(row.crewId));
                writer.write(','); writer.write(Integer.toString(row.originalRouteIndex));
                writer.write(','); writer.write(Integer.toString(row.firstTask));
                writer.write(','); writer.write(number(row.firstTaskServiceDuration));
                writer.write(','); writer.write(csv(row.route));
                writer.write(','); writer.write(Integer.toString(row.sequence));
                writer.write(','); writer.write(Integer.toString(row.previousNode));
                writer.write(','); writer.write(Integer.toString(row.taskId));
                writer.write(','); writer.write(number(row.travelStart));
                writer.write(','); writer.write(number(row.travelEnd));
                writer.write(','); writer.write(number(row.travelDuration));
                writer.write(','); writer.write(number(row.arrivalTime));
                writer.write(','); writer.write(number(row.repairStart));
                writer.write(','); writer.write(number(row.repairEnd));
                writer.write(','); writer.write(number(row.repairDuration));
                writer.write(','); writer.write(number(row.completionTime));
                writer.newLine();
            }
        }
    }


    /**
     * Prints cumulative task and zone completion statistics at
     * 100, 200, 300, ... and at the exact horizon W.
     *
     * Task completion time is the repair completion time.
     * Zone completion time is the maximum completion time of all tasks
     * associated with that zone.
     */
    private static void printScheduleStatistics(Data data, List<ScheduleRow> rows) {
        if (rows == null || rows.isEmpty()) {
            System.out.println();
            System.out.println("No schedule rows available for statistics.");
            return;
        }

        Map<Integer, Double> taskCompletion = new HashMap<Integer, Double>();
        Map<Integer, Double> crewFinish = new LinkedHashMap<Integer, Double>();
        Map<Integer, Double> crewTravel = new LinkedHashMap<Integer, Double>();
        Map<Integer, Double> crewRepair = new LinkedHashMap<Integer, Double>();

        double totalTravel = 0.0;
        double totalRepair = 0.0;

        for (ScheduleRow row : rows) {
            Double oldTaskCompletion = taskCompletion.get(row.taskId);
            if (oldTaskCompletion == null || row.completionTime > oldTaskCompletion) {
                taskCompletion.put(row.taskId, row.completionTime);
            }

            Double oldCrewFinish = crewFinish.get(row.crewRank);
            if (oldCrewFinish == null || row.completionTime > oldCrewFinish) {
                crewFinish.put(row.crewRank, row.completionTime);
            }

            double travel = crewTravel.containsKey(row.crewRank)
                    ? crewTravel.get(row.crewRank) : 0.0;
            crewTravel.put(row.crewRank, travel + row.travelDuration);

            double repair = crewRepair.containsKey(row.crewRank)
                    ? crewRepair.get(row.crewRank) : 0.0;
            crewRepair.put(row.crewRank, repair + row.repairDuration);

            totalTravel += row.travelDuration;
            totalRepair += row.repairDuration;
        }

        Map<Integer, Double> zoneCompletion =
                computeZoneCompletionTimes(data.getZone2tasks(), taskCompletion);

        List<Double> taskTimes = new ArrayList<Double>(taskCompletion.values());
        List<Double> finiteZoneTimes = new ArrayList<Double>();

        int zonesWithMissingTaskCompletion = 0;
        for (Double value : zoneCompletion.values()) {
            if (value != null && Double.isFinite(value)) {
                finiteZoneTimes.add(value);
            } else {
                zonesWithMissingTaskCompletion++;
            }
        }

        double taskMakespan = maxValue(taskTimes);
        double zoneMakespan = finiteZoneTimes.isEmpty()
                ? Double.NaN : maxValue(finiteZoneTimes);

        double horizon = data.getHorizon();
        if (!(horizon > 0.0) || !Double.isFinite(horizon)) {
            horizon = taskMakespan;
            System.out.println();
            System.out.println(
                    "WARNING: Data.getHorizon() is invalid; using task makespan as W.");
        }

        List<Double> milestones = buildMilestones(horizon);

        System.out.println();
        System.out.println("==========================================================================");
        System.out.println("TASK AND ZONE COMPLETION STATISTICS");
        System.out.println("==========================================================================");
        System.out.println("Time horizon W              : " + number(horizon));
        System.out.println("Tasks in schedule           : " + taskCompletion.size());
        System.out.println("Zones evaluated             : " + zoneCompletion.size());
        System.out.println("Crews in schedule           : " + crewFinish.size());

        System.out.println();
        System.out.printf(
                Locale.ROOT,
                "%-12s %14s %14s %14s %14s%n",
                "Time", "Tasks done", "Task prop.", "Zones done", "Zone prop.");
        System.out.println(
                "--------------------------------------------------------------------------");

        for (double t : milestones) {
            int tasksDone = countCompleted(taskCompletion.values(), t);
            int zonesDone = countCompleted(zoneCompletion.values(), t);

            double taskProportion = taskCompletion.isEmpty()
                    ? Double.NaN
                    : (double) tasksDone / taskCompletion.size();

            double zoneProportion = zoneCompletion.isEmpty()
                    ? Double.NaN
                    : (double) zonesDone / zoneCompletion.size();

            System.out.printf(
                    Locale.ROOT,
                    "%-12.2f %14d %13.2f%% %14d %13.2f%%%n",
                    t,
                    tasksDone,
                    100.0 * taskProportion,
                    zonesDone,
                    100.0 * zoneProportion);
        }

        System.out.println();
        System.out.println("Additional statistics");
        System.out.println("---------------------");
        System.out.println("Task makespan               : " + number(taskMakespan));
        System.out.println("Average task completion     : " + number(mean(taskTimes)));
        System.out.println("Median task completion      : " + number(percentile(taskTimes, 0.50)));
        System.out.println("90th pct. task completion   : " + number(percentile(taskTimes, 0.90)));

        if (!finiteZoneTimes.isEmpty()) {
            System.out.println("Zone makespan               : " + number(zoneMakespan));
            System.out.println("Average zone completion     : " + number(mean(finiteZoneTimes)));
            System.out.println("Median zone completion      : " + number(percentile(finiteZoneTimes, 0.50)));
            System.out.println("90th pct. zone completion   : " + number(percentile(finiteZoneTimes, 0.90)));
        }

        System.out.println("Total crew travel time      : " + number(totalTravel));
        System.out.println("Total crew repair time      : " + number(totalRepair));
        System.out.println("Total active crew time      : " + number(totalTravel + totalRepair));

        double repairShare = totalTravel + totalRepair > 0.0
                ? totalRepair / (totalTravel + totalRepair)
                : Double.NaN;

        System.out.printf(
                Locale.ROOT,
                "Repair share of active time : %.2f%%%n",
                100.0 * repairShare);

        List<Double> crewFinishTimes = new ArrayList<Double>(crewFinish.values());
        System.out.println("Average crew finish time    : " + number(mean(crewFinishTimes)));
        System.out.println("Latest crew finish time     : " + number(maxValue(crewFinishTimes)));

        int tasksDoneAtHorizon = countCompleted(taskCompletion.values(), horizon);
        int zonesDoneAtHorizon = countCompleted(zoneCompletion.values(), horizon);

        System.out.println("Tasks unfinished at W       : "
                + (taskCompletion.size() - tasksDoneAtHorizon));
        System.out.println("Zones unfinished at W       : "
                + (zoneCompletion.size() - zonesDoneAtHorizon));

        if (zonesWithMissingTaskCompletion > 0) {
            System.out.println(
                    "Zones with missing task data : "
                            + zonesWithMissingTaskCompletion
                            + " (counted incomplete at all finite times)");
        }

        printLatestTasks(taskCompletion, 5);
        printLatestZones(zoneCompletion, 5);

        System.out.println();
        System.out.println("Crew summary");
        System.out.println("------------");
        System.out.printf(
                Locale.ROOT,
                "%-10s %14s %14s %14s %14s%n",
                "Crew", "Finish", "Travel", "Repair", "Repair share");

        for (Integer crew : crewFinish.keySet()) {
            double travel = crewTravel.get(crew);
            double repair = crewRepair.get(crew);
            double active = travel + repair;
            double share = active > 0.0 ? repair / active : Double.NaN;

            System.out.printf(
                    Locale.ROOT,
                    "%-10s %14.2f %14.2f %14.2f %13.2f%%%n",
                    "Crew " + crew,
                    crewFinish.get(crew),
                    travel,
                    repair,
                    100.0 * share);
        }

        System.out.println("==========================================================================");
        System.out.println();
    }

    /**
     * Computes each zone's completion time as the latest completion time among
     * all tasks associated with that zone.
     *
     * Supports both direct indexing (zone2tasks[zoneId], index 0 unused)
     * and compact indexing (zone2tasks[zoneId - 1]).
     */
    private static Map<Integer, Double> computeZoneCompletionTimes(
            ArrayList<Integer>[] zone2tasks,
            Map<Integer, Double> taskCompletion) {

        if (zone2tasks == null) {
            throw new IllegalStateException(
                    "Data.getZone2tasks() is null; zone statistics cannot be computed.");
        }

        Map<Integer, Double> result = new LinkedHashMap<Integer, Double>();

        boolean directIndexing =
                zone2tasks.length > 1
                        && (zone2tasks[0] == null || zone2tasks[0].isEmpty());

        int firstIndex = directIndexing ? 1 : 0;

        for (int arrayIndex = firstIndex;
             arrayIndex < zone2tasks.length;
             arrayIndex++) {

            ArrayList<Integer> tasks = zone2tasks[arrayIndex];

            if (tasks == null || tasks.isEmpty()) {
                continue;
            }

            int zoneId = directIndexing ? arrayIndex : arrayIndex + 1;

            double zoneTime = 0.0;
            boolean missingTask = false;

            for (Integer taskId : tasks) {
                if (taskId == null) {
                    continue;
                }

                Double taskTime = taskCompletion.get(taskId);
                if (taskTime == null) {
                    missingTask = true;
                    break;
                }

                zoneTime = Math.max(zoneTime, taskTime);
            }

            result.put(
                    zoneId,
                    missingTask ? Double.POSITIVE_INFINITY : zoneTime);
        }

        return result;
    }

    /** Creates 100, 200, 300, ... and always appends the exact horizon W. */
    private static List<Double> buildMilestones(double horizon) {
        List<Double> milestones = new ArrayList<Double>();

        for (double t = 100.0; t < horizon - 1e-9; t += 100.0) {
            milestones.add(t);
        }

        milestones.add(horizon);
        return milestones;
    }

    private static int countCompleted(Iterable<Double> times, double threshold) {
        int count = 0;

        for (Double time : times) {
            if (time != null
                    && Double.isFinite(time)
                    && time <= threshold + 1e-9) {
                count++;
            }
        }

        return count;
    }

    private static double mean(List<Double> values) {
        if (values == null || values.isEmpty()) {
            return Double.NaN;
        }

        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    private static double maxValue(List<Double> values) {
        if (values == null || values.isEmpty()) {
            return Double.NaN;
        }

        double result = Double.NEGATIVE_INFINITY;
        for (double value : values) {
            result = Math.max(result, value);
        }
        return result;
    }

    private static double percentile(List<Double> values, double p) {
        if (values == null || values.isEmpty()) {
            return Double.NaN;
        }

        List<Double> sorted = new ArrayList<Double>(values);
        Collections.sort(sorted);

        if (sorted.size() == 1) {
            return sorted.get(0);
        }

        double position = p * (sorted.size() - 1);
        int lower = (int) Math.floor(position);
        int upper = (int) Math.ceil(position);

        if (lower == upper) {
            return sorted.get(lower);
        }

        double fraction = position - lower;
        return sorted.get(lower)
                + fraction * (sorted.get(upper) - sorted.get(lower));
    }

    private static void printLatestTasks(
            Map<Integer, Double> completionTimes,
            int limit) {

        List<Map.Entry<Integer, Double>> entries =
                new ArrayList<Map.Entry<Integer, Double>>(completionTimes.entrySet());

        Collections.sort(
                entries,
                new Comparator<Map.Entry<Integer, Double>>() {
                    @Override
                    public int compare(
                            Map.Entry<Integer, Double> a,
                            Map.Entry<Integer, Double> b) {
                        return Double.compare(b.getValue(), a.getValue());
                    }
                });

        System.out.println();
        System.out.println("Latest task completions");
        System.out.println("-----------------------");

        for (int i = 0; i < Math.min(limit, entries.size()); i++) {
            Map.Entry<Integer, Double> entry = entries.get(i);
            System.out.printf(
                    Locale.ROOT,
                    "Task %-6d : %s%n",
                    entry.getKey(),
                    number(entry.getValue()));
        }
    }

    private static void printLatestZones(
            Map<Integer, Double> completionTimes,
            int limit) {

        List<Map.Entry<Integer, Double>> entries =
                new ArrayList<Map.Entry<Integer, Double>>(completionTimes.entrySet());

        Collections.sort(
                entries,
                new Comparator<Map.Entry<Integer, Double>>() {
                    @Override
                    public int compare(
                            Map.Entry<Integer, Double> a,
                            Map.Entry<Integer, Double> b) {
                        return Double.compare(b.getValue(), a.getValue());
                    }
                });

        System.out.println();
        System.out.println("Latest zone completions");
        System.out.println("-----------------------");

        for (int i = 0; i < Math.min(limit, entries.size()); i++) {
            Map.Entry<Integer, Double> entry = entries.get(i);

            String value = Double.isFinite(entry.getValue())
                    ? number(entry.getValue())
                    : "INCOMPLETE";

            System.out.printf(
                    Locale.ROOT,
                    "Zone %-6d : %s%n",
                    entry.getKey(),
                    value);
        }
    }

    private static void validateData(Data data) {
        if (data == null) {
            throw new IllegalArgumentException("Data is null.");
        }
        if (data.getServiceTimeMatrix() == null) {
            throw new IllegalStateException(
                    "Data.getServiceTimeMatrix() is null.");
        }
        if (data.getCrewTravelTimeMatrix() == null) {
            throw new IllegalStateException(
                    "Data.getCrewTravelTimeMatrix() is null.");
        }
    }

    private static void validateTravelMatrix(double[][] matrix) {
        if (matrix == null || matrix.length == 0) {
            throw new IllegalArgumentException("Crew travel-time matrix is empty.");
        }
    }

    private static double travelTime(double[][] matrix, int fromNode, int toNode) {
        if (fromNode < 0 || fromNode >= matrix.length
                || matrix[fromNode] == null
                || toNode < 0 || toNode >= matrix[fromNode].length) {
            throw new IllegalArgumentException(
                    "Crew travel matrix has no entry for " + fromNode + " -> " + toNode
                            + ". Matrix rows=" + matrix.length);
        }
        return matrix[fromNode][toNode];
    }

    private static List<Integer> parseIntegerList(String body) {
        List<Integer> values = new ArrayList<Integer>();
        String[] parts = body.split(",");
        for (String part : parts) {
            String s = part.trim();
            if (!s.isEmpty()) {
                values.add(Integer.parseInt(s));
            }
        }
        return values;
    }

    private static String routeAsText(List<Integer> route) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < route.size(); i++) {
            if (i > 0) {
                sb.append(" -> ");
            }
            sb.append(route.get(i));
        }
        return sb.toString();
    }

    private static String normalizeScenarioName(String s) {
        return s.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]", "");
    }

    private static String safeFileName(String s) {
        String safe = s.trim().replaceAll("[^A-Za-z0-9._-]+", "_");
        return safe.isEmpty() ? "scenario" : safe;
    }

    private static String number(double value) {
        // Locale.ROOT prevents comma decimal separators in CSV numbers.
        return String.format(Locale.ROOT, "%.6f", value);
    }

    private static String csv(String value) {
        if (value == null) {
            return "";
        }
        boolean quote = value.indexOf(',') >= 0
                || value.indexOf('"') >= 0
                || value.indexOf('\n') >= 0
                || value.indexOf('\r') >= 0;
        String escaped = value.replace("\"", "\"\"");
        return quote ? "\"" + escaped + "\"" : escaped;
    }

    /** Parsed crew route as it appeared in the solution file. */
    public static final class CrewRoute {
        private final int originalRouteIndex;
        private final List<Integer> nodes;

        CrewRoute(int originalRouteIndex, List<Integer> nodes) {
            this.originalRouteIndex = originalRouteIndex;
            this.nodes = new ArrayList<Integer>(nodes);
        }

        int firstTask() {
            return nodes.get(1);
        }

        public int getOriginalRouteIndex() {
            return originalRouteIndex;
        }

        public List<Integer> getNodes() {
            return Collections.unmodifiableList(nodes);
        }
    }

    /** One CSV row / one real task visit. */
    private static final class ScheduleRow {
        private final String scenario;
        private final int crewRank;
        private final String crewId;
        private final int originalRouteIndex;
        private final int firstTask;
        private final double firstTaskServiceDuration;
        private final String route;
        private final int sequence;
        private final int previousNode;
        private final int taskId;
        private final double travelStart;
        private final double travelEnd;
        private final double travelDuration;
        private final double arrivalTime;
        private final double repairStart;
        private final double repairEnd;
        private final double repairDuration;
        private final double completionTime;

        ScheduleRow(
                String scenario,
                int crewRank,
                String crewId,
                int originalRouteIndex,
                int firstTask,
                double firstTaskServiceDuration,
                String route,
                int sequence,
                int previousNode,
                int taskId,
                double travelStart,
                double travelEnd,
                double travelDuration,
                double arrivalTime,
                double repairStart,
                double repairEnd,
                double repairDuration,
                double completionTime) {
            this.scenario = scenario;
            this.crewRank = crewRank;
            this.crewId = crewId;
            this.originalRouteIndex = originalRouteIndex;
            this.firstTask = firstTask;
            this.firstTaskServiceDuration = firstTaskServiceDuration;
            this.route = route;
            this.sequence = sequence;
            this.previousNode = previousNode;
            this.taskId = taskId;
            this.travelStart = travelStart;
            this.travelEnd = travelEnd;
            this.travelDuration = travelDuration;
            this.arrivalTime = arrivalTime;
            this.repairStart = repairStart;
            this.repairEnd = repairEnd;
            this.repairDuration = repairDuration;
            this.completionTime = completionTime;
        }
    }

    /**
     * Supports the two common service-array layouts:
     *   A) direct node indexing: service[taskId], usually with service[0] for depot;
     *   B) compact task indexing: service[taskId - 1].
     * The choice is made once from the largest real task ID in the routes.
     */
    private static final class ServiceTimeAccessor {
        private final double[] service;
        private final boolean directNodeIndexing;

        private ServiceTimeAccessor(double[] service, boolean directNodeIndexing) {
            this.service = service;
            this.directNodeIndexing = directNodeIndexing;
        }

        static ServiceTimeAccessor forRoutes(double[] service, List<CrewRoute> routes) {
            int maxTaskId = 0;
            for (CrewRoute route : routes) {
                // Exclude start depot and final dummy sink.
                for (int i = 1; i < route.nodes.size() - 1; i++) {
                    maxTaskId = Math.max(maxTaskId, route.nodes.get(i));
                }
            }

            if (service.length > maxTaskId) {
                return new ServiceTimeAccessor(service, true);
            }
            if (service.length >= maxTaskId) {
                return new ServiceTimeAccessor(service, false);
            }
            throw new IllegalArgumentException(
                    "Service-time array is too short for task ID " + maxTaskId
                            + ". Length=" + service.length);
        }

        double get(int taskId) {
            int index = directNodeIndexing ? taskId : taskId - 1;
            if (index < 0 || index >= service.length) {
                throw new IllegalArgumentException(
                        "No service time for task " + taskId + " (index " + index + ")");
            }
            return service[index];
        }
    }

    /**
     * Optional command-line shell. It assumes some application code has already
     * populated Data.getInstance() in this JVM. In most projects it is cleaner
     * to call export(...) directly immediately after reading the instance.
     *
     * args: <experimentRoot> <scenario> [outputCsv]
     */
    public static void main(String[] args) throws Exception {
        if (args.length < 2 || args.length > 3) {
            System.err.println(
                    "Usage: CrewScheduleCsvExporter <experimentRoot> <scenario> [outputCsv]");
            System.err.println(
                    "Important: Data.getInstance() must already have been populated in this JVM.");
            System.exit(2);
        }

        Data data = Data.getInstance();
        Path root = Paths.get(args[0]);
        String scenario = args[1];

        if (args.length == 3) {
            export(data, root, scenario, Paths.get(args[2]));
        } else {
            export(data, root, scenario);
        }
    }
}