package problem.analyses;

import ilog.concert.IloException;
import problem.graph.Data;
import problem.milp.MILPCplex;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Collects only objective-value statistics for saved optimal solutions.
 *
 * Data.getInstance() must already represent the instance/tower configuration
 * identified by (filename, towerNumber), exactly as for the existing optimal
 * statistics workflow.
 */
public final class OptimalObjectiveStatistics {

    private static final Path OPTIMAL_SUMMARY =
            Paths.get("isOptimal", "opt_summary.csv");

    private static final Path CREW_ONLY_SOLUTIONS =
            Paths.get("summary", "crewOnly", "solutions");

    private static final Path MULTITHREAD_SOLUTIONS =
            Paths.get("summary", "multithread", "solutions");

    private static final Path OUTPUT_CSV =
            Paths.get(
                    "optimalObjectiveStatistics",
                    "objValStatistics.csv");

    private static final int HORIZON = 600;
    private static final int INTERVAL = 100;

    private static final String TECHNICIAN_SECTION = "Technician Plans";
    private static final String TOWER_SECTION = "Tower Routes";

    private static final Pattern ROUTE_PATTERN =
            Pattern.compile("^\\s*\\[(.*)]\\s*$");

    private OptimalObjectiveStatistics() {
    }

    /**
     * If (filename, towerNumber) is marked optimal, locate its saved solution,
     * read both technician and tower routes, replay those routes in MILPCplex,
     * and append objective values at 0,100,...,600 to
     * optimalObjectiveStatistics/objValStatistics.csv.
     *
     * @return true when an optimal row was collected; false when the pair is
     *         marked non-optimal in isOptimal/opt_summary.csv.
     */
    public static boolean collect(String filename, int towerNumber)
            throws IOException, IloException {

        if (filename == null || filename.trim().isEmpty()) {
            throw new IllegalArgumentException("filename must not be blank.");
        }
        if (towerNumber < 0) {
            throw new IllegalArgumentException("towerNumber must be >= 0.");
        }

        boolean isOptimal = readOptimality(filename, towerNumber);
        if (!isOptimal) {
            return false;
        }

        Data data = Data.getInstance();
        if (data == null) {
            throw new IllegalStateException("Data.getInstance() returned null.");
        }
        if (Math.abs(data.getHorizon() - HORIZON) > 1e-9) {
            throw new IllegalStateException(
                    "Expected planning horizon " + HORIZON
                            + ", but Data horizon is " + data.getHorizon() + ".");
        }
        if (data.getTowerNumber() != towerNumber) {
            throw new IllegalStateException(
                    "Data tower number (" + data.getTowerNumber()
                            + ") does not match requested towerNumber ("
                            + towerNumber + ").");
        }

        Path solutionDirectory = towerNumber == 0
                ? CREW_ONLY_SOLUTIONS
                : MULTITHREAD_SOLUTIONS;

        Path solutionFile =
                findSolutionFile(solutionDirectory, filename, towerNumber);

        SolutionRoutes routes = readSolutionRoutes(solutionFile);

        if (routes.technicianPlans.isEmpty()) {
            throw new IllegalArgumentException(
                    "No technician plans found in solution file: " + solutionFile);
        }

        if (routes.towerRoutes.size() != towerNumber) {
            throw new IllegalArgumentException(
                    "Solution file " + solutionFile
                            + " contains " + routes.towerRoutes.size()
                            + " tower routes, but towerNumber=" + towerNumber + ".");
        }

        MILPCplex milp = new MILPCplex();
        double[] objectiveValues = milp.buildAndSolve2index(
                routes.technicianPlans,
                routes.towerRoutes);

        int expectedValues = HORIZON / INTERVAL + 1;
        if (objectiveValues.length != expectedValues) {
            throw new IllegalStateException(
                    "Expected " + expectedValues
                            + " objective statistics, but MILPCplex returned "
                            + objectiveValues.length + ".");
        }

        appendStatistics(
                filename,
                towerNumber,
                true,
                objectiveValues);

        return true;
    }

    // =====================================================================
    // Optimality summary
    // =====================================================================

    private static boolean readOptimality(
            String filename,
            int towerNumber) throws IOException {

        if (!Files.isRegularFile(OPTIMAL_SUMMARY)) {
            throw new IllegalArgumentException(
                    "Optimality summary does not exist: "
                            + OPTIMAL_SUMMARY.toAbsolutePath());
        }

        try (BufferedReader reader = Files.newBufferedReader(
                OPTIMAL_SUMMARY,
                StandardCharsets.UTF_8)) {

            String headerLine = reader.readLine();
            if (headerLine == null) {
                throw new IllegalArgumentException(
                        "Optimality summary is empty: "
                                + OPTIMAL_SUMMARY.toAbsolutePath());
            }

            List<String> header = parseCsvLine(headerLine);
            int filenameIndex = findColumn(header, "filename");
            int towerIndex = findColumn(header, "towerNumber");
            int optimalIndex = findColumn(header, "isOptimal");

            Boolean result = null;
            String line;
            int lineNumber = 1;

            while ((line = reader.readLine()) != null) {
                lineNumber++;
                if (line.trim().isEmpty()) {
                    continue;
                }

                List<String> fields = parseCsvLine(line);
                int requiredSize = Math.max(
                        filenameIndex,
                        Math.max(towerIndex, optimalIndex)) + 1;

                if (fields.size() < requiredSize) {
                    throw new IllegalArgumentException(
                            "Malformed row " + lineNumber
                                    + " in " + OPTIMAL_SUMMARY + ": " + line);
                }

                if (!fields.get(filenameIndex).trim().equals(filename)) {
                    continue;
                }

                int rowTowerNumber;
                try {
                    rowTowerNumber = Integer.parseInt(
                            fields.get(towerIndex).trim());
                } catch (NumberFormatException e) {
                    throw new IllegalArgumentException(
                            "Invalid towerNumber at row " + lineNumber
                                    + ": " + fields.get(towerIndex),
                            e);
                }

                if (rowTowerNumber != towerNumber) {
                    continue;
                }

                String value = fields.get(optimalIndex).trim();
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
                                + filename + "', towerNumber=" + towerNumber
                                + " in " + OPTIMAL_SUMMARY.toAbsolutePath());
            }

            return result;
        }
    }

    private static int findColumn(List<String> header, String wanted) {
        for (int i = 0; i < header.size(); i++) {
            if (header.get(i).trim().equalsIgnoreCase(wanted)) {
                return i;
            }
        }
        throw new IllegalArgumentException("CSV column not found: " + wanted);
    }

    // =====================================================================
    // Solution file location and parsing
    // =====================================================================

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

        Pattern solutionPattern = Pattern.compile(
                "^" + Pattern.quote(baseName)
                        + "-(\\d+)-" + towerNumber + "\\.txt$");

        List<Path> matches = new ArrayList<>();
        try (Stream<Path> paths = Files.walk(solutionDirectory)) {
            paths.filter(Files::isRegularFile)
                    .filter(path -> solutionPattern
                            .matcher(path.getFileName().toString())
                            .matches())
                    .forEach(matches::add);
        }

        if (matches.isEmpty()) {
            throw new IllegalArgumentException(
                    "Could not find solution file matching '"
                            + baseName + "-<crewCount>-" + towerNumber
                            + ".txt' below "
                            + solutionDirectory.toAbsolutePath());
        }

        if (matches.size() > 1) {
            throw new IllegalArgumentException(
                    "More than one solution file matched instance '"
                            + filename + "' with towerNumber=" + towerNumber
                            + ": " + matches);
        }

        return matches.get(0);
    }

    private static SolutionRoutes readSolutionRoutes(Path solutionFile)
            throws IOException {

        List<List<Integer>> technicianPlans = new ArrayList<>();
        List<List<Integer>> towerRoutes = new ArrayList<>();

        Section section = Section.NONE;
        boolean sawTechnicianHeading = false;
        boolean sawTowerHeading = false;

        try (BufferedReader reader = Files.newBufferedReader(
                solutionFile,
                StandardCharsets.UTF_8)) {

            String line;
            while ((line = reader.readLine()) != null) {
                String trimmed = line.trim();

                if (isHeading(trimmed, TECHNICIAN_SECTION)) {
                    section = Section.TECHNICIANS;
                    sawTechnicianHeading = true;
                    continue;
                }

                if (isHeading(trimmed, TOWER_SECTION)) {
                    section = Section.TOWERS;
                    sawTowerHeading = true;
                    continue;
                }

                if (trimmed.isEmpty() || section == Section.NONE) {
                    continue;
                }

                Matcher matcher = ROUTE_PATTERN.matcher(trimmed);
                if (!matcher.matches()) {
                    continue;
                }

                List<Integer> route = parseIntegerList(matcher.group(1));
                if (route.size() < 3) {
                    throw new IllegalArgumentException(
                            "Route must contain depot, at least one visited node, "
                                    + "and terminal node: " + line);
                }
                if (route.get(0) != 0) {
                    throw new IllegalArgumentException(
                            "Route does not begin at depot 0: " + line);
                }

                if (section == Section.TECHNICIANS) {
                    technicianPlans.add(route);
                } else {
                    towerRoutes.add(route);
                }
            }
        }

        if (!sawTechnicianHeading) {
            throw new IllegalArgumentException(
                    "Missing '" + TECHNICIAN_SECTION
                            + "' section in " + solutionFile);
        }
        if (!sawTowerHeading) {
            throw new IllegalArgumentException(
                    "Missing '" + TOWER_SECTION
                            + "' section in " + solutionFile);
        }

        return new SolutionRoutes(technicianPlans, towerRoutes);
    }

    private static boolean isHeading(String line, String heading) {
        return line.equalsIgnoreCase(heading)
                || line.equalsIgnoreCase(heading + ":");
    }

    private enum Section {
        NONE,
        TECHNICIANS,
        TOWERS
    }

    private static final class SolutionRoutes {
        private final List<List<Integer>> technicianPlans;
        private final List<List<Integer>> towerRoutes;

        private SolutionRoutes(
                List<List<Integer>> technicianPlans,
                List<List<Integer>> towerRoutes) {
            this.technicianPlans = technicianPlans;
            this.towerRoutes = towerRoutes;
        }
    }

    // =====================================================================
    // Output CSV
    // =====================================================================

    private static void appendStatistics(
            String filename,
            int towerNumber,
            boolean isOptimal,
            double[] objectiveValues) throws IOException {

        Path parent = OUTPUT_CSV.toAbsolutePath().getParent();
        if (parent != null) {
            Files.createDirectories(parent);
        }

        boolean writeHeader =
                !Files.exists(OUTPUT_CSV) || Files.size(OUTPUT_CSV) == 0;

        try (BufferedWriter writer = Files.newBufferedWriter(
                OUTPUT_CSV,
                StandardCharsets.UTF_8,
                StandardOpenOption.CREATE,
                StandardOpenOption.APPEND)) {

            if (writeHeader) {
                writeHeader(writer);
            }

            StringBuilder row = new StringBuilder();
            row.append(csv(filename))
                    .append(',')
                    .append(towerNumber)
                    .append(',')
                    .append(isOptimal ? "TRUE" : "FALSE");

            for (double value : objectiveValues) {
                row.append(',')
                        .append(String.format(Locale.ROOT, "%.6f", value));
            }

            writer.write(row.toString());
            writer.newLine();
        }
    }

    private static void writeHeader(BufferedWriter writer) throws IOException {
        StringBuilder header = new StringBuilder(
                "filename,towerNumber,isOptimal");

        for (int t = 0; t <= HORIZON; t += INTERVAL) {
            header.append(',').append(t);
        }

        writer.write(header.toString());
        writer.newLine();
    }

    // =====================================================================
    // Parsing helpers
    // =====================================================================

    private static List<Integer> parseIntegerList(String body) {
        List<Integer> values = new ArrayList<>();
        String[] parts = body.split(",");

        for (String part : parts) {
            String value = part.trim();
            if (!value.isEmpty()) {
                values.add(Integer.parseInt(value));
            }
        }

        return values;
    }

    private static List<String> parseCsvLine(String line) {
        List<String> fields = new ArrayList<>();
        StringBuilder current = new StringBuilder();
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

        boolean quote = value.indexOf(',') >= 0
                || value.indexOf('"') >= 0
                || value.indexOf('\n') >= 0
                || value.indexOf('\r') >= 0;

        String escaped = value.replace("\"", "\"\"");
        return quote ? "\"" + escaped + "\"" : escaped;
    }
}
