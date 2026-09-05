package lib;

import problem.Constants;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.util.Scanner;
import java.util.stream.Collectors;

// utility class
public class Utility {
    public static double bPCTiLim;
    public static int algo;
    public static ArrayList<Integer> towerNumbers;

    /**
     * Default configuration file used by the properties-based implementation.
     * The path is relative to the application's working directory, which should
     * normally be the project root.
     */
    public static final Path DEFAULT_CONFIG_FILE = Paths.get("config", "run.properties");

    /**
     * Legacy reader kept for backward compatibility.
     */
    public Utility(String filename) {
        try (BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {
            //------------------------------------------------------
            bPCTiLim = Double.parseDouble(bufferedReader.readLine().trim());

            String line = bufferedReader.readLine();

            ArrayList<Integer> list = Arrays.stream(line.split(","))
                    .map(String::trim)
                    .map(Integer::parseInt)
                    .collect(Collectors.toCollection(ArrayList::new));
            towerNumbers = list;

            algo = Integer.parseInt(bufferedReader.readLine().trim());

            int BUDGET = Integer.parseInt(bufferedReader.readLine().trim());
            double ALPHA = Double.parseDouble(bufferedReader.readLine().trim());

            applyAlgorithmConfiguration(BUDGET, ALPHA);
        } catch (IOException | NumberFormatException e) {
            e.printStackTrace();
            System.exit(-1);
        }
    }

    /**
     * Loads run parameters from config/run.properties.
     */
    public static void loadFromProperties() {
        loadFromProperties(DEFAULT_CONFIG_FILE);
    }

    /**
     * Loads run parameters from a specified Java .properties file.
     *
     * @param configFile path to the properties file
     */
    public static void loadFromProperties(Path configFile) {
        Properties properties = new Properties();

        try (BufferedReader reader = Files.newBufferedReader(configFile, StandardCharsets.UTF_8)) {
            properties.load(reader);

            bPCTiLim = getRequiredDouble(properties, "algorithm.timeLimitSeconds");
            towerNumbers = parseIntegerList(
                    getRequiredProperty(properties, "experiment.towerNumbers"),
                    "experiment.towerNumbers"
            );
            algo = getRequiredInt(properties, "algorithm.code");

            int BUDGET = getRequiredInt(properties, "robust.uncertaintyBudget");
            double ALPHA = getRequiredDouble(properties, "robust.alpha");

            applyAlgorithmConfiguration(BUDGET, ALPHA);
        } catch (IOException e) {
            throw new IllegalStateException(
                    "Unable to read configuration file: " + configFile.toAbsolutePath(), e
            );
        }
    }

    private static void applyAlgorithmConfiguration(int BUDGET, double ALPHA) {
        if(algo==9810){
            algo = 29;
            Constants.UPPER_BOUNDING_HEURISTIC = true;
        } else if (algo==7810) {
            algo = 29;
            Constants.UPPER_BOUNDING_HEURISTIC = true;
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
        }else if(algo==99){
            algo = 4;
            Constants.DETERMINE_MINIMUM_NUMBER_CREWS = true;
        } else if (algo==122){
            Constants.U_ICEA = true;
            algo = 29;
        } else if(algo==199){
            algo = 4;
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
            Constants.DETERMINE_MINIMUM_NUMBER_CREWS = true;
        }else if(algo==265){
            algo = 2;
            Constants.FIX_CREW_AND_TOWER_ROUTES = true;
            Constants.SOLUTION_ANALYSIS = true;
        } else if(algo==266){
            //do nothing
        } else if(algo==267){
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
        }
        else if(algo==98){
            algo = 29;
            Constants.FIX_CREW_ROUTES = true;
        }
        else if(algo==97){
            algo = 29;
            Constants.SOLVE_FOR_CREWS = true;
            Constants.WRITE_FOR_CREWS = true;
        }
        else if(algo==96){
            algo = 29;
            Constants.NO_REPOSITIONING = true;
        } else if (algo==89) {
            algo = 2;
            Constants.FIX_CREW_ROUTES = true;
        } else if(algo==88){
            algo = 2;
            Constants.FIX_CREW_AND_TOWER_ROUTES = true;
        } else if(algo==79){
            algo = 29;
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
        } else if(algo==71){
            algo = 29;
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
            towerNumbers = new ArrayList<>(List.of(0));
            Constants.SOLVE_FOR_CREWS = true;
        } else if(algo==78){
            algo = 29;
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
            Constants.FIX_CREW_ROUTES = true;
        } else if(algo==76){
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
            algo = 29;
            Constants.NO_REPOSITIONING = true;
        }
        else if(algo==70){
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
            algo = 2;
        }
        else if(algo==73){
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
            algo = 29;
            Constants.FIX_CREW_ROUTES = true;
            Constants.COST_OF_PRIORITY = true;
            Constants.COST_OF_PRIORITY_CREW = true;
            Constants.COST_OF_PRIORITY_TOWER = true;
        } else if(algo==74){
            Constants.ROBUST = true;
            Constants.BUDGET = BUDGET;
            Constants.ALPHA = ALPHA;
            algo = 29;
            Constants.COST_OF_PRIORITY = true;
            Constants.COST_OF_PRIORITY_CREW = true;
            Constants.COST_OF_PRIORITY_TOWER = false;
            towerNumbers = new ArrayList<>(List.of(0));
            Constants.SOLVE_FOR_CREWS = true;
        } else if(algo==69){
            algo = 29;
            Constants.BRANCH_AND_PRICE = true;
        } else if(algo==42){
            algo = 29;
            Constants.COST_OF_PRIORITY = true;
            Constants.COST_OF_PRIORITY_CREW = true;
            Constants.COST_OF_PRIORITY_TOWER = false;
        } else if(algo==41){
            algo = 29;
            Constants.COST_OF_PRIORITY = true;
            Constants.COST_OF_PRIORITY_CREW = true;
            Constants.COST_OF_PRIORITY_TOWER = true;
        } else if(algo==43){
            algo = 29;
            Constants.FIX_CREW_ROUTES = true;
            Constants.COST_OF_PRIORITY = true;
            Constants.COST_OF_PRIORITY_CREW = true;
            Constants.COST_OF_PRIORITY_TOWER = true;
        } else if(algo==44){
            algo = 29;
            Constants.COST_OF_PRIORITY = true;
            Constants.COST_OF_PRIORITY_CREW = true;
            Constants.COST_OF_PRIORITY_TOWER = false;
            towerNumbers = new ArrayList<>(List.of(0));
            Constants.SOLVE_FOR_CREWS = true;
        }
    }

    private static String getRequiredProperty(Properties properties, String key) {
        String value = properties.getProperty(key);

        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "Missing required property '" + key + "'."
            );
        }

        return value.trim();
    }

    private static int getRequiredInt(Properties properties, String key) {
        String value = getRequiredProperty(properties, key);

        try {
            return Integer.parseInt(value);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(
                    "Property '" + key + "' must be an integer, but was '" + value + "'.", e
            );
        }
    }

    private static double getRequiredDouble(Properties properties, String key) {
        String value = getRequiredProperty(properties, key);

        try {
            return Double.parseDouble(value);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(
                    "Property '" + key + "' must be numeric, but was '" + value + "'.", e
            );
        }
    }

    private static ArrayList<Integer> parseIntegerList(String value, String key) {
        try {
            return Arrays.stream(value.split(","))
                    .map(String::trim)
                    .filter(item -> !item.isEmpty())
                    .map(Integer::parseInt)
                    .collect(Collectors.toCollection(ArrayList::new));
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(
                    "Property '" + key + "' must be a comma-separated list of integers, "
                            + "but was '" + value + "'.", e
            );
        }
    }

    private String readLine(BufferedReader bufferedReader) throws IOException {
        Scanner scanner = new Scanner(bufferedReader.readLine());
        scanner.useDelimiter("\\s+");
        scanner.next();
        return scanner.next();
    }

    private boolean parseBoolean(BufferedReader bufferedReader) throws IOException {
        return Integer.parseInt(readLine(bufferedReader)) == 1;
    }
}
