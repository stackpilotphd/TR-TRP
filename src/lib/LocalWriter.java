package lib;

import problem.Constants;
import problem.graph.Data;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

public class LocalWriter {

    public static String filename;
    public static double root1LB;
    public static double root1CPU;
    public static double rootCutLB;
    public static double rootCutCPU;
    public static int capacityLinkingCutsRoot;
    public static int minVehCutsRoot;
    public static int capacityCutRoot;
    public static int subsetRowTreeRoot;
    public static int subsetRowRouteRoot;

    public static double upperBoundAlgoValue;
    public static double upperBoundAlgoCPU;

    public static double problemFRvalue;
    public static int problemFRCalls;
    public static double problemFRCPU;

    public static int sepMWItotCuts;
    public static double sepMWItotCPU;

    public static int capacityCutTotal;
    public static int subsetRowTreeTotal;
    public static int subsetRowRouteTotal;
    public static int capacityLinkingCutsTotal;
    public static int minVehCutsTotal;
    public static int repairCutsTotal;
    public static int totalNodesExplored;
    public static double totalCPU;

    public static double bestObjectiveValue;
    public static boolean isOptimal;
    public static double totalCTC;
    public static double totalPenaltyTree;
    public static double totalPenaltyRoute;
    public static double totalCrewCost;
    public static double totalTowerPrize;
    public static double weightLowerBound;

    public static int fleet;
    public static int managerialAlgo;
    public static String algorithm;

    public static int customerPerCluster;
    public static int clusterNumber;

    public static double bestLBatTermination;
    public static double gap;
    public static int remainingNodes;

    public static int totalRouteColumns;
    public static int totalTreeColumns;
    public static double routeEnumerationCPU;
    public static double treeEnumerationCPU;
    public static int repairIterations;

    public static String sharedString;
    public static StringBuilder gbl_string;

    public static String uncertainty;
    public static double strongBranchingCPU;
    public static int strongBranchingSubproblemsSolved;
    public static double bendersBestValue;
    public static double bendersCPU;
    public static int bestHeuristicIteration;
    public static int heuristicMIPDepthFoundBestSolution;

    public static void initialize() {
        switch (Utility.algo){
            case 1 -> algorithm = "BPC";
            case 2 -> algorithm = "MIP";
            case 4 -> algorithm = "ICEA";
            case 6 -> algorithm = "Heuristic";
            default -> algorithm = "N/A";
        }


        if(Constants.ROBUST){
            if(Constants.R_TOGGLE == 0){
                uncertainty = "service";
            } else if(Constants.R_TOGGLE == 1)
                uncertainty = "travel";
        }

        if(headers == null) {
            headers = "Parameters" + "," + " " + "," + " " + "," +
                    " " + ","  +
                    " " + "," + " " + "," +
                    " " + "," +
                    " " + "," + " " + "," +
                    "Total" + "," + " " + "," +" " + "," + " " + "," +
                    "MIP-Heuristic" +"," + " " + ","
                    + "Strong Branching" + "," + " " + ","
                    + "Ben-Heuristic" + "," + " " + ","+ " " + ","+
                    "ICEA#1" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#2" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#3" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#4" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#5" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#6" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#7" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#8" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
                    "ICEA#9" +  "," + " " + "," +" " + "," + " " + "," + " " + "," +
//                    "Algorityhm" + "," + "Equity Algo" + "," +
                    System.lineSeparator() +
                    "filename" + "," + "Crew#" + "," + "Tower#" + "," +
                    "algorithm" + ","  +
                    "BUDGET" + "," + "Alpha" + "," +
                    "LowerBoundLifting" + "," + "NODE ELIMINATION" + "," + "MIP HEURISTIC" + "," +
                    "BestVal" + "," + "LB at termination" + "," +"Node#" + "," + "CPU" + "," +
                    "CPU" +"," + "Depth Last Updated Best Value" + ","
                    + "time" + "," + "SubproblemsSolved" + ","
                    + "BH-BestValue" + "," + "time" + ","+ "iter" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
                    "RootVal" + "," + "RootCPU" + "," +"Node#" + "," + "CPU" + "," + "BKS" + "," +
//                    "MIP Heuristic Timer" + "," + "StrongBranchingCPU" +  "," + "StringBranchingSubproblems" + "," +
                    System.lineSeparator();

//            headers = "filename" + "," + "Crew#" + "," + "Tower#" + "," +
//                    "Zone#" + "," + "Task#" + "," +
//                    "RootNode LB" + "," + "RootNode CPU" + "," +
//                    "Node#" + "," + "CPU" + "," +
//                    "BestVal" + "," + "isOptimal" + "," +
////                    "totalCrewCost" + "," + "totalTowerPrize" + "," +
//                    "algorithm" + "," + "LB at termination" + "," + "remainingNodes" + "," +
//                    "ROBUST" + "," + "BUDGET" + "," + "Alpha" + "," + "uncertainty" + "," +
//                    "CREW_WORK_LENGTH (h)" + "," + "isPulse" + "," +
//                    System.lineSeparator();
        }

        gbl_string = new StringBuilder();
        gbl_string.append(headers);
    }

    public static void restart() {
        heuristicMIPDepthFoundBestSolution = 0;
        bestHeuristicIteration = 0;
        iceaStrings = new ArrayList<>();
        filename = "";
        root1LB = 0.0;
        root1CPU = 0.0;
        rootCutLB = 0.0;
        rootCutCPU = 0.0;
        capacityLinkingCutsRoot = 0;
        minVehCutsRoot = 0;
        capacityCutRoot = 0;
        subsetRowTreeRoot = 0;
        subsetRowRouteRoot = 0;

        upperBoundAlgoValue = 0.0;
        upperBoundAlgoCPU = 0.0;

        problemFRvalue = 0.0;
        problemFRCalls = 0;
        problemFRCPU = 0.0;

        sepMWItotCuts = 0;
        sepMWItotCPU = 0.0;

        capacityCutTotal = 0;
        subsetRowTreeTotal = 0;
        subsetRowRouteTotal = 0;
        capacityLinkingCutsTotal = 0;
        minVehCutsTotal = 0;
        repairCutsTotal = 0;
        totalNodesExplored = 0;
        totalCPU = 0.0;

        bestObjectiveValue = 0;
        isOptimal = true;
        fleet = 0;

        switch (Utility.algo){
            case 1 -> algorithm = "BPC";
            case 2 -> algorithm = "MIP";
            case 3 -> algorithm = "IECA2";
            case 4 -> algorithm = "ICEA";
            case 6 -> algorithm = "Heuristic";
            default -> algorithm = "N/A";
        }

        totalCTC = 0;
        totalTowerPrize = 0;
        totalPenaltyRoute = 0;
        totalPenaltyTree = 0;
        totalCrewCost = 0;
        weightLowerBound = 0;

        sharedString ="";

        customerPerCluster = 0;
        clusterNumber = 0;
        bestLBatTermination = 0.;
        gap = 0.;
        remainingNodes = 0;

        totalRouteColumns = 0;
        totalTreeColumns = 0;
        routeEnumerationCPU = 0;
        treeEnumerationCPU = 0;
        repairIterations = 0;

        strongBranchingCPU = 0;
        strongBranchingSubproblemsSolved = 0;
        StaticSharedValues.mipHeuristicTimer = 0;
        StaticSharedValues.strongBranchingSubproblemsSolved = 0;
        StaticSharedValues.strongBranchingCPU = 0;
        bendersCPU = 0;
        bendersBestValue = 0;

    }

    public static String headers;
    public static ArrayList<String> iceaStrings;
    // Method to write the field values into a CSV file
    public static void writeValuesToCSV(String filename) {
        if(Utility.algo==29)
            return;//MULTI THREADED NOT SUPPORTED
        // Get the current date and time in the desired format
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyMMdd.HHmmss");
        String formattedDateTime = LocalDateTime.now().format(formatter);
        double epsilon = StaticSharedValues.budget > 0 ? Constants.ALPHA : 0;
        // Create the full file path with the given filename and current date-time
        Path filePath = Paths.get("computational", filename + "_" + formattedDateTime + ".txt");
        Data data = Data.getInstance();
        {
            sharedString = filename +","+ data.getCrewNumber() + "," + data.getTowerNumber() + "," +
                    algorithm + "," + (StaticSharedValues.budget-1)  + "," + epsilon  + "," +
                    Constants.SERVICE_BOUND_CUT  + "," + Constants.USE_NODE_ELIMINATION_CREW_PULSE  + "," +
                    Constants.SHOULD_RUN_MIP_HEURISTIC  + "," +
                    bestObjectiveValue + "," + bestLBatTermination + "," + totalNodesExplored + "," + totalCPU;
        }


        sharedString = sharedString + "," + StaticSharedValues.mipHeuristicTimer*1e-9 +"," + heuristicMIPDepthFoundBestSolution + ","
                + strongBranchingCPU*1e-9 + "," + strongBranchingSubproblemsSolved + ","
                + bendersBestValue + "," + bendersCPU + "," + bestHeuristicIteration;

        for(String iceaString : iceaStrings) {
            sharedString = sharedString + "," + iceaString;
        }

        sharedString = sharedString  + System.lineSeparator();
        gbl_string.append(sharedString);
        try (FileWriter writer = new FileWriter(filePath.toFile())) {
            writer.write(sharedString);
            System.out.println("CSV file created successfully: " + filePath);
        } catch (IOException e) {
            System.err.println("An error occurred while writing to the file: " + e.getMessage());
        }
    }

    public static void writeGlobalString(String filename) {
        if(Utility.algo==29)
            return;//MULTI THREADED NOT SUPPORTED
        // Get the current date and time in the desired format
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyMMdd.HHmmss");
        String formattedDateTime = LocalDateTime.now().format(formatter);

        String outputFile;
        {
            String baseName = Paths.get(filename)
                    .getFileName()
                    .toString();

            if (baseName.endsWith(".txt")) {
                baseName = baseName.substring(0, baseName.length() - 4);
            }

            outputFile = baseName;
        }

        String out_path;
        switch (Utility.algo) {
            case 1 -> out_path = "bpc";
            case 2 -> out_path = "solver";
            case 4 -> out_path = "icea";
            case 22 -> out_path = "crewFirstTowerSecond";
            default -> out_path = "N/A";
        }
        if(out_path.equals("solver"))
            return;
        if(out_path.equals("N/A"))
            return;
        if(Constants.ROBUST)
            out_path = "robust";
        // Create the full file path with the given filename and current date-time
        Path filePath = Paths.get("summary/" + out_path + "/", outputFile + "_" + formattedDateTime + ".csv");
        // Create the parent folder(s) if they do not already exist
        try {
            Files.createDirectories(filePath.getParent());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try (FileWriter writer = new FileWriter(filePath.toFile())) {
            writer.write(gbl_string.toString());
            System.out.println("Global CSV file created successfully: " + filePath);
        } catch (IOException e) {
            System.err.println("An error occurred while writing to the file: " + e.getMessage());
        }


    }
}
