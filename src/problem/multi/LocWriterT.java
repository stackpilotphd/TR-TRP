package problem.multi;

import lib.StaticSharedValues;
import problem.BP.Route;
import problem.Constants;
import problem.graph.Data;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

public class LocWriterT {

    private StringBuilder gbl_string;
    private String filename;
    public LocWriterT() {
        head();
    }



    private boolean timeOut;



    public void setFilename(String filename) {
        this.filename = filename;
    }

    private void head(){
        String headers = "Parameters" + "," + " " + "," + " " + "," +
                " " + ","  +
                " " + "," + " " + "," +
//                " " + "," +
//                " " + "," + " " + "," +
//                "" + "," + " " + "," +
                "Total" + "," + " " + "," + " " + "," + " " + "," +
                "T#1"+ "," + " " +  "," + " " + "," +" " + "," + " " + "," +
                "T#2"+ "," + " " +  "," + " " + "," +" " + "," + " " + "," +
                "T#3"+ "," + " " +  "," + " " + "," +" " + "," + " " + "," +
                "T#4"+ "," + " " +  "," + " " + "," +" " + "," + " " + "," +
                "T#5"+ "," + " " +  "," + " " + "," +" " + "," + " " + "," +
                System.lineSeparator() +
                "filename" + "," + "Crew#" + "," + "Tower#" + "," +
                "algorithm" + ","  +
                "BUDGET" + "," + "Alpha" + "," +
//                "LowerBoundLifting" + "," + "NODE ELIMINATION" + "," + "STRONG BRANCHING" + "," +
//                "BRANCH_ON_TOWER_SIZE" + ","  + "PARALLEL_BRANCH_AND_BOUND" + "," +
                "BestVal" + ","  + "CPU" + "," + "TimeOut" + "," + "Zones" + "," +
                "RootLB"+ ","+"BKS" + "," +"LB_Term" + ","  +"Node#" + "," + "CPU" + "," +
                "RootLB"+ ","+"BKS" + ","  +"LB_Term" + ","  +"Node#" + "," + "CPU" + "," +
                "RootLB"+ ","+"BKS" + ","  +"LB_Term" + ","  +"Node#" + "," + "CPU" + "," +
                "RootLB"+ ","+"BKS" + ","  +"LB_Term" + ","  +"Node#" + "," + "CPU" + "," +
                "RootLB"+ ","+"BKS" + ","  +"LB_Term" + ","  +"Node#" + "," + "CPU" + "," +
                "GblValidLB"+ ","+"OptGap" + ","+
                System.lineSeparator();
        gbl_string = new StringBuilder();
        gbl_string.append(headers);
    }

    public static int numberOfZonesVisitedByTowers;
    public void writeIntermediateResults(double bestObjectiveValue
            , double totalCPU, int numZonesAssigned, ArrayList<OptimizationResult> results) {
        // Get the current date and time in the desired format
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyMMdd.HHmmss");
        String formattedDateTime = LocalDateTime.now().format(formatter);
        double epsilon = StaticSharedValues.budget > 0 ? Constants.ALPHA : 0;
        // Create the full file path with the given filename and current date-time
        Path filePath = Paths.get("computational", filename + "_" + formattedDateTime + ".txt");
        Data data = Data.getInstance();
        StringBuilder sharedString;
        if(Constants.UPPER_BOUNDING_HEURISTIC){
            sharedString = new StringBuilder(filename + "," + data.getCrewNumber() + "," + data.getTowerNumber() + "," +
                    "UB" + "," + (StaticSharedValues.budget - 1) + "," + epsilon + "," +
//                    Constants.SERVICE_BOUND_CUT + "," + Constants.USE_NODE_ELIMINATION_CREW_PULSE + "," +
//                    Constants.STRONG_BRANCHING + "," +
//                    "" +"," +"" + "," +
                    bestObjectiveValue + "," + totalCPU + "," + timeOut + ","+numZonesAssigned);

            numberOfZonesVisitedByTowers = numZonesAssigned;

            sharedString.append(System.lineSeparator());
        } else {
            sharedString = new StringBuilder(filename + "," + data.getCrewNumber() + "," + data.getTowerNumber() + "," +
                    "ICEA-MT" + "," + (StaticSharedValues.budget - 1) + "," + epsilon + "," +
//                    Constants.SERVICE_BOUND_CUT + "," + Constants.USE_NODE_ELIMINATION_CREW_PULSE + "," +
//                    Constants.STRONG_BRANCHING + "," +
//                    "" +"," +"" + "," +
                    bestObjectiveValue + "," + totalCPU + "," + timeOut + ","+numZonesAssigned);

            numberOfZonesVisitedByTowers = numZonesAssigned;
            double wekestLB = bestObjectiveValue;
            int count = 0;
            for(OptimizationResult optimizationResult : results){
                count++;
                double thisLB = optimizationResult.getLBtermination();
                wekestLB = Math.min(thisLB,wekestLB);
                sharedString.append(",").append(optimizationResult.getRootLB()).append(",")
                        .append(optimizationResult.getBestValue()).append(",")
                        .append(thisLB).append(",")
                        .append(optimizationResult.getNodesExplored()).append(",").append(optimizationResult.getTime());
            }
            if(count < 5){
                sharedString.append(",").append(" ").append(",")
                        .append(" ").append(",")
                        .append(" ").append(",")
                        .append(" ").append(",").append(" ");
            }

            double gap = Math.max(0.,(1. - wekestLB / Math.max(bestObjectiveValue,1.))*100);
            sharedString.append(",").append(wekestLB).append(",").append(gap);
            sharedString.append(System.lineSeparator());
        }


        gbl_string.append(sharedString);

        // Create the parent folder(s) if they do not already exist
        try {
            Files.createDirectories(filePath.getParent());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        try (FileWriter writer = new FileWriter(filePath.toFile())) {
            writer.write(sharedString.toString());
            System.out.println("intermediate file created successfully: " + filePath);
        } catch (IOException e) {
            System.err.println("An error occurred while writing to the file: " + e.getMessage());
        }
    }

    public void writeGlobalString() {
        // Get the current date and time in the desired format
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyMMdd.HHmmss");
        String formattedDateTime = LocalDateTime.now().format(formatter);
        String out_path = "multithread";
        if(Constants.UPPER_BOUNDING_HEURISTIC)
            out_path = "upperBound";
        if(Constants.WRITE_FOR_CREWS)
            out_path = "crewOnly";
        if(Constants.U_ICEA)
            out_path = "u_icea";
        boolean is0Towers = Data.getInstance().getTowerNumber() < 1;
        if(Constants.FIX_CREW_ROUTES) out_path = "crewFirstTowerSecond";
        if(Constants.NO_REPOSITIONING) out_path = "noReposition";
        if(Constants.BRANCH_AND_PRICE) out_path = "bpc";
        if(Constants.ROBUST) {
            int gamma = Constants.BUDGET;
            int alpha = (int) (100 * Constants.ALPHA);
            if(Constants.NO_REPOSITIONING){
                String opener = "robust/noReposition/g"+gamma+"/0"+alpha;
                out_path = opener;
            } else {
                if(Constants.COST_OF_PRIORITY) {
                    String opener = "robust/priority";
                    if(is0Towers)
                        out_path =opener+"/crewFirst/g"+gamma+"/0"+alpha;
                    else if (Constants.FIX_CREW_ROUTES){
                        out_path =opener+"/crewFirstTowerSecond/g"+gamma+"/0"+alpha;
                    } else
                        out_path =opener;
                } else {
                    String opener = "robust/g"+gamma+"/0"+alpha;
                    if(Constants.UPPER_BOUNDING_HEURISTIC)
                        opener = "robust/heuristic/g"+gamma+"/0"+alpha;
                    out_path = opener;
                    if(is0Towers)
                        out_path =opener+"/crewFirst";
                    else {
                        if(Constants.FIX_CREW_ROUTES)
                            out_path = opener+"/crewFirstTowerSecond";
                    }
                }
            }
        } else        if(Constants.COST_OF_PRIORITY) {
            out_path = "priority";
            if(is0Towers)
                out_path ="priority/crewFirst";
            else {
                if(Constants.FIX_CREW_ROUTES)
                    out_path = "priority/crewFirstTowerSecond";
            }
        }
        // Create the full file path with the given filename and current date-time
        Path filePath = Paths.get("summary/" + out_path + "/", formattedDateTime +"_"
                +Data.getInstance().getTowerNumber()+"towers" + ".csv");

        try {
            // Create the parent folder(s) if they do not already exist
            Files.createDirectories(filePath.getParent());

            try (FileWriter writer = new FileWriter(filePath.toFile())) {
                writer.write(gbl_string.toString());
                System.out.println("Global CSV file created successfully: " + filePath);
            }

        } catch (IOException e) {
            System.err.println("An error occurred while writing to the file: " + e.getMessage());
            e.printStackTrace();
        }
    }

    public void writeSolution(SolutionT solution) {
        if(solution==null)
            return;
        String out_path = "multithread";
        if(Constants.UPPER_BOUNDING_HEURISTIC)
            out_path = "upperBound";
        if(Constants.WRITE_FOR_CREWS)
            out_path = "crewOnly";
        if(Constants.U_ICEA)
            out_path = "u_icea";
        boolean is0Towers = Data.getInstance().getTowerNumber() < 1;
        if(Constants.FIX_CREW_ROUTES) out_path = "crewFirstTowerSecond";
        if(Constants.NO_REPOSITIONING) out_path = "noReposition";
        if(Constants.BRANCH_AND_PRICE) out_path = "bpc";
        if(Constants.ROBUST) {
            int gamma = Constants.BUDGET;
            int alpha = (int) (100 * Constants.ALPHA);
            if(Constants.COST_OF_PRIORITY) {
                String opener = "robust/priority";
                if(is0Towers)
                    out_path =opener+"/crewFirst/g"+gamma+"/0"+alpha;
                else if (Constants.FIX_CREW_ROUTES){
                    out_path =opener+"/crewFirstTowerSecond/g"+gamma+"/0"+alpha;
                } else
                    out_path =opener;
            } else {
                String opener = "robust/g"+gamma+"/0"+alpha;
                if(Constants.UPPER_BOUNDING_HEURISTIC)
                    opener = "robust/heuristic/g"+gamma+"/0"+alpha;
                out_path = opener;
                if(is0Towers)
                    out_path =opener+"/crewFirst";
                else {
                    if(Constants.FIX_CREW_ROUTES)
                        out_path = opener+"/crewFirstTowerSecond";
                }
            }
        } else        if(Constants.COST_OF_PRIORITY) {
            out_path = "priority";
            if(is0Towers)
                out_path ="priority/crewFirst";
            else {
                if(Constants.FIX_CREW_ROUTES)
                    out_path = "priority/crewFirstTowerSecond";
            }
        }
        Data data = Data.getInstance();
        String basename = filename;
        String name = basename.substring(0, basename.lastIndexOf('.'))
                + "-"+data.getCrewNumber()+"-"+data.getTowerNumber();
        String path = "summary/"+out_path+"/solutions/";
        String filePath = path + name+".txt";

        File file = new File(filePath);
        // Create parent folders if they don't exist
        File parentDir = file.getParentFile();
        if (parentDir != null && !parentDir.exists()) {
            parentDir.mkdirs();
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            writer.write("Technician Plans");
            writer.newLine();
            for (Route route : solution.getBest_crew()) {
                writer.write(route.getPattern().toString());
                writer.newLine();
            }
            writer.write("Tower Routes");
            writer.newLine();
            for(ArrayList<Integer> list : solution.getTowerRoutesHashSet()) {
                writer.write(list.toString());
                writer.newLine();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private boolean isFeasible;
    public void setFeasible(boolean b) {
        isFeasible = b;
    }

    public boolean isTimeOut() {
        return timeOut;
    }

    public void setTimeOut(boolean timeOut) {
        this.timeOut = timeOut;
    }
}
