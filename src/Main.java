import lib.*;
import problem.Constants;
import problem.analyses.OutputHandler;
import problem.multi.LocWriterT;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Main {
    public static void main(String[] args) throws Exception {

        String runParaFilePath = "run_parameters.txt";  // Default file path
        String defaultInstancePath = "TestInstances"; //default instance input
        String in = "";
        if (args.length > 0) {
            runParaFilePath = args[0];  // Use the provided path if available
            if(args.length > 1){
                in = args[1];
                if(args.length > 2)
                    throw new IllegalArgumentException("incorrect argument");
            }
        } else {
            in = defaultInstancePath;
        }

        LocWriterT locWriterT = new LocWriterT();
        initialize(runParaFilePath);

        Experiments experiments = new Experiments();
        ArrayList<Integer> towerNumbers = Utility.towerNumbers;


        if(Constants.SOLVE_FOR_CREWS)
            towerNumbers = new ArrayList<>(List.of(0));


            ArrayList<Integer> budgets = new ArrayList<>(new ArrayList<>(List.of(0)));
            ArrayList<Double> uncertainties = new ArrayList<>(new ArrayList<>(List.of(0.)));
            if(Constants.ROBUST) {
                if(Constants.DETERMINE_MINIMUM_NUMBER_CREWS) {
                    budgets = new ArrayList<>(new ArrayList<>(List.of(Constants.BUDGET)));
                    uncertainties = new ArrayList<>(new ArrayList<>(List.of(Constants.ALPHA)));
                } else {
                    budgets = new ArrayList<>(new ArrayList<>(List.of(Constants.BUDGET)));
                    uncertainties = new ArrayList<>(new ArrayList<>(List.of(Constants.ALPHA)));
//                    budgets = new ArrayList<>(new ArrayList<>(List.of(5,3,1)));
//                    uncertainties = new ArrayList<>(new ArrayList<>(List.of(0.35,0.25,0.10)));
                }
            }
            for(Double uncertaintyLevel : uncertainties){
                for (Integer budget : budgets){
                    if(Constants.ROBUST){
                        Constants.BUDGET = budget;
                        Constants.ALPHA = uncertaintyLevel;
                        int gamma = Constants.BUDGET;
                        int alpha = (int) (100 * Constants.ALPHA);
                        in = "RobustInstances/g"+gamma+"/0"+alpha;
                    }
                    //---------READ THE FILES FROM THE INPUT DIRECTORY---------
                    List<Path> files = Readers.getFiles(in);
                    LocalWriter.initialize();
                    TimerHelper timer = TimerHelper.getInstance();

                    if(Constants.DETERMINE_MINIMUM_NUMBER_CREWS){
                        Constants.SHOULD_RUN_BENDERS_HEURISTIC = false;
                        towerNumbers = new ArrayList<>(List.of(0));
                        for(Integer tNum : towerNumbers){
                            boolean[] tabu = new boolean[files.size()];
                            HashMap<String,Integer> filename2fleet = new HashMap<>();

                            while (true){
                                boolean shouldStop = true;
                                for (int k = 1; k < 41; k++) {
                                    int fleetSize = k;
                                    System.out.println("DETERMINING THE FLEET:"+k);
                                    for (int i = 0; i < files.size(); i++) {
                                        if(tabu[i])
                                            continue;
                                        double start = timer.startGlobal(Utility.bPCTiLim * 1e9);
                                        StaticSharedValues.feasibleFleetExperiment = false;
                                        Path file = files.get(i);
                                        LocalWriter.restart();
                                        String filename = file.getFileName().toString();
                                        LocalWriter.filename = filename;
                                        Readers.readInstance(file,tNum, fleetSize);
                                        //----------------------------
                                        experiments.run(filename, Utility.algo);
                                        //-------------------------------------
                                        if(StaticSharedValues.feasibleFleetExperiment){
                                            tabu[i] = true;
                                            filename2fleet.put(filename,fleetSize);
                                        } else {
                                            shouldStop = false;
                                        }
                                    }
                                    if(shouldStop)
                                        break;
                                }
                                if(shouldStop)
                                    break;
                            }
                            System.out.println("Printing fleets:");
                            for(String fileName : filename2fleet.keySet()) System.out.println(fileName+":"+filename2fleet.get(fileName));
                            for (String fileName : filename2fleet.keySet()) {
                                // Find the Path in "files" that matches the name
                                Path path = files.stream()
                                        .filter(f -> f.getFileName().toString().equals(fileName))
                                        .findFirst()
                                        .orElseThrow();

                                List<String> lines = Files.readAllLines(path);

                                if (!lines.isEmpty()) {
                                    String[] parts = lines.get(0).trim().split("\\s+");
                                    parts[0] = String.valueOf(filename2fleet.get(fileName));
                                    lines.set(0, String.join(" ", parts));
                                }

                                Files.write(path, lines);
                            }
                        }
                        System.exit(666);
                    }




                    for(Integer tNum : towerNumbers){
                        for (int fid = 0; fid < files.size(); fid++) {
                            Path file = files.get(fid);
                            System.out.println(file+" towers " +tNum);
                            double start = timer.startGlobal(Utility.bPCTiLim * 1e9);
                            //-------------------------
                            LocalWriter.restart();
                            StaticSharedValues.restart();
                            String filename = file.getFileName().toString();
                            LocalWriter.filename = filename;
                            if(Utility.algo==29){
                                locWriterT.setFilename(filename);
                                experiments.setLocWriterT(locWriterT);
                                experiments.setStartTime(start);
                            }

                            if(Utility.algo != 5) Readers.readInstance(file,tNum, 0);
                            //----------------------------
                            experiments.run(filename, Utility.algo);
                            double elapsed = timer.getTimePassedSeconds(start);
                            System.out.println("Total CPU:"+elapsed);
                            //-------------------------------------
                            if(Utility.algo == 29){

                            } else                         {
                                LocalWriter.totalCPU = elapsed;
                                LocalWriter.isOptimal = LocalWriter.totalCPU < Utility.bPCTiLim;
                                LocalWriter.writeValuesToCSV(filename.contains(".")
                                        ? filename.substring(0, filename.lastIndexOf('.'))
                                        : filename);
                            }
                        }

                    }
                    if(Utility.algo == 29){
                        locWriterT.writeGlobalString();
                    } else
                        LocalWriter.writeGlobalString(LocalWriter.filename);
                }
            }


        if(Constants.SOLUTION_ANALYSIS){
            System.out.println("The analysis files are in summary/analyses/percentRestored");
        }
        if(Utility.algo==266){
            OutputHandler handler = new OutputHandler();
            handler.writeOutput();
            handler.writePriorityMetricsCsv(
                    Path.of(
                            "summary/analyses",
                            "priority-order-metrics.csv"
                    )
            );
            handler.writeTowerMetricsCsv(
                    Path.of(
                            "summary/analyses",
                            "tower-route-metrics.csv"
                    )
            );

        }
    }

    private static void initialize(String filePath) {
//        new Utility(filePath);
        Utility.loadFromProperties();
        Msg.writeCurrentDateTimeToFile();
    }
}
