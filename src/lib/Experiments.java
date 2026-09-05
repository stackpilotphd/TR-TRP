package lib;

import com.gurobi.gurobi.GRBException;
import ilog.concert.IloException;
import problem.*;
import problem.BP.*;
import problem.analyses.*;
import problem.graph.Data;
import problem.milp.GenericTRTRPMIP;
import problem.milp.MIP;
import problem.milp.genericSolverTools.SolverConfig;
import problem.multi.Engine;
import problem.multi.LocWriterT;
import problem.multi.SolutionT;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.*;
import java.util.concurrent.ExecutionException;

public class Experiments {

    private LocWriterT locWriterT;
    private double startTime;

    public void setLocWriterT(LocWriterT locWriterT) {
        this.locWriterT = locWriterT;
    }

    public void setStartTime(double startTime) {
        this.startTime = startTime;
    }

    private void  isOptimalWriter(String filename){
        try {
            checkAndSaveOptimality(filename,Data.getInstance().getTowerNumber());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void checkAndSaveOptimality(String filename, int towerNumber) throws IOException {
        Path baseFolder = Paths.get("isOptimal");
        Path towerFolder = baseFolder.resolve(String.valueOf(towerNumber));
        Path summaryFile = baseFolder.resolve("opt_summary.csv");

        // Check whether isOptimal/<towerNumber>/<filename> exists and is a file
        Path fileToCheck = towerFolder.resolve(filename);
        boolean isOptimal = Files.isRegularFile(fileToCheck);

        // Make sure the isOptimal directory exists
        Files.createDirectories(baseFolder);

        // Add the CSV header if the file does not exist or is empty
        boolean writeHeader = !Files.exists(summaryFile) || Files.size(summaryFile) == 0;

        StringBuilder output = new StringBuilder();

        if (writeHeader) {
            output.append("filename,towerNumber,isOptimal")
                    .append(System.lineSeparator());
        }

        output.append(escapeCsv(filename))
                .append(",")
                .append(towerNumber)
                .append(",")
                .append(isOptimal)
                .append(System.lineSeparator());

        // Create the file if necessary, otherwise append
        Files.writeString(
                summaryFile,
                output.toString(),
                StandardCharsets.UTF_8,
                StandardOpenOption.CREATE,
                StandardOpenOption.APPEND
        );
    }

    private String escapeCsv(String value) {
        if (value.contains(",") || value.contains("\"") ||
                value.contains("\n") || value.contains("\r")) {
            return "\"" + value.replace("\"", "\"\"") + "\"";
        }
        return value;
    }

    public void run(String filename, int algo) throws IloException, IOException, ExecutionException, InterruptedException {
        if(false){
            gantGenerator();
            System.exit(2);
        }
        if(false){
            isOptimalWriter(filename);
            return;
        }
        if(false){
            OptimalTaskAndZoneStatistics.collect(filename,Data.getInstance().getTowerNumber());
            return;
        }
        if(false){
            OptimalObjectiveStatistics.collect(filename, Data.getInstance().getTowerNumber());
            return;
        }
        if(false){
            OptimalServiceRestoredStatistics.collect(filename, Data.getInstance().getTowerNumber());
            return;
        }

        if(Utility.algo != 29){
            Constants.MT_EXTRA_THREAD = false;
        }
        switch (algo) {
            case 2 -> {
                System.out.println("Solving MIP Model...");
                runMILP();
            }
            case 4 -> {
                System.out.println("Tower Length Expansion algorithm...");
                runLengthExpantionAlgorithm2();
            }

            case 29 -> {
                if(Constants.UPPER_BOUNDING_HEURISTIC){
                    runUpperBoundingHeuristic();
                    return;
                }
                double startTime = System.nanoTime();
                System.out.println("Running Multiple Threads...");
                Engine engine = new Engine(locWriterT);
                engine.execute(startTime);

                System.out.println("Over. Elapsed:"+(System.nanoTime()-startTime)*1e-9);
            }

            case 266 -> solutionAnalyses(filename);
            default -> throw new IllegalArgumentException(Msg.algoType);
        }
        System.out.println("over");
        System.out.println("*".repeat(50));
    }

    private void gantGenerator() throws IOException {
        Data data = Data.getInstance();

        Path experimentRoot = Paths.get("gantChartExperiments");

        int identifier = 3;
        String scenario = switch (identifier) {
            case 1 -> "crewFirstTowerSecond";
            case 2 -> "noRepositioning";
            case 3 -> "optimal";
            default -> "";
        };

        Path csv = CrewScheduleCsvExporter.export(
                data,
                experimentRoot,
                scenario
        );

        System.out.println("Written to: " + csv);
    }

    private void solutionAnalyses(String filename) throws IOException {
        if(!StaticCounters.isInitialized){
            PriorityOrderStatistics.reset();
            PositionalStatistics.reset();
            TowerRouteStatistics.reset();
            StaticCounters.isInitialized = true;
            int len = 10;
            StaticCounters.numberOfTasksAtPosition = new int[len];
            StaticCounters.numberSharedTasksAtPosition = new int[len];
            StaticCounters.totalWeightAtPosition = new double[len];
            StaticCounters.totalServiceAtPosition = new double[len];
            StaticCounters.priorityCounterAtPosition = new int[len];
            StaticCounters.numberOfTasksAtPositionInSharedInstances = new int[len];
        }

        Data data = Data.getInstance();

        double[] zoneWeights = data.getWeights();
        int[] zonePriorities = data.getZonePriority();

        ZoneAttributeProvider zoneAttributes =
                ZoneAttributeProvider.fromArrays(
                        zoneWeights,
                        zonePriorities
                );

        SolutionAnalysis analysis =
                new SolutionAnalysis(
                        filename,
                        zoneAttributes,
                        SolutionAnalysis
                                .TowerRouteFormat
                                .DEPOT_AT_BOTH_ENDS
                );
        analysis.analyze();
    }

    private static void runLengthExpantionAlgorithm2() throws IloException {
        Data data = Data.getInstance();

        if(data.getTowerNumber() <= 0 || Utility.algo == 22 || Utility.algo == 23
                || Utility.algo == 24 || Utility.algo == 41 || Constants.NO_REPOSITIONING) Constants.SHOULD_RUN_BENDERS_HEURISTIC = false;


        int bestHeuristicIteration = -1;


        ArrayList<Route> best_crew = new ArrayList<>();
        ArrayList<Route> best_towerRoutes = new ArrayList<>();
        ArrayList<Double> route_values = new ArrayList<>();
        ArrayList<ArrayList<Boolean>> route_waits = new ArrayList<>();

        double heuristicBestValue = Double.MAX_VALUE;

        LocalWriter.bestHeuristicIteration = bestHeuristicIteration;

        BendersCuts.toggle = false;

        StringBuilder builder = new StringBuilder();
        TimerHelper cpu = TimerHelper.getInstance();
        double start = System.nanoTime();
        int maxLen = data.getNodeNumber();
        int len = data.getTowerNumber();
        int prev_len;
        prev_len = len;

        int iceaIteration = 0;


        TimerHelper timer = TimerHelper.getInstance();
        timer.getTime();



        double optimal_value = heuristicBestValue;

        if(Utility.algo==1){
            //BPC
            len = StaticSharedValues.maximumTowerRouteLength;
            prev_len = 0;
        }


        EarlyTerminationICEA.isFirstIteration = true;
        EarlyTerminationICEA.limLB = prev_len;
        EarlyTerminationICEA.limUB = len;
        EarlyTerminationICEA.isRerun = false;
        while (len < maxLen) {

            iceaIteration++;
            StaticSharedValues.iceaIteration = iceaIteration;
            StaticSharedValues.iceaBuilder = new StringBuilder();

            StaticSharedValues.limLB = prev_len;
            StaticSharedValues.limUB = len;
            StaticSharedValues.isFirstIteration = true;
            //-----------------------------------------------------------------------
            //-----------------------------------------------------------------------
            //-----------------------------------------------------------------------
            BTree bnBTree = new BTree();
            if(!best_crew.isEmpty())
                bnBTree.setInitialSolution(best_crew,best_towerRoutes,route_values,route_waits,Math.min(heuristicBestValue,optimal_value));
            else
                bnBTree.setInitialUB(Math.min(heuristicBestValue,optimal_value));


            bnBTree.solve();
            double current_value = bnBTree.getObjValue();

            LocalWriter.iceaStrings.add(StaticSharedValues.iceaBuilder.toString());

            builder.append("Expansion#").append(iceaIteration).append("[").append(prev_len).append(";").append(len).append("]")
                    .append(":").append(StaticSharedValues.lowerBound).append("/")
                    .append(current_value).append(".CPU:").append(bnBTree.getElapsed()).append(".Nodes:").append(bnBTree.getNodeNumber())
                    .append(System.lineSeparator());
            System.out.println(builder.toString());



            if((optimal_value < 1e16 && current_value > optimal_value+1e-6)) {
                boolean shouldCut = true;
                if(bestHeuristicIteration != -1){
                    if(iceaIteration <= bestHeuristicIteration)
                        shouldCut = false;
                }

                if(shouldCut){
                    if(EarlyTerminationICEA.hasTerminatedEarly){
                        if(EarlyTerminationICEA.isFirstIteration) {
                            //CASE 1: we terminated early, but this is the first ICEA iteration;
                            //so, we must continue running the ICEA, and shouldcut = false
                            shouldCut = false;
                        }
                        else {
                            if(!EarlyTerminationICEA.stopCheckling){
                                //CASE 2: we terminated early, and this is the second ICEA iteration
                                if(current_value > EarlyTerminationICEA.bestBound){
                                    //CASE 2a: best solution from 2nd iteration is worse that the best lwoer bound from 1st iteartion
                                    //so, we need to prove that either the first iteration is optimal
                                    //or that the 1st one does not improve the solution
                                    //therefore, we need to finish running the suspended nodes
                                    //RERUN FIRST ITERATION
                                    StaticSharedValues.iceaIteration = 9;
                                    StaticSharedValues.iceaBuilder = new StringBuilder();

                                    StaticSharedValues.limLB = EarlyTerminationICEA.limLB;
                                    StaticSharedValues.limUB = EarlyTerminationICEA.limUB;
                                    StaticSharedValues.isFirstIteration = true;
                                    EarlyTerminationICEA.isRerun = true;

                                    BTree treeForRerun = new BTree();
                                    if(!best_crew.isEmpty())
                                        treeForRerun.setInitialSolution(best_crew,best_towerRoutes,route_values,route_waits,Math.min(heuristicBestValue,optimal_value));
                                    else
                                        treeForRerun.setInitialUB(Math.min(heuristicBestValue,optimal_value));
                                    treeForRerun.solve();
                                    double rerun_value = treeForRerun.getObjValue();
                                    if(rerun_value < current_value){
                                        //case I: after rerun, we find a better solution, which is optimal
                                        //update the optimal solution
                                        current_value = rerun_value;
                                        best_crew = treeForRerun.getBestCrew();
                                        best_towerRoutes = treeForRerun.getBestTower();
                                        route_values = treeForRerun.getRouteValues();
                                        route_waits = treeForRerun.getWaits();
                                        //terminate the ICEA procedure
                                        shouldCut = true;
                                    } else {
                                        //case II: after rerun, we prove that 1st iteration's solution is worse than the 2nd one
                                        //so, we need to run 3rd iteration of ICEA to prove optimality
                                        shouldCut = false;
                                    }
                                }
                                //else CASE 2b: current value < best bound from 1st iteration
                                //so, we do not need to finish running the first one
                                //because the second solution is already better than the best lower bound from 1st iteeration
                            }
                        }

                    }
                }

                if(shouldCut){
                    System.out.println("Expansion total CPU:"+cpu.getTimePassedSeconds(start));
                    System.out.println(current_value+">"+optimal_value);
                    System.out.println("Terminating...");
                    break;
                }
            }

            if(current_value < heuristicBestValue -1e-6){
                best_crew = bnBTree.getBestCrew();
                best_towerRoutes = bnBTree.getBestTower();
                route_values = bnBTree.getRouteValues();
                route_waits = bnBTree.getWaits();
                bestHeuristicIteration = -1;
            }
            optimal_value = Math.min(current_value,optimal_value);

            if((data.getTowerNumber() <= 0))
                break;

            len++;
            prev_len = len;

            if (timer.hasTimedOut()) {
                LocalWriter.isOptimal = false;
                break;
            }
            if(Constants.DETERMINE_MINIMUM_NUMBER_CREWS)break;
            if(!EarlyTerminationICEA.isFirstIteration)
                EarlyTerminationICEA.stopCheckling = true; //we check only the 1st and 2nd iterations
            EarlyTerminationICEA.isFirstIteration = false;
            if(Utility.algo == 23 || Utility.algo == 24 || Utility.algo==1 || Constants.NO_REPOSITIONING)
                break;
        }

        HashSet<ArrayList<Integer>> towerRoutesHashSet = new HashSet<>();
        for (int i = 0; i < best_towerRoutes.size(); i++) {
            Route route = best_towerRoutes.get(i);
            double v = route_values.get(i);
            System.out.print("\t" + v + ":");
            System.out.print(route.getPattern().toString());
            System.out.println(route_waits.get(i));

            towerRoutesHashSet.add(route.getPattern());
            if (!route_waits.get(i).equals(route.getWaitBooleans()))
                throw new IllegalArgumentException("wait patternes do not match");
        }

        System.out.println("Optimal Tower Routes:");
        for(ArrayList<Integer> list : towerRoutesHashSet)
            System.out.println("\t"+list.toString());


        System.out.println("Optimal Repair Crews:");
        for(Route route : best_crew)
            System.out.println(route.getPattern().toString());

        if(Utility.algo == 21){
            String filePath = "fixed_crew_routes.txt";
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
                for (Route route : best_crew) {
                    writer.write(route.getPattern().toString());
                    writer.newLine(); // each pattern on its own line
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        if(data.getTowerNumber() <= 0){
            String filePath = "summary/joint/fixedCrews/"+ LocalWriter.filename;
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
                for (Route route : best_crew) {
                    writer.write(route.getPattern().toString());
                    writer.newLine(); // each pattern on its own line
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        System.out.print("Optimal Repair Schedule:[");
        double[] restoration = new double[data.getNodeNumber()];
        double[] taksCompletion = new double[data.getTasks()];
        for(Route route : best_crew){
            for (int j = 1; j < route.getPattern().size() - 1; j++) {
                int key = route.getPattern().get(j);
                taksCompletion[key] += route.getArrivals().get(j);
            }
        }
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double max = 0;
            for(Integer k : data.getZone2tasks()[i])
                max = Math.max(taksCompletion[k],max);
            restoration[i] = max;
        }
        boolean[] tabu = new boolean[data.getNodeNumber()];
        for (int k = 1; k < data.getNodeNumber(); k++) {
            if(k!=1)
                System.out.print(", ");
            double earliest = Double.MAX_VALUE;
            int id = -1;
            for (int i = 1; i < data.getNodeNumber(); i++) {
                if(tabu[i])
                    continue;
                if(restoration[i] < earliest){
                    earliest = restoration[i];
                    id = i;
                }
            }
            tabu[id] = true;
            System.out.print(id);
        }
        System.out.println("]");
        //--------------------------------------------------------------------
        //--------------------------------------------------------------------
        //--------------------------------------------------------------------
        LocalWriter.bestObjectiveValue = optimal_value;
        if(cpu.hasTimedOut())
            LocalWriter.isOptimal = false;

        System.out.println("Final cost:"+optimal_value);


        {
            if(Constants.SHOULD_WRITE_SOLUTION && !Constants.DETERMINE_MINIMUM_NUMBER_CREWS){
                String out_path;
                switch (Utility.algo) {
                    case 1 -> out_path = "bpc";
                    case 2 -> out_path = "solver";
                    case 22 -> out_path = "crewFirstTowerSecond";
                    default -> out_path = "N/A";
                }

                if(Constants.ROBUST)
                    out_path = "robust";

                String path = "summary/solutions/";
                if(!out_path.equals("N/A")) {
                    path = "summary/" + out_path + "/" + "solutions/";
                    if(Constants.NO_REPOSITIONING && Utility.algo >= 60)
                        path = path+"/no_repo/";
                }

                String baseName = LocalWriter.filename;
                String name = baseName.substring(0, baseName.lastIndexOf('.'))
                        + "-"+data.getCrewNumber()+"-"+data.getTowerNumber();
                String filePath = path + name+".txt";

                try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
                    writer.write("Technician Plans");
                    writer.newLine();
                    for (Route route : best_crew) {
                        writer.write(route.getPattern().toString());
                        writer.newLine();
                    }
                    writer.write("Tower Routes");
                    writer.newLine();
                    for(ArrayList<Integer> list : towerRoutesHashSet) {
                        writer.write(list.toString());
                        writer.newLine();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }


    private void runUpperBoundingHeuristic() throws IloException {
        Data data = Data.getInstance();

        Constants.SHOULD_RUN_BENDERS_HEURISTIC = true;

        if(data.getTowerNumber() <= 0 || Utility.algo == 22 || Utility.algo == 23
                || Utility.algo == 24 || Utility.algo == 41 || Constants.NO_REPOSITIONING) Constants.SHOULD_RUN_BENDERS_HEURISTIC = false;




        HashMap<Integer,ArrayList<Route>> iteration2crewRoute = new HashMap<>();
        HashMap<Integer,ArrayList<Route>> iteration2towerRoute = new HashMap<>();
        HashMap<Integer,ArrayList<Double>> iteration2towerValues = new HashMap<>();
        HashMap<Integer,ArrayList<ArrayList<Boolean>>> iteration2towerWaits = new HashMap<>();
        HashMap<Integer,Double> iteration2bestValue = new HashMap<>();
        int bestHeuristicIteration = -1;
        double bestHeuristicObjvective = 9999999;
        double heuristicTotalCPU = 0;
        TimerHelper cpu = TimerHelper.getInstance();
        double gbl_start = System.nanoTime();
        {
            Random random = new Random(20250910);
            BendersCuts.toggle = true;
            BendersCuts.matrixY = new int[data.getNodeNumber()];
            BendersCuts.hashSetY = new HashSet<>();
            Constants.RELAX_MIP = true;
            int len = data.getTowerNumber();
            int prev_len;
            prev_len = len;
            int maxLen = data.getNodeNumber();
            int iceaIteration = 0;
            StringBuilder builder = new StringBuilder();
            double optimal_value = Double.MAX_VALUE;



            ArrayList<Route> best_crew;
            ArrayList<Route> best_towerRoutes;
            ArrayList<Double> route_values;
            ArrayList<ArrayList<Boolean>> route_waits;
            boolean gbl_timeout = false;

            while (len < maxLen) {
                if(gbl_timeout)
                    break;
                double innerTimer = System.nanoTime();
                ArrayList<Route> local_best_crew = null;
                ArrayList<Route> local_best_towerRoutes = null;
                ArrayList<Double> local_route_values = null;
                ArrayList<ArrayList<Boolean>> local_route_waits = null;

                iceaIteration++;
                StaticSharedValues.iceaIteration = iceaIteration;

                StaticSharedValues.limLB = prev_len;
                StaticSharedValues.limUB = len;
                StaticSharedValues.isFirstIteration = true;
                //-----------------------------------------------------------------------
                //-----------------------------------------------------------------------
                MIP mip = new MIP();
                mip.buildAndSolve2index();
                int MAXIMUM_CONSECUTIVE_NON_IMPROVEMENTS = 30;
                int non_improving_counter = 0;
                double local_optimum = Double.MAX_VALUE;
                while (true) {
                    if((System.nanoTime() - gbl_start)*1e-9 > Constants.BENDERS_TOTAL_TILIM){
                        System.out.println("Benders Heuristic time out");
                        gbl_timeout = true;
                        break;
                    }
                    if((System.nanoTime() - innerTimer)*1e-9 > Constants.BENDERS_SINGLE_ITERATION_TILIM){
                        break;
                    }

                    mip.solve();


                    if(mip.getObjective() >= local_optimum - 1e-6)
                        break;

                    BTree bnBTree = new BTree();
                    bnBTree.setInitialUB(local_optimum);
                    bnBTree.solve();
                    double current_value = bnBTree.getObjValue();
                    if(current_value < local_optimum){
                        local_best_crew = bnBTree.getBestCrew();
                        local_best_towerRoutes = bnBTree.getBestTower();
                        local_route_values = bnBTree.getRouteValues();
                        local_route_waits = bnBTree.getWaits();
                        local_optimum = current_value;
                        non_improving_counter = 0;
                    } else {
                        non_improving_counter++;
                    }
                    System.out.println("Benders["+BendersCuts.currentValue+","+current_value+"]. Local Optimum:"+local_optimum
                            +",counter:"+non_improving_counter+";CPU:"+(System.nanoTime() - innerTimer)*1e-9);
                    if(non_improving_counter >= MAXIMUM_CONSECUTIVE_NON_IMPROVEMENTS) {
                        System.out.println("REACHED MAXIMUM CONSECUTIVE NON IMPROVEMENTS");
                        break;
                    }

                    if(current_value > BendersCuts.currentValue - 1e-6){
                        mip.addOptimalityCut(current_value);
                        continue;
                    }
                    break;
                }

                //-----------------------------------------------------------------------
                if(!gbl_timeout){
                    if((System.nanoTime() - innerTimer)*1e-9 < Constants.BENDERS_SINGLE_ITERATION_TILIM) {
                        int choose = nChooseK(data.getNodeNumber()-1,len);
                        int offset = BendersCuts.hashSetY.size();

                        if(choose > offset){
                            int randomIterations = 0;
                            if(true){
                                //DIVERSIFY AND RUN ONE MORE ITERATION
                                {
                                    ArrayList<Integer> integers = new ArrayList<>();
                                    boolean[]tabu = new boolean[data.getNodeNumber()];

                                    while (integers.size() != len) {
                                        int min = Integer.MAX_VALUE;
                                        int candidate = -1;
                                        for (int i = 1; i < data.getNodeNumber(); i++) {
                                            if(!tabu[i]){
                                                int v = BendersCuts.matrixY[i];
                                                if(v < min){
                                                    candidate = i;
                                                    min = v;
                                                }
                                            }
                                        }
                                        if(candidate != -1){
                                            integers.add(candidate);
                                            tabu[candidate] = true;
                                        } else
                                            break;
                                    }

                                    if(!BendersCuts.hashSetY.contains(new HashSet<>(integers)) && integers.size() == len){
                                        randomIterations++;
                                        for(Integer i : integers) BendersCuts.matrixY[i]++;
                                        BendersCuts.hashSetY.add(new HashSet<>(integers));
                                        BTree bnBTree = new BTree();
                                        bnBTree.setInitialUB(local_optimum);
                                        bnBTree.solve();
                                        double current_value = bnBTree.getObjValue();
                                        if(current_value < local_optimum){
                                            local_best_crew = bnBTree.getBestCrew();
                                            local_best_towerRoutes = bnBTree.getBestTower();
                                            local_route_values = bnBTree.getRouteValues();
                                            local_route_waits = bnBTree.getWaits();
                                            local_optimum = current_value;
                                            System.out.println("Benders-Diversification["+BendersCuts.currentValue+","+current_value+"]. Local Optimum:"+local_optimum+ ",counter:"+non_improving_counter);
                                        }
                                    }
                                }
                            }

                            double[] raw = new double[data.getNodeNumber()];
                            int multiplier = data.getNodeNumber() - 1;

                            PriorityQueue<Tuple> tuples = new PriorityQueue<>(Comparator.comparingDouble(o -> o.v));
                            for (int i = 1; i < data.getNodeNumber(); i++) tuples.add(new Tuple(i, BendersCuts.matrixY[i]));
                            while (!tuples.isEmpty()) {
                                Tuple t = tuples.poll();
                                raw[t.id] = multiplier;   // more frequent → smaller multiplier
                                if(raw[t.id] < 1e-6)
                                    throw new IllegalArgumentException("0 probability!");
                                multiplier--;
                            }

                            double total = 0.0;
                            for (int i = 1; i < raw.length; i++) total += raw[i];

                            double[] probabilities = new double[data.getNodeNumber()];
                            for (int i = 1; i < raw.length; i++) {
                                probabilities[i] = raw[i] / total;
                            }

                            HashSet<HashSet<Integer>> hashSet = new HashSet<>(BendersCuts.hashSetY);
                            while((System.nanoTime() - innerTimer)*1e-9 < Constants.BENDERS_SINGLE_ITERATION_TILIM){
                                if(randomIterations + offset >= choose)
                                    break;
                                HashSet<Integer> integers = weightedSample(probabilities, len, random);
                                if(hashSet.contains(integers))
                                    continue;
                                if(!BendersCuts.hashSetY.contains(new HashSet<>(integers))){
                                    for(Integer i : integers) BendersCuts.matrixY[i]++;
                                    randomIterations++;
                                    hashSet.add(integers);
                                    BendersCuts.hashSetY.add(integers);
                                    BTree bnBTree = new BTree();
                                    bnBTree.setInitialUB(local_optimum);
                                    bnBTree.solve();
                                    double current_value = bnBTree.getObjValue();
                                    if(current_value < local_optimum){
                                        local_best_crew = bnBTree.getBestCrew();
                                        local_best_towerRoutes = bnBTree.getBestTower();
                                        local_route_values = bnBTree.getRouteValues();
                                        local_route_waits = bnBTree.getWaits();
                                        local_optimum = current_value;
                                        System.out.println("Benders-Random["+BendersCuts.currentValue+","+current_value+"]. Local Optimum:"+local_optimum
                                                +",counter:"+randomIterations+";CPU:"+(System.nanoTime() - innerTimer)*1e-9);
                                    }
                                }
                            }
                            System.out.println("Random Iterations:"+randomIterations);
                        }
                    }
                }
                double current_value = local_optimum;
                double timer = (System.nanoTime() - innerTimer)*1e-9;
                builder.append("Ben-Heuristic#").append(iceaIteration).append("[").append(prev_len).append(";").append(len).append("]")
                        .append(":").append(StaticSharedValues.lowerBound).append("/")
                        .append(current_value).append(".CPU:").append(timer)
                        .append(System.lineSeparator());
                System.out.println(builder.toString());

                if(true){
                    System.out.println("Ben-Matrix#"+iceaIteration+":");
                    for (int i = 1; i < data.getNodeNumber(); i++) System.out.println("\t"+i+":"+BendersCuts.matrixY[i]);
                    System.out.println("------------------------------------");
                }

                if((optimal_value < 1e16 && current_value > optimal_value+1e-6)) {
                    heuristicTotalCPU = cpu.getTimePassedSeconds(gbl_start);
                    System.out.println("Heuristic total CPU:"+heuristicTotalCPU);
                    System.out.println(current_value+">"+optimal_value);
                    System.out.println("Terminating...");
                    break;
                }
                optimal_value = current_value;
                bestHeuristicObjvective = optimal_value;
                best_crew = local_best_crew;
                best_towerRoutes = local_best_towerRoutes;
                route_values = local_route_values;
                route_waits = local_route_waits;
                if(best_crew != null){
                    if(!best_crew.isEmpty()){
                        iteration2crewRoute.put(iceaIteration,best_crew);
                        iteration2towerRoute.put(iceaIteration,best_towerRoutes);
                        iteration2towerValues.put(iceaIteration,route_values);
                        iteration2towerWaits.put(iceaIteration,route_waits);
                    }
                    bestHeuristicIteration = iceaIteration;
                }
                if((data.getTowerNumber() <= 0))
                    break;

                len++;
                prev_len = len;
            }
            BendersCuts.toggle = false;

            LocalWriter.bendersBestValue = bestHeuristicObjvective;
            LocalWriter.bendersCPU = (System.nanoTime() - gbl_start)*1e-9;
        }




        ArrayList<Route> best_crew = new ArrayList<>();
        ArrayList<Route> best_towerRoutes = new ArrayList<>();
        ArrayList<Double> route_values = new ArrayList<>();
        ArrayList<ArrayList<Boolean>> route_waits = new ArrayList<>();

        double heuristicBestValue = Double.MAX_VALUE;
        if(bestHeuristicIteration != -1) {
            best_crew = iteration2crewRoute.get(bestHeuristicIteration);
            best_towerRoutes = iteration2towerRoute.get(bestHeuristicIteration);
            route_values = iteration2towerValues.get(bestHeuristicIteration);
            route_waits = iteration2towerWaits.get(bestHeuristicIteration);
            heuristicBestValue = bestHeuristicObjvective;

            HashSet<ArrayList<Integer>> hashSet = new HashSet<>();


            for (int i = 0; i < best_towerRoutes.size(); i++) {
                Route route = best_towerRoutes.get(i);
                hashSet.add(route.getPattern());
            }
            System.out.println("Initial Tower Routes:");
            for(ArrayList<Integer> list : hashSet)
                System.out.println("\t"+list.toString());


            System.out.println("Initial Repair Crews:");
            for(Route route : best_crew)
                System.out.println("\t"+route.getPattern().toString() + " | " + " Total travel time: " + route.getArrivals().getLast() + "/" + data.getHorizon());


            System.out.println("HEURISTIC SOLUTION at iteration " + bestHeuristicIteration + ":"+heuristicBestValue);
        }
        LocalWriter.bestHeuristicIteration = bestHeuristicIteration;

        BendersCuts.toggle = false;

        System.out.println("Upper Bounding Heuristic Over");

        heuristicTotalCPU = cpu.getTimePassedSeconds(gbl_start);
        SolutionT best_solution = new SolutionT(heuristicBestValue,best_crew,best_towerRoutes,route_values,route_waits);
        best_solution.print();
        int numZonesAssignedToTowers = best_solution.getNumZonesAssignedToTowers();

        locWriterT.writeIntermediateResults(heuristicBestValue,heuristicTotalCPU,numZonesAssignedToTowers,null);
        locWriterT.writeSolution(best_solution);

    }

    private static void runGenericMILP() throws IOException, IloException, GRBException {
        SolverConfig config = SolverConfig.load("config/solver.properties");
        GenericTRTRPMIP mip = new GenericTRTRPMIP(config);
        Path sharedResults = Path.of("summary/solver", "milp-results.csv");
        if(Constants.ROBUST)
            sharedResults = Path.of("summary/solver/robust", "milp-results.csv");
        mip.buildAndSolveMIP(sharedResults);
    }






    private static void runMILP() throws IloException {
        try {
            runGenericMILP();
        } catch (IOException | GRBException e) {
            throw new RuntimeException(e);
        }
    }


    public static HashSet<Integer> weightedSample(double[] probs, int len, Random random) {
        HashSet<Integer> chosen = new HashSet<>();
        List<Integer> available = new ArrayList<>();
        for (int i = 1; i < probs.length; i++) { // skip 0 if needed
            if (probs[i] > 0) available.add(i);
        }

        while (chosen.size() < len && !available.isEmpty()) {
            // total probability of available set
            double sum = 0.0;
            for (int idx : available) sum += probs[idx];

            // roulette-wheel pick
            double r = random.nextDouble() * sum;
            double cumulative = 0.0;
            int picked = -1;
            for (int idx : available) {
                cumulative += probs[idx];
                if (r <= cumulative) {
                    picked = idx;
                    break;
                }
            }

            chosen.add(picked);
            available.remove((Integer) picked);
        }
        return chosen;
    }

    public static int nChooseK(int n, int k) {
        if (k < 0 || k > n) return 0;
        if (k == 0 || k == n) return 1;

        k = Math.min(k, n - k); // symmetry
        int result = 1;

        for (int i = 1; i <= k; i++) {
            result = result * (n - i + 1) / i;
        }

        return result;
    }
}
