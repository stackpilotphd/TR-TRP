package problem.BP;

import ilog.concert.IloException;
import lib.*;
import problem.Constants;
import problem.graph.Arc;
import problem.graph.Data;
import problem.multi.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.regex.Pattern;
import java.util.stream.Stream;

public class BTree {
    private enum BranchingMode { DEFAULT, PSEUDOCOST }

    private static final int LOWER_BOUND_STALL_WINDOW = 100;
    private static final double LOWER_BOUND_STALL_DELTA = 1e-6;
    private static final Pattern TOWER_ARC_BRANCH_PATTERN = Pattern.compile("TA(\\d+),(\\d+)");
    private static final Pattern CREW_ARC_BRANCH_PATTERN = Pattern.compile("CA(\\d+),(\\d+)");
    private final Data data;
    private boolean hasTimedOut,hasFoundFeasibleSolution;
    private TimerHelper timer;
    private double best_value;
    private ArrayList<Route> best_route;
    private ArrayList<Double> route_values;
    private ArrayList<ArrayList<Boolean>> route_waits;
    private ArrayList<Route> best_crew;
    private BNode best_node;
    private double[] best_ys;
    private int depth;
    private BNode deepest;
    private double elapsed;
    private int tot_nodes;
    private int heuristicMIPrunCounter;
    private LowerBoundProgressTracker lowerBoundProgressTracker;
    private PseudocostTable pseudocostTable;
    private BranchingMode branchingMode;


    private final boolean isMultiThread;

    public BTree() {
        isMultiThread = Utility.algo==29 && !Constants.UPPER_BOUNDING_HEURISTIC;
        this.data = Data.getInstance();
    }

    private boolean isThreadFiveOrGreater;
    public boolean solve() throws IloException {
        isThreadFiveOrGreater = ucea_constraintl != null && (ucea_constraintl.id >= 5);


        double timer28 = System.nanoTime();
        bestLB = 999;
        heuristicMIPrunCounter = 0;
        lowerBoundProgressTracker = new LowerBoundProgressTracker(
                LOWER_BOUND_STALL_WINDOW, LOWER_BOUND_STALL_DELTA);
        pseudocostTable = new PseudocostTable();
        branchingMode = BranchingMode.DEFAULT;
        StaticSharedValues.mipHeuristicTimer = 0;
        if(Utility.algo ==1)
            initialValue = 1e9;

        depth = -1;

        PriorityQueue<BNode> queue = new PriorityQueue<>();
        hasTimedOut = false;

        timer = TimerHelper.getInstance();
        if(!Constants.MT_EXTRA_THREAD&&isExtraBound){
            timer.startGlobal(300 * 1e9);
        }

        double start = timer.getTime();
        initialize(queue);
        int node_num = 0;


        while (!queue.isEmpty()) {



            BNode node = queue.poll();
            if(node.getDepth() > depth) {
                depth = node.getDepth();
                deepest = node;
            }

            LocalWriter.bestLBatTermination = node.getCost();
            this.bestLB = node.getCost();

            if (prune(node)) {
                recordGlobalLowerBound(queue, node_num);
                continue;
            }

            if (timer.hasTimedOut()) {
                hasTimedOut = true;
                LocalWriter.isOptimal = false;
                LocalWriter.bestLBatTermination = queue.peek() != null ? queue.peek().getCost() : 0;
                LocalWriter.remainingNodes = queue.size();
                break;
            }

            if(isMultiThread && Constants.U_ICEA){
                if(ucea_constraintl.id >= 5){
                    if(sharedBounds.hasTerminatedForuth()) {
                        //WE CAN TERMINATE THREAD 5 AND ONWARDS ONLY IF THE CURRENT BEST SOLUTION IS IN ITERATIONS T1,T2 OR T3
                        int bestid = sharedBounds.getBestID();
                        if(bestid > 0 && bestid < 4) {
                            System.out.println("TERMINATING THREAD 5 BECAUSE T4 HAS FINISHED AND BEST ID IS "+bestid+".");
                            break;
                        }
                    }
                } else if(ucea_constraintl.id == 4){
                    if(sharedBounds.hasTerminatedThired()){
                        //WE CAN TERMINATE THREAD 4 AND ONWARDS ONLY IF THE CURRENT BEST SOLUTION IS IN ITERATIONS T1,T2
                        int bestid = sharedBounds.getBestID();
                        if(bestid > 0 && bestid < 3) {
                            System.out.println("TERMINATING THREAD 4 BECAUSE T3 HAS FINISHED AND BEST ID IS "+bestid+".");
                            break;
                        }
                    }
                } else if(ucea_constraintl.id == 3)
                    if(sharedBounds.hasTerminatedSicend()){
                        //WE CAN TERMINATE THREAD 3 AND ONWARDS ONLY IF THE CURRENT BEST SOLUTION IS IN ITERATION T1
                        int bestid = sharedBounds.getBestID();
                        if(bestid == 1) {
                            System.out.println("TERMINATING THREAD 3 BECAUSE T2 HAS FINISHED AND BEST ID IS "+bestid+".");
                            break;
                        }
                    }
            }

            if(BendersCuts.toggle){
                if(node.getDepth() >= Constants.BENDERS_MAX_DEPTH){
                    recordGlobalLowerBound(queue, node_num);
                    continue;
                }
            }

            if(!BendersCuts.toggle)
                if (Constants.SOLVE_FOR_CREWS || Constants.BRANCH_AND_PRICE || (node_num % 100 == 0 )|| (isThreadFiveOrGreater && node_num % 20 == 0)) {
                    if(isMultiThread) {
                        node.print(node_num, best_value, ucea_constraintl.id, sharedBounds.getUB(),sharedBounds.getBestID());
                    }
                    else
                        node.print(node_num, best_value);
                }


            solveNode(node, queue);
            recordGlobalLowerBound(queue, node_num);

            if(isExtraBound){
                if (timer.hasTimedOut()){
                    if(queue.isEmpty())
                        this.bestLB = node.getCost();
                }
            }
            node_num++;
        }


        if(!BendersCuts.toggle)
            System.out.println(
                printOuter(node_num, timer.getTimePassedSeconds(start)));
        if(BendersCuts.toggle) return true;

        printSolution();
        if (best_node != null&&!isMultiThread) best_node.printBranchStringsWithCosts();
        System.out.println(".".repeat(50));
        System.out.println("Total Number of Branch and Bound Nodes:" + node_num);


        if(isMultiThread){
            sharedBounds.updateBest(best_value,ucea_constraintl.id);
        }


        
        System.out.println("BnB cost:"+best_value);
        this.elapsed = timer.getTimePassedSeconds(start);
        this.tot_nodes = node_num;

        StaticSharedValues.iceaBuilder.append(tot_nodes);
        StaticSharedValues.iceaBuilder.append(",");
        StaticSharedValues.iceaBuilder.append(elapsed);
        StaticSharedValues.iceaBuilder.append(",");
        StaticSharedValues.iceaBuilder.append(best_value);

        LocalWriter.totalNodesExplored += node_num;

        if(deepest != null)
            System.out.println("Deepest node:"+deepest.getDepth());

        if(EarlyTerminationICEA.isRerun){
            EarlyTerminationICEA.suspendedNodes.clear();
        }


        if(Constants.U_ICEA && ucea_constraintl!=null){
            switch (ucea_constraintl.id){
                case 2 -> {
                    sharedBounds.setSecondThreadTerminated(true);
                }
                case 3 -> {
                    sharedBounds.setHasThirdThreadTerminated(true);
                }
                case 4 -> {
                    sharedBounds.setHasFourthThreadTerminated(true);
                }
            }
        }
        return true;
    }

    private void setBestWaits(ArrayList<ArrayList<Boolean>> routeWaits) {
        route_waits = routeWaits;
    }

    private void setBestRouteValues(ArrayList<Double> routeValues) {
        route_values = routeValues;
    }

    private void setBestTower(ArrayList<Route> bestTowerRoutes) {
        this.best_route = bestTowerRoutes;
    }


    private String printOuter(int node_num, double time) {
        if(isMultiThread){
            return "\t\tT"+ucea_constraintl.id+ "'s Best LB and Cost:[" + bestLB+";" + best_value
                    + "],Branch and Bound Nodes:" + node_num + ",CPU:" + time;
        }
        return "Best cost:[" + best_value
                + "],Branch and Bound Nodes:" + node_num + ",CPU:" + time;
    }




    private void solveNode(BNode node, PriorityQueue<BNode> queue) throws IloException {
        double pc_time = timer.getTime();
        PC pc = new PC();
        LP lp = new LP();
        lp.setICEAconsraint(ucea_constraintl, isThreadFiveOrGreater);
        lp.setSharedBound(sharedBounds);
        if(Constants.MT_EXTRA_THREAD){
            if(isExtraBound || isThreadFiveOrGreater)
                lp.setExtraBound(true);
        }

        boolean pc_result = pc.run(lp, node, best_value);
        if (pc_result) updatePseudocostObservation(node, lp.getCost());
        if(node.getDepth() <= 0)
            this.rootLB = lp.getCost();


        if(BendersCuts.toggle){
            if(!lp.solve()){
                lp.clear();
                return;
            }
            if(!lp.isFeasible()){
                double costMIP = lp.reconstructMIP();
                if(Constants.CONSOLE)            System.out.println("MIP COST:"+costMIP);
                if(costMIP < best_value - 1e-6) {
                    double previousCost = best_value;
                    double relative_gap = (previousCost - costMIP) / previousCost * 100;
                    updateBestValue(costMIP);
                    best_route.clear();
                    best_crew.clear();
                    best_route.addAll(lp.getMipTowers());
                    best_crew.addAll(lp.getMipCrews());
                    route_values.clear();
                    route_waits.clear();

                    ArrayList<Double> routeVals = new ArrayList<>();
                    ArrayList<ArrayList<Boolean>> routeWaits = new ArrayList<>();
                    for(Route route : lp.getMipTowers()){
                        routeVals.add(1.);
                        routeWaits.add(route.getWaitBooleans());
                    }

                    route_values.addAll(routeVals);
                    route_waits.addAll(routeWaits);
                    hasFoundFeasibleSolution  = true;
                }
                if(node.getDepth()+1 >= Constants.BENDERS_MAX_DEPTH){
                    lp.clear();
                    return;
                }
            }
        }

        if(Constants.DETERMINE_MINIMUM_NUMBER_CREWS)return;
        if(Constants.CONSOLE) System.out.println("Price and Cut time = " + timer.getTimePassedSeconds(pc_time) + ", LB:" + lp.getCost());
        if (!pc_result) {
            if(Constants.CONSOLE)
                System.out.println("pruning the node...");
            node.setCostWhenPruned(lp.getCost());
            lp.clear();
            return;
        }

        if(Constants.SHOULD_RUN_MIP_HEURISTIC && Utility.algo != 1 && !isThreadFiveOrGreater){
            if(node.getDepth() >= 15){
                boolean shouldRun = best_value >= 9999999;
                if(!shouldRun) shouldRun = this.heuristicMIPrunCounter < 101;
                if(shouldRun){
                    if(!lp.isFeasible()){
                        this.heuristicMIPrunCounter++;
                        double mipHeuristicTimerStart = System.nanoTime();
                        double costMIP = lp.reconstructMIP();
                        if(Constants.CONSOLE)            System.out.println("MIP COST:"+costMIP);
                        StaticSharedValues.mipHeuristicTimer += (System.nanoTime() - mipHeuristicTimerStart);
                        double mipObjective = costMIP;
                        if(mipObjective < best_value) {
                            double previousCost = best_value;
                            double relative_gap = (previousCost - mipObjective) / previousCost * 100;
                            updateBestValue(mipObjective);
                            best_route.clear();
                            best_crew.clear();
                            best_route.addAll(lp.getMipTowers());
                            best_crew.addAll(lp.getMipCrews());
                            route_values.clear();
                            route_waits.clear();

                            ArrayList<Double> routeVals = new ArrayList<>();
                            ArrayList<ArrayList<Boolean>> routeWaits = new ArrayList<>();
                            for(Route route : lp.getMipTowers()){
                                routeVals.add(1.);
                                routeWaits.add(route.getWaitBooleans());
                            }

                            route_values.addAll(routeVals);
                            route_waits.addAll(routeWaits);
                            hasFoundFeasibleSolution  = true;
                            if(isMultiThread) {
                                System.out.println("Updating the MIP UB(T" + ucea_constraintl.id + ");" + best_value + " (" + relative_gap + "%)");
                            }
                            else
                                System.out.println("Updating the MIP UB("+node.getDepth()+");" + best_value + " ("+relative_gap+"%)");
                            LocalWriter.heuristicMIPDepthFoundBestSolution = node.getDepth();
                        }
                    }
                }

            }
        }

        double previous_cost = node.getCost();
        double current_cost = lp.getCost();
        if(current_cost >= previous_cost - 1e-6){
            node.increaseConsecutiveNonImprovement();
        } else
            node.restartConsecutiveNonImprovement();

        if (lp.isFeasible()) {
            if (lp.getCost() < best_value) {
                double previousCost = best_value;
                double relative_gap = (previousCost - lp.getCost()) / previousCost * 100;
                updateBestValue(lp.getCost());
                best_route.clear();
                best_crew.clear();
                best_route.addAll(lp.getReliefRoutes());
                best_crew.addAll(lp.getCrewRoutes());
                route_values.clear();
                route_values.addAll(lp.getRouteValues());
                route_waits.clear();
                route_waits.addAll(lp.getRouteWaites());

                best_node = node;
                hasFoundFeasibleSolution  = true;
                if(isMultiThread){
                    System.out.println("\t\tUpdating the UB(T#"+ucea_constraintl.id+");" + best_value + " ("+relative_gap+"%)");
                } else {
                    System.out.println("\t\tUpdating the UB("+node.getDepth()+");" + best_value + " ("+relative_gap+"%)");
                }
            }
            lp.clear();
        } else {


            node.setCost(lp.getCost());

            if(Utility.algo == 6){
                if(node.getDepth() >= Constants.MAX_DEPTH)
                    return;
            }

            BT branch = new BT();


            ArrayList<BNode> nodes = null;
            if (branchingMode == BranchingMode.PSEUDOCOST) {
                nodes = new PseudocostBranching(pseudocostTable).branch(lp, node);
            }
            if (nodes == null) {
                double[][] parentTowerArcValues =
                        new double[data.getNodeNumber()][data.getNodeNumber() + 1];
                double[][] parentCrewArcValues =
                        new double[data.getTasks()][data.getTasks() + 1];
                lp.getMap(parentTowerArcValues, false);
                lp.getMap(parentCrewArcValues, true);
                nodes = branch.branch(lp, node,isThreadFiveOrGreater);
                attachDefaultArcPseudocostMetadata(
                        nodes, node, parentTowerArcValues, parentCrewArcValues);
            }
            queue.addAll(nodes);
            lp.clear();
        }
    }

    private void recordGlobalLowerBound(PriorityQueue<BNode> queue, int tot_nodes) {
        if (queue.isEmpty()) return;
        int id = ucea_constraintl == null ? 0 : ucea_constraintl.id;
        double globalLowerBound = queue.peek().getCost();
        boolean stalled = lowerBoundProgressTracker.record(globalLowerBound);
        if (stalled && branchingMode == BranchingMode.DEFAULT) {
            branchingMode = BranchingMode.PSEUDOCOST;
            System.out.println("Switching T("+id + ") branching mode to PSEUDOCOST after "
                    + lowerBoundProgressTracker.getIterationCount()
                    + " processed nodes; global LB="
                    + lowerBoundProgressTracker.getLastRecordedGlobalLowerBound());
        }
    }

    private void updatePseudocostObservation(BNode node, double childLpBound) {
        if (!node.hasPseudocostBranchMetadata() || node.isPseudocostUpdateDone()) return;

        double objectiveGain = childLpBound - node.getParentLpBound();
        if (objectiveGain < 0.0 && objectiveGain > -1e-6) objectiveGain = 0.0;
        if (objectiveGain < 0.0 || !Double.isFinite(objectiveGain)) return;

        double observedPseudocost = objectiveGain / node.getBranchDelta();
        if (!Double.isFinite(observedPseudocost)) return;

        pseudocostTable.observe(
                node.getBranchedVariableId(),
                node.getBranchDirection(),
                observedPseudocost);
        node.markPseudocostUpdateDone();
    }

    private void attachDefaultArcPseudocostMetadata(
            ArrayList<BNode> children,
            BNode parent,
            double[][] towerArcValues,
            double[][] crewArcValues
    ) {
        boolean crew = false;
        int from = -1;
        int to = -1;

        for (BNode child : children) {
            String branch = child.getBranchStrig();
            if (branch == null) continue;
            var towerMatcher = TOWER_ARC_BRANCH_PATTERN.matcher(branch);
            var crewMatcher = CREW_ARC_BRANCH_PATTERN.matcher(branch);
            if (towerMatcher.find()) {
                from = Integer.parseInt(towerMatcher.group(1));
                to = Integer.parseInt(towerMatcher.group(2));
                break;
            }
            if (crewMatcher.find()) {
                crew = true;
                from = Integer.parseInt(crewMatcher.group(1));
                to = Integer.parseInt(crewMatcher.group(2));
                break;
            }
        }
        if (from < 0) return;

        double value = crew ? crewArcValues[from][to] : towerArcValues[from][to];
        if (!HF.is_fractionalHP(value)) return;
        PseudocostBranching.attachArcMetadata(
                children,
                parent.getCost(),
                PseudocostBranching.arcVariableId(crew, from, to),
                value,
                crew,
                from,
                to);
    }


    private boolean hasPruned;
    private boolean prune(BNode node) {
        if(node.getDepth() > 0 ){
            if ((node.getCost() > best_value - 1e-6) || (node.getCost() > initialValue - 1e-6)) {
                if(initialValue < 1e-6)
                    throw new IllegalArgumentException("0 UB!");
                if(!hasPruned){
                    System.out.println("Pruning the node due to high cost:"+node.getCost()+">"+Math.min(best_value,initialValue));
                    hasPruned = true;
                }
                node.setCostWhenPruned(node.getCost());
                return true;
            }
            if(isMultiThread){
                if ((node.getCost() > sharedBounds.getUB() - 1e-6)) {
                    if(!hasPruned) {
                        System.out.println();
                        System.out.println("\t\tPruning thread T"+ucea_constraintl.id +" due to high cost based on global UB at T " +sharedBounds.getBestID() +":"
                                +node.getCost()+">"+sharedBounds.getUB());
                        System.out.println();
                        hasPruned = true;
                    }
                    node.setCostWhenPruned(node.getCost());
                    return true;
                }
            }
        }
        return false;
    }



    public boolean hasTimedOut() {
        return hasTimedOut;
    }

    private void initialize(PriorityQueue<BNode> queue) {
        BNode node = new BNode();

        if(ucea_constraintl!=null)
            node.setICEAid(ucea_constraintl.id);


        node.setFleetLB(0);
        node.setFleetUB(data.getTowerNumber());



        node.setCrewLB(0);
        node.setCrewUB(data.getCrewNumber());
        node.createCx();
        int[][][] feasibleSchedule_arc = new int[2][][];
        for (int k = 0; k < 2; k++) {
            if(k==0){
                //towers
                feasibleSchedule_arc[0] = new int[data.getNodeNumber()][data.getNodeNumber()+1];
            } else {
                //crews
                feasibleSchedule_arc[1] = new int[data.getTasks()][data.getTasks()+1];
            }
        }
        node.setFeasibleSchedule_arc(feasibleSchedule_arc);

        node.initializePackingBounds(data.getNodeNumber() + 1);

        node.setABounds(0,0,data.getTowerNumber());

        if(BendersCuts.toggle){
            updateBestValue(initialValue);
        } else {
            updateBestValue(1e19);
        }
        best_route = new ArrayList<>();
        best_crew = new ArrayList<>();
        route_values = new ArrayList<>();
        route_waits = new ArrayList<>();


        if(initialCrew != null){
            node.setInitialCrewSolution(initialCrew);
        }



        if(BendersCuts.toggle){
            for (int i = 1; i < data.getNodeNumber(); i++) {
                node.setPackingBounds(i,BendersCuts.currentYs[i], BendersCuts.currentYs[i]);
            }
        }


        if(Constants.FIX_CREW_ROUTES){
            try {
                System.out.println("Applying FIXED CREW ROUTES");

                String pathLink = "fixCrewSolutions";
                if(Constants.ROBUST) {
                    int gamma = Constants.BUDGET;
                    int alpha = (int) (100 * Constants.ALPHA);
                    if(Constants.COST_OF_PRIORITY) {
                        String opener = "summary/robust/priority/crewFirst/g"+gamma+"/0"+alpha;
                        pathLink = opener+"/solutions";
                    } else {
                        String opener = "summary/robust/g"+gamma+"/0"+alpha;
                        pathLink = opener+"/crewFirst/solutions";
                    }
                } else {
                    if(Constants.COST_OF_PRIORITY){
                        pathLink = "summary/priority/crewFirst/solutions";
                    } else {
                        pathLink = "summary/crewOnly/solutions";
                    }
                }


                Path fixedSolutionPath = findFixedCrewSolution(
                        Path.of(pathLink),
                        LocalWriter.filename
                );

                applyFixedCrewRoutes(fixedSolutionPath, node);

            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

        if(Constants.FIX_CREW_AND_TOWER_ROUTES){
            try {
                System.out.println("Applying FIXED CREW ROUTES");

                Path fixedSolutionPath = findFixedCrewSolution(
                        Path.of("fixCrewSolutions"),
                        LocalWriter.filename
                );

                applyFixedCrewRoutes(fixedSolutionPath, node);
                applyFixedTowerRoutes(fixedSolutionPath, node);

            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

        queue.offer(node);
    }

    public static Path findFixedCrewSolution(Path solutionDir, String instanceFilename)
            throws IOException {

        String filename = Path.of(instanceFilename)
                .getFileName()
                .toString();

        String instanceName = filename.contains(".")
                ? filename.substring(0, filename.lastIndexOf('.'))
                : filename;

        Pattern pattern = Pattern.compile(
                Pattern.quote(instanceName) + "-\\d+-\\d+\\.txt"
        );

        List<Path> matches;

        try (Stream<Path> files = Files.list(solutionDir)) {
            matches = files
                    .filter(Files::isRegularFile)
                    .filter(path -> pattern.matcher(path.getFileName().toString()).matches())
                    .sorted(Comparator.comparing(path -> path.getFileName().toString()))
                    .toList();
        }

        if (matches.isEmpty()) {
            throw new IllegalArgumentException(
                    "No fixed crew solution found for instance " + instanceFilename
                            + " in folder " + solutionDir
            );
        }

        if (matches.size() > 1) {
            throw new IllegalArgumentException(
                    "Multiple fixed crew solutions found for instance " + instanceFilename
                            + ": " + matches
            );
        }

        return matches.get(0);
    }

    public void applyFixedCrewRoutes(Path filePath, BNode node) throws IOException {

        for (String line : Files.readAllLines(filePath)) {

            line = line.trim();

            // Skip empty lines if any
            if (line.isEmpty()) {
                continue;
            }

            if(line.equals("Technician Plans")){
                continue;
            }
            if(line.equals("Tower Routes")){
                break;
            }

            // Remove "[" and "]"
            line = line.replace("[", "").replace("]", "").trim();

            String[] tokens = line.split(",");

            ArrayList<Integer> route = new ArrayList<>();

            for (String token : tokens) {
                route.add(Integer.parseInt(token.trim()));
            }

            if (route.size() < 2) {
                continue;
            }

            int from = route.get(0);

            for (int i = 1; i < route.size(); i++) {
                int to = route.get(i);

                Arc arc = new Arc(from, to);
                node.setFeasibleArcValue(arc, 1, 1, true);

                from = to;
            }
        }
    }
    public void applyFixedTowerRoutes(Path filePath, BNode node) throws IOException {

        boolean hasFound = false;
        int depotEnd = data.getDepotEnd(false);
        for (String line : Files.readAllLines(filePath)) {



            line = line.trim();

            if(!hasFound){
                if(!line.equals("Tower Routes"))
                    continue;
                else {
                    hasFound = true;
                    continue; //skip the line "Tower Routes"
                }
            }



            // Skip empty lines if any
            if (line.isEmpty()) {
                continue;
            }


            // Remove "[" and "]"
            line = line.replace("[", "").replace("]", "").trim();

            String[] tokens = line.split(",");

            ArrayList<Integer> route = new ArrayList<>();

            for (String token : tokens) {
                route.add(Integer.parseInt(token.trim()));
            }

            if (route.size() < 2) {
                continue;
            }

            int from = route.get(0);

            for (int i = 1; i < route.size(); i++) {
                int to = route.get(i);

                Arc arc = new Arc(from, to);
                node.setFeasibleArcValue(arc, 1, 1, false);

                if(to != depotEnd)
                    node.setPackingBounds(to,1.,1.);

                from = to;
            }
        }
    }
    private void updateBestValue(double v) {
        best_value = v;
        if(isMultiThread){
            sharedBounds.updateBest(best_value, ucea_constraintl.id);
        }
    }


    private void printSolution() {
        boolean noSolution =
                best_crew == null
                        || best_route == null
                        || best_crew.isEmpty() && best_route.isEmpty();

        if (best_value >= 1000000 || noSolution) {
            if (isMultiThread) {
                System.out.print("T" + ucea_constraintl.id);
                System.out.println(" has failed to find a better solution.");
            } else {
                System.out.println("\nThe algorithm has failed to find a better solution.");
                System.out.println("\nCost:" + best_value);
            }
            return;
        }

        if(isMultiThread)
            System.out.print("T"+ucea_constraintl.id+"\t");
        System.out.print("B");
        System.out.print("\t\t");
        System.out.print("E");
        System.out.print("\t\t");
        System.out.print("S");
        System.out.print("\t\t");
        System.out.print("T");
        System.out.print("\t\t");
        System.out.print("S");
        System.out.print("\t\t");
        System.out.print("O");
        System.out.print("\t\t");
        System.out.print("L");
        System.out.print("\t\t");
        System.out.print("U");
        System.out.print("\t\t");
        System.out.print("T");
        System.out.print("\t\t");
        System.out.print("I");
        System.out.print("\t\t");
        System.out.print("O");
        System.out.print("\t\t");
        System.out.print("N");
        System.out.println();

        System.out.println("Cost:" + best_value);

        double totTower = 0.;
        for (int i = 0; i < best_route.size(); i++) {
            totTower += route_values.get(i);
        }
        System.out.println("Tower Routes(" + totTower + "):");

        double[] towerArrivalTimes = new double[data.getNodeNumber() + 1];
        double[] manualServiceDuration = new double[data.getNodeNumber()];
        HashSet<ArrayList<Integer>> hashSet = new HashSet<>();
        for (int i = 0; i < best_route.size(); i++) {
            Route route = best_route.get(i);
            double v = route_values.get(i);
            System.out.print("\t" + v + ":");
            System.out.print(route.getPattern().toString());
            System.out.println(route_waits.get(i));

            hashSet.add(route.getPattern());
            if (!route_waits.get(i).equals(route.getWaitBooleans()))
                throw new IllegalArgumentException("wait patternes do not match");

            //Only for elementary solutions
            Msg.todo();
            HashMap<Integer, Double> map = route.computeCostTower(route.getPattern(), v);


            for (Integer key : map.keySet())
                manualServiceDuration[key] += map.get(key);

            for (int j = 1; j < route.getPattern().size() - 1; j++) {
                int key = route.getPattern().get(j);
                towerArrivalTimes[key] += route.getArrivals().get(j) * v;
            }

        }

        System.out.println("Tower Routes:");
        for(ArrayList<Integer> list : hashSet)
            System.out.println("\t"+list.toString());

        double[] repairTimes = new double[data.getTasks()];
        System.out.println("Repair crews:");
        for (Route route : best_crew) {
            double v = 1.0;
//            System.out.print("\t" + v + ":");
            System.out.println("\t" +route.getPattern().toString());

            for (int j = 1; j < route.getPattern().size() - 1; j++) {
                int key = route.getPattern().get(j);
                repairTimes[key] += route.getArrivals().get(j) * v;
            }
        }

        double[] completionTimes = new double[data.getNodeNumber()];
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double max = 0;
            for(Integer k : data.getZone2tasks()[i])
                max = Math.max(repairTimes[k],max);
            completionTimes[i] = max;
        }
    }



    public double getObjValue() {
        return best_value;
    }


    public ArrayList<Route> getBestCrew() {
        return best_crew;
    }



    public ArrayList<Route> getBestTower() {
        return best_route;
    }

    private ArrayList<Route> initialCrew;
    private ArrayList<Route> initialTower;
    private ArrayList<Double> initialRouteValues;
    private ArrayList<ArrayList<Boolean>> initialWaits;
    private double initialValue;

    public void setInitialSolution(ArrayList<Route> crew, ArrayList<Route> tow, ArrayList<Double> inVals, ArrayList<ArrayList<Boolean>> inWaits, double inV) {
        this.initialCrew = crew;
        this.initialTower = tow;
        initialRouteValues = inVals;
        initialWaits = inWaits;
        initialValue = inV;
    }

    public ArrayList<Double> getRouteValues() {
        return route_values;
    }

    public ArrayList<ArrayList<Boolean>> getWaits() {
        return route_waits;
    }

    public double getElapsed() {
        return elapsed;
    }


    public int getNodeNumber() {
        return tot_nodes;
    }

    public void setInitialUB(double val) {
        initialValue = val;
    }





    private SharedBounds sharedBounds;
    public void setSharedBound(SharedBounds _sharedBounds) {
        sharedBounds = _sharedBounds;
    }

    private ICEAconstraint ucea_constraintl;
    public void setICEAconstraint(ICEAconstraint extraConstraint) {
        ucea_constraintl = extraConstraint;
    }

    private boolean isExtraBound;

    public void setExtraBound(){
        isExtraBound = true;
    }

    private double bestLB;
    public double getbestLB() {
        if(bestLB < 1e-6)
            return 999;
        return bestLB;
    }

    private double rootLB;
    public double getRootLB() {
        return rootLB;
    }




    public boolean hasBestSolution() {
        return best_crew != null
                && best_route != null
                && route_values != null
                && route_waits != null
                && (!best_crew.isEmpty() || !best_route.isEmpty());
    }


}
