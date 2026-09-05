package problem.milp;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import lib.*;
import problem.BP.BendersCuts;
import problem.Constants;
import problem.graph.Data;

import java.util.*;

public class MILPCplex {

    private IloCplex cplex;
    private IloObjective obj;
    private Data data;

    private IloNumVar[][][] varXtowers, varXcrews;
    private IloNumVar[][] varY, varGamma, varZ, varOmega, varExtra, varSchedule;
    private IloNumVar[] varDelta, varLambda;


    private IloRange limRange;
    private IloNumVar[] varA; private IloNumVar varEta;

    /*
     * Optional fixed two-index routes used only when replaying an already
     * optimal solution for objective-value statistics.  The ordinary
     * no-argument buildAndSolve2index() path leaves these fields null.
     */
    private List<List<Integer>> fixedTechnicianPlans2index;
    private List<List<Integer>> fixedTowerRoutes2index;
    private double[] objectiveValueStatistics2index;

    private static final int OBJECTIVE_STATISTICS_HORIZON = 600;
    private static final int OBJECTIVE_STATISTICS_INTERVAL = 100;

    /**
     * When the upper-bounding heuristic is enabled, tower routes may move only
     * between zones in non-increasing order of zone weight. Depot arcs are not
     * restricted because the depot indices do not represent zones.
     */
    private void applyTowerWeightOrdering(IloNumVar[][] towerArcVars) throws IloException {
        // A saved route must be replayed exactly, independently of an optional
        // route-generation heuristic that may currently be enabled.
        if (fixedTechnicianPlans2index != null)
            return;

        if (!Constants.UPPER_BOUNDING_HEURISTIC)
            return;

        double[] zoneWeights = data.getWeights();
        for (int i = 1; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber(); j++) {
                if (i != j && zoneWeights[i] < zoneWeights[j])
                    towerArcVars[i][j].setUB(0.);
            }
        }
    }


    /**
     * Replays a saved optimal two-index solution and returns the cumulative
     * weighted interruption objective at times 0, 100, ..., 600.
     *
     * The saved route arcs are fixed by setting the lower bound of the
     * corresponding xR/xT variables to 1.  All timing/service variables remain
     * continuous and are determined by the MILP, so the reported values use
     * exactly the same timing logic as buildAndSolve2index().
     */
    public double[] buildAndSolve2index(
            List<List<Integer>> technicianPlans,
            List<List<Integer>> towerRoutes) throws IloException {

        if (technicianPlans == null || technicianPlans.isEmpty()) {
            throw new IllegalArgumentException(
                    "technicianPlans must contain at least one route.");
        }

        this.fixedTechnicianPlans2index = copyRoutes(technicianPlans);
        this.fixedTowerRoutes2index = towerRoutes == null
                ? Collections.emptyList()
                : copyRoutes(towerRoutes);
        this.objectiveValueStatistics2index = null;

        try {
            buildAndSolve2index();

            if (objectiveValueStatistics2index == null) {
                throw new IllegalStateException(
                        "The fixed-route MILP did not produce an objective-statistics solution.");
            }

            return Arrays.copyOf(
                    objectiveValueStatistics2index,
                    objectiveValueStatistics2index.length);
        } finally {
            this.fixedTechnicianPlans2index = null;
            this.fixedTowerRoutes2index = null;
            this.objectiveValueStatistics2index = null;
        }
    }

    private static List<List<Integer>> copyRoutes(
            List<List<Integer>> routes) {

        List<List<Integer>> result = new ArrayList<>();
        for (List<Integer> route : routes) {
            if (route == null) {
                throw new IllegalArgumentException("A route must not be null.");
            }
            result.add(new ArrayList<>(route));
        }
        return result;
    }


    public void buildAndSolve2index() throws IloException {


        this.cplex = new IloCplex();
        cplex.setParam(IloCplex.Param.TimeLimit, Utility.bPCTiLim);
        this.data = Data.getInstance();
        IloNumVar[][] varXtowers = new IloNumVar[data.getNodeNumber()][data.getNodeNumber()+1];
        IloNumVar[][] varXcrews = new IloNumVar[data.getTasks()][data.getTasks()+1];


        for (int i = 0; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber()+1; j++) {
                if(i==0&&j==data.getDepotEnd(false))
                    continue;
                if(i!=j) {
                    varXtowers[i][j] = cplex.boolVar("xT_"+ HF.getVarName(List.of(i,j)));
                }
            }
        }

        applyTowerWeightOrdering(varXtowers);

        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks()+1; j++) {
                if(i==0&&j==data.getDepotEnd(true))
                    continue;
                if(i!=j) {
                    if(Constants.RELAX_MIP)
                        varXcrews[i][j] = cplex.numVar(0.,1.,"xR_"+ HF.getVarName(List.of(i,j)));
                    else
                        varXcrews[i][j] = cplex.boolVar("xR_"+ HF.getVarName(List.of(i,j)));
                }
            }
        }

        IloNumVar[] varY = new IloNumVar[data.getNodeNumber()];
        IloNumVar[] varExtra = new IloNumVar[data.getNodeNumber()];
        IloNumVar[] varGamma = new IloNumVar[data.getNodeNumber()+1];
        IloNumVar[] varZ = new IloNumVar[data.getNodeNumber()];
        IloNumVar[] varOmega = new IloNumVar[data.getTasks()+1];
        for (int i = 0; i < data.getNodeNumber(); i++) {
            varExtra[i] = cplex.numVar(0.,Double.MAX_VALUE,"e_"+ HF.getVarName(List.of(i)));
            varGamma[i] = cplex.numVar(0.,Double.MAX_VALUE,"gT_"+ HF.getVarName(List.of(i)));
            varZ[i] = cplex.numVar(0.,Double.MAX_VALUE,"zT_"+ HF.getVarName(List.of(i)));
            if(i==0){
                varZ[i].setUB(0);
                varGamma[i].setUB(0.);
                continue;
            }
            if(Constants.RELAX_MIP) {
                if(Constants.KEEP_INTEGER_Ys || BendersCuts.toggle)
                    varY[i] = cplex.boolVar("yT_"+ HF.getVarName(List.of(i)));
                else
                    varY[i] = cplex.numVar(0.,1.,"yT_"+ HF.getVarName(List.of(i)));
            } else
                varY[i] = cplex.boolVar("yT_"+ HF.getVarName(List.of(i)));
        }

        varGamma[data.getDepotEnd(false)] = cplex.numVar(0.,Double.MAX_VALUE,"gT_"+ HF.getVarName(List.of(data.getDepotEnd(false))));

        for (int i = 0; i < data.getTasks()+1; i++) {
            varOmega[i] = cplex.numVar(0.,Double.MAX_VALUE,"oC_"+ HF.getVarName(List.of(i)));
            if(i==0) varOmega[0].setUB(0);
        }

        IloNumVar[] varDelta = new IloNumVar[data.getNodeNumber()];
        for (int i = 1; i < data.getNodeNumber(); i++) {
            varDelta[i] = cplex.numVar(0.,Double.MAX_VALUE,"delta_"+ HF.getVarName(List.of(i)));
        }

        {
            IloNumExpr expr = cplex.numExpr();
            for (int i = 1; i < data.getTasks(); i++) expr = cplex.sum(expr,varXcrews[0][i]);
            cplex.addEq(expr,data.getCrewNumber(),"crewFromDepot");
        }
        {
            IloNumExpr expr = cplex.numExpr();
            for (int i = 1; i < data.getTasks(); i++) expr = cplex.sum(expr,varXcrews[i][data.getDepotEnd(true)]);
            cplex.addEq(expr,data.getCrewNumber(),"crewToDepot");
        }

        {
            for (int i = 1; i < data.getTasks(); i++) {
                IloNumExpr expr = cplex.numExpr();
                for (int j = 1; j < data.getTasks()+1; j++) if(i!=j) expr = cplex.sum(expr,varXcrews[i][j]);
                for (int j = 0; j < data.getTasks(); j++) if(i!=j) expr = cplex.sum(expr,cplex.prod(-1.,varXcrews[j][i]));
                cplex.addEq(expr,0.,"flow_"+i);
            }
        }

        for (int i = 1; i < data.getTasks(); i++) {
            IloNumExpr expr = cplex.numExpr();
            for (int j = 1; j < data.getTasks()+1; j++) if(i!=j) expr = cplex.sum(expr,varXcrews[i][j]);
            cplex.addEq(expr,1.,"visit_"+i);
        }


        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks()+1; j++) {
                if(i==0 && j == data.getDepotEnd(true))
                    continue;
                if(i!=j){
                    double m1 = data.getHorizon();
                    IloNumExpr expr = cplex.numExpr();
                    expr = cplex.sum(expr,varOmega[i]);
                    expr = cplex.sum(expr,cplex.prod(-1.,varOmega[j]));
                    expr = cplex.sum(expr,cplex.prod(
                            m1+ data.getCrewTravelTimeMatrix()[i][j]+data.getServiceTimeMatrix()[j]
                            , varXcrews[i][j]));
                    cplex.addGe(m1
                            ,expr,"taskTime_"+HF.getVarName(List.of(i,j)));
                }
            }
        }

        {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varOmega[data.getDepotEnd(true)]);
            cplex.addLe(expr,data.getHorizon(),"crewShift");
        }
        for (int i = 1; i < data.getNodeNumber(); i++) {
            for(Integer q : data.getZone2tasks()[i]){
                IloNumExpr expr = cplex.numExpr();
                expr = cplex.sum(expr,varDelta[i]);
                expr = cplex.sum(expr,cplex.prod(-1.,varOmega[q]));
                cplex.addGe(expr,0.,"deltaToMaxTaskTime_"+HF.getVarName(List.of(i,q)));
            }
        }


        for (int i = 1; i < data.getNodeNumber(); i++) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varY[i]);
            for (int j = 0; j < data.getNodeNumber(); j++) if(i!=j) expr = cplex.sum(expr,cplex.prod(-1., varXtowers[j][i]));
            cplex.addEq(expr,0.,"linkZone_"+HF.getVarName(List.of(i)));
        }

        {
            IloNumExpr expr = cplex.numExpr();
            for (int i = 1; i < data.getNodeNumber(); i++) expr = cplex.sum(expr,varXtowers[0][i]);
            cplex.addEq(expr,data.getTowerNumber(),"towerFromDepot");
        }
        {
            IloNumExpr expr = cplex.numExpr();
            for (int i = 1; i < data.getNodeNumber(); i++) expr = cplex.sum(expr,varXtowers[i][data.getDepotEnd(false)]);
            cplex.addEq(expr,data.getTowerNumber(),"towerToDepot");
        }
        {
            for (int i = 1; i < data.getNodeNumber(); i++) {
                IloNumExpr expr = cplex.numExpr();
                for (int j = 1; j < data.getNodeNumber()+1; j++) if(i!=j) expr = cplex.sum(expr,varXtowers[i][j]);
                for (int j = 0; j < data.getNodeNumber(); j++) if(i!=j) expr = cplex.sum(expr,cplex.prod(-1.,varXtowers[j][i]));
                cplex.addEq(expr,0.,"flowT_"+i);
            }
        }

        for (int i = 0; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber()+1; j++) {
                if(i==0&&j==data.getDepotEnd(false))
                    continue;
                if(i!=j){
                    double min = Double.MAX_VALUE;
                    if(i==0)
                        min = 0;
                    else {
                        for(Integer q : data.getZone2tasks()[i])
                            min = Math.min(min,data.getCrewTravelTimeMatrix()[q][data.getDepotEnd(true)]);

                    }
                    double delta;
                    if(j==data.getDepotEnd(false))
                        delta = data.getTowerTravelTimeMatrix()[i][j];
                    else
                        delta = data.getTowerTravelTimeMatrix()[i][j]+data.getPositionTimeMatrix()[j];
                    double m = data.getHorizon() - min;
                    IloNumExpr expr = cplex.numExpr();
                    expr = cplex.sum(expr,varGamma[i]);
                    expr = cplex.sum(expr,cplex.prod(-1.,varGamma[j]));
                    expr = cplex.sum(expr,varZ[i]);
                    expr = cplex.sum(expr,cplex.prod(m + delta
                            , varXtowers[i][j]));
                    cplex.addGe(m
                            ,expr,"(TowerTimes)"+HF.getVarName(List.of(i,j)));
                }
            }
        }

        for (int i = 1; i < data.getNodeNumber(); i++) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varGamma[i]);
            expr = cplex.sum(expr,varZ[i]);
            expr = cplex.sum(expr,cplex.prod(-1.,varExtra[i]));
            cplex.addLe(expr,0.,"serviceUB_"+HF.getVarName(List.of(i)));
        }
        if(BendersCuts.toggle){
            cplex.setOut(null);
            varEta = cplex.numVar(0.,Double.MAX_VALUE,"eta");
            {
                IloNumExpr expr = cplex.numExpr();
                for (int i = 1; i < data.getNodeNumber(); i++){
                    expr = cplex.sum(expr,cplex.prod(data.getWeights()[i],varDelta[i]));
                    expr = cplex.sum(expr,cplex.prod(-1.*data.getWeights()[i],varZ[i]));
                }
                expr = cplex.sum(expr,cplex.prod(-1.,varEta));
                cplex.addGe(0.,expr);
            }

            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varEta);
            obj = cplex.addMinimize(expr);


        } else {
            IloNumExpr expr = cplex.numExpr();
            for (int i = 1; i < data.getNodeNumber(); i++){
                expr = cplex.sum(expr,cplex.prod(data.getWeights()[i],varDelta[i]));
                expr = cplex.sum(expr,cplex.prod(-1.*data.getWeights()[i],varZ[i]));
            }
            obj = cplex.addMinimize(expr);
        }


        for (int i = 1; i < data.getNodeNumber(); i++) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varExtra[i]);
            double min = Double.MAX_VALUE;
            for(Integer q : data.getZone2tasks()[i]) min = Math.min(min,data.getCrewTravelTimeMatrix()[q][data.getDepotEnd(true)]);
            double m = data.getHorizon() - min;

            expr = cplex.sum(expr,cplex.prod(-1.*m,varY[i]));
            cplex.addLe(expr,0.,"(E1)"+HF.getVarName(List.of(i)));
        }
        for (int i = 1; i < data.getNodeNumber(); i++) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varExtra[i]);
            expr = cplex.sum(expr,cplex.prod(-1.,varDelta[i]));
            cplex.addLe(expr,0.,"(E2)"+HF.getVarName(List.of(i)));
        }
        for (int i = 1; i < data.getNodeNumber(); i++) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varExtra[i]);

            double min = Double.MAX_VALUE;
            for(Integer q : data.getZone2tasks()[i]) min = Math.min(min,data.getCrewTravelTimeMatrix()[q][data.getDepotEnd(true)]);
            double m = data.getHorizon() - min;
            expr = cplex.sum(expr,cplex.prod(-1.*m,varY[i]));

            expr = cplex.sum(expr,cplex.prod(-1.,varDelta[i]));
            cplex.addGe(expr,-1.*data.getHorizon(),"(E3)"+HF.getVarName(List.of(i)));
        }


        /*
         * Statistics/replay mode: the model is now fully constructed.
         * Before solve(), force every arc appearing in the saved optimal routes.
         */
        if (fixedTechnicianPlans2index != null) {
            fixSaved2indexRoutes(
                    varXcrews,
                    varXtowers,
                    fixedTechnicianPlans2index,
                    fixedTowerRoutes2index);
        }

        if(cplex.solve()){
            System.out.println("MIP solved successfully:"+cplex.getObjValue());

            if (fixedTechnicianPlans2index != null) {
                objectiveValueStatistics2index =
                        collectObjectiveValueStatistics(
                                varDelta,
                                varGamma,
                                varZ);
            }


            for (int i = 0; i < data.getTasks(); i++) {
                for (int j = 1; j < data.getTasks()+1; j++) {
                    if(i==0&&j==data.getDepotEnd(true))
                        continue;
                    if(i!=j){
                        double v = cplex.getValue(varXcrews[i][j]);
                        if(v > 1e-6)
                            System.out.println("Crew"+"["+
                                    i+"->"+j+":"+v+"]");

                    }
                }
            }

            for (int i = 0; i < data.getNodeNumber(); i++) {
                for (int j = 1; j < data.getNodeNumber()+1; j++) {
                    if(i==0&&j==data.getDepotEnd(false))
                        continue;
                    if(i!=j){
                        double v = cplex.getValue(varXtowers[i][j]);
                        if(v > 1e-6)
                            System.out.println("Tower"+"["+
                                    i+"->"+j+":"+v+"]");

                    }
                }
            }

            System.out.println("Zone Repair Times by Solver:");
            double[] repairTimes = new double[data.getNodeNumber()];
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v1= cplex.getValue(varDelta[i]);
                repairTimes[i] = v1;
//                System.out.println("\t"+i+":"+v1 + "*" + data.getWeights()[i]+"=("+v1*data.getWeights()[i]+")");
                System.out.println("\t"+i+":"+v1);
            }

            System.out.println("Task Completion Times by Solver:");
            for (int i = 1; i < data.getTasks(); i++) {
                double v1= cplex.getValue(varOmega[i]);
                System.out.println("\t"+i+":"+v1);
            }

            double[] service = new double[data.getNodeNumber()+1];
            System.out.println("Tower Service Durations:");
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v = cplex.getValue(varZ[i]);
                if(v > 1e-6){
                    service[i] = v;
                    System.out.println("\t"+i +":"+v);
//                    System.out.println("\t"+i +":"+v+ "*" + data.getWeights()[i]+"=("+v*data.getWeights()[i]+")");
                }
            }

            double totalYs = 0;
            double[] as = new double[data.getNodeNumber()];
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v = cplex.getValue(varY[i]);
                if(v > 1e-6){
                    as[i] = v;
                    totalYs += v;
                }
            }
            System.out.println("Tower ys:" + totalYs);
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v = as[i];
                if(v > 1e-6){
                    System.out.println("\tY:"+i+":"+v);
                }
            }

            System.out.println("Tower arrivals:");
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v = cplex.getValue(varGamma[i]);
                if(v > 1e-6){
                    System.out.println("\tGamma:"+i+":"+v);
                }
            }
            double[] manual_tower_arrival_times = new double[data.getNodeNumber()+1];
            double[] manual_crew_arrival_times = new double[data.getTasks()+1];
            if(!Constants.RELAX_MIP){
                double[] zone2arrival = new double[data.getNodeNumber()+1];
                System.out.println("Tower Routes:");
                {
                    // Step 1: Build the adjacency list
                    Map<Integer, List<Integer>> nextNodeMap = new HashMap<>();
                    for (int i = 0; i < data.getNodeNumber(); i++) {
                        for (int j = 1; j < data.getNodeNumber()+1; j++) {
                            if (i == j || (i == 0 && j == data.getDepotEnd(false))) continue;
                            double v1 = cplex.getValue(varXtowers[i][j]);
                            if (v1 > 1e-6) {
//                                System.out.println("x_" + i + "_" + j + ":" + v1);
                                nextNodeMap.computeIfAbsent(i, k -> new ArrayList<>()).add(j);
                            }
                        }
                    }
                    // Step 2: Reconstruct routes
                    List<List<Integer>> routes = new ArrayList<>();
                    Set<Integer> visited = new HashSet<>();

                    List<Integer> starts = nextNodeMap.getOrDefault(0, new ArrayList<>());
                    for (int start : starts) {
                        if (visited.contains(start)) continue;

                        List<Integer> route = new ArrayList<>();
                        route.add(0);
                        int current = start;

                        while (true) {
                            route.add(current);
                            visited.add(current);
                            List<Integer> nextList = nextNodeMap.getOrDefault(current, new ArrayList<>());

                            Integer next = null;
                            for (int candidate : nextList) {
                                if (!visited.contains(candidate)) {
                                    next = candidate;
                                    break;
                                }
                            }

                            if (next == null || next == data.getDepotEnd(false)) {
                                if (next != null) route.add(next);
                                else route.add(data.getDepotEnd(false));
                                break;
                            }

                            current = next;
                        }

                        routes.add(route);
                    }

                    // Step 3: Print routes and compute travel times
                    for (int r = 0; r < routes.size(); r++) {
                        List<Integer> route = routes.get(r);
                        double totalTime = 0.0;
                        for (int k = 0; k < route.size() - 1; k++) {
                            int from = route.get(k);
                            int to = route.get(k + 1);
                            zone2arrival[to] = totalTime + data.getTowerTravelTimeMatrix()[from][to] + data.getPositionTimeMatrix()[to];
                            totalTime += data.getTowerTravelTimeMatrix()[from][to] + data.getPositionTimeMatrix()[to] + service[to];
                        }
                        System.out.println("Tower Route " + (r + 1) + ": " + route + " | Total travel time: " + totalTime);
                    }

                    for (int i = 1; i < data.getNodeNumber(); i++) {
                        if(cplex.getValue(varY[i]) > 1e-6)
                            System.out.println("Manual Zone Arrival at "+i+":"+zone2arrival[i]);
                    }
                    manual_tower_arrival_times = zone2arrival;
                }
                System.out.println("Crew Routes:");
                {

                    // Step 1: Build the adjacency list
                    Map<Integer, List<Integer>> nextNodeMap = new HashMap<>();
                    for (int i = 0; i < data.getTasks(); i++) {
                        for (int j = 1; j < data.getTasks()+1; j++) {
                            if (i == j || (i == 0 && j == data.getDepotEnd(true))) continue;
                            double v1 = cplex.getValue(varXcrews[i][j]);
                            if (v1 > 1e-6) {
//                                System.out.println("x_" + i + "_" + j + ":" + v1);
                                nextNodeMap.computeIfAbsent(i, k -> new ArrayList<>()).add(j);
                            }
                        }
                    }
                    // Step 2: Reconstruct routes
                    List<List<Integer>> routes = new ArrayList<>();
                    Set<Integer> visited = new HashSet<>();

                    List<Integer> starts = nextNodeMap.getOrDefault(0, new ArrayList<>());
                    for (int start : starts) {
                        if (visited.contains(start)) continue;

                        List<Integer> route = new ArrayList<>();
                        route.add(0);
                        int current = start;

                        while (true) {
                            route.add(current);
                            visited.add(current);
                            List<Integer> nextList = nextNodeMap.getOrDefault(current, new ArrayList<>());

                            Integer next = null;
                            for (int candidate : nextList) {
                                if (!visited.contains(candidate)) {
                                    next = candidate;
                                    break;
                                }
                            }

                            if (next == null || next == data.getDepotEnd(true)) {
                                if (next != null) route.add(next);
                                else route.add(data.getDepotEnd(true));
                                break;
                            }

                            current = next;
                        }

                        routes.add(route);
                    }

                    // Step 3: Print routes and compute travel times
                    for (int r = 0; r < routes.size(); r++) {
                        List<Integer> route = routes.get(r);
                        double totalTime = 0.0;

                        for (int k = 0; k < route.size() - 1; k++) {
                            int from = route.get(k);
                            int to = route.get(k + 1);
                            totalTime += data.getCrewTravelTimeMatrix()[from][to] + data.getServiceTimeMatrix()[to];
                            manual_crew_arrival_times[to] = totalTime;
                        }

                        ArrayList<String> zoneBasedRoute = new ArrayList<>();
                        zoneBasedRoute.add("[0]");
                        for (int k = 1; k < route.size()-1; k++) {
                            String z = data.getTask2zones()[route.get(k)].toString();
                            zoneBasedRoute.add(z);
                        }
                        zoneBasedRoute.add("[0]");
                        System.out.println("Crew Route " + (r + 1) + ": " + zoneBasedRoute.toString() + " | Total travel time: " + totalTime+"/"+data.getHorizon());
//                        System.out.println("Crew Route " + (r + 1) + ": " + route + " | Total travel time: " + totalTime+"/"+data.getHorizon());
                    }
                }
            }

            double[] utilities = new double[data.getNodeNumber()+1];
            double W = data.getHorizon();
            for (int i = 1; i < data.getNodeNumber(); i++){
                double manual_repairTime = 0;
                for(Integer q : data.getZone2tasks()[i])
                    manual_repairTime = Math.max(manual_repairTime,manual_crew_arrival_times[q]);
                utilities[i] = W - (manual_repairTime - service[i]);
//                System.out.println("Tui"+i+":"+(int)W +"-"+"("+HF.truncate(repairTimes[i],100) +"-"+ HF.truncate(service[i],100)+")"+"="+utilities[i]);//TODO
            }
            System.out.println("Utilities:");
            for (int i = 1; i < data.getNodeNumber(); i++)
                System.out.println("\t"+i+":"+utilities[i]);


            System.out.println("Final cost:"+cplex.getObjValue());

            LocalWriter.bestObjectiveValue = cplex.getObjValue();

        } else {
            System.out.println("the model is either infeasible or no integer solution was found within the TL...");
//            throw new IllegalArgumentException(Msg.infeasibility);
        }
    }

    /**
     * Fixes all consecutive arcs from the saved routes by setting LB = 1.
     * This is the requested replay mechanism; no other route variables are
     * explicitly fixed.
     */
    private void fixSaved2indexRoutes(
            IloNumVar[][] crewArcVars,
            IloNumVar[][] towerArcVars,
            List<List<Integer>> technicianPlans,
            List<List<Integer>> towerRoutes) throws IloException {

        if (technicianPlans.size() != data.getCrewNumber()) {
            throw new IllegalArgumentException(
                    "Expected " + data.getCrewNumber()
                            + " technician routes, but solution file contains "
                            + technicianPlans.size() + ".");
        }

        if (towerRoutes.size() != data.getTowerNumber()) {
            throw new IllegalArgumentException(
                    "Expected " + data.getTowerNumber()
                            + " tower routes, but solution file contains "
                            + towerRoutes.size() + ".");
        }

        fixRouteArcs(
                crewArcVars,
                technicianPlans,
                data.getDepotEnd(true),
                "technician");

        fixRouteArcs(
                towerArcVars,
                towerRoutes,
                data.getDepotEnd(false),
                "tower");
    }

    private void fixRouteArcs(
            IloNumVar[][] arcVars,
            List<List<Integer>> routes,
            int expectedTerminal,
            String routeType) throws IloException {

        for (int routeIndex = 0; routeIndex < routes.size(); routeIndex++) {
            List<Integer> route = routes.get(routeIndex);

            if (route.size() < 3) {
                throw new IllegalArgumentException(
                        routeType + " route " + routeIndex
                                + " must contain depot, at least one visited node, "
                                + "and terminal node: " + route);
            }

            if (route.get(0) != 0) {
                throw new IllegalArgumentException(
                        routeType + " route " + routeIndex
                                + " does not start at depot 0: " + route);
            }

            if (route.get(route.size() - 1) != expectedTerminal) {
                throw new IllegalArgumentException(
                        routeType + " route " + routeIndex
                                + " ends at " + route.get(route.size() - 1)
                                + " instead of expected terminal "
                                + expectedTerminal + ": " + route);
            }

            for (int k = 0; k < route.size() - 1; k++) {
                int from = route.get(k);
                int to = route.get(k + 1);

                if (from < 0 || from >= arcVars.length
                        || arcVars[from] == null
                        || to < 0 || to >= arcVars[from].length
                        || arcVars[from][to] == null) {
                    throw new IllegalArgumentException(
                            "No " + routeType + " routing variable exists for arc "
                                    + from + " -> " + to
                                    + " in route " + route + ".");
                }

                arcVars[from][to].setLB(1.0);
            }
        }
    }

    /**
     * Computes cumulative weighted service interruption through each milestone.
     *
     * For zone i at time t:
     *   interruption_i(t)
     *     = min(delta_i, t)
     *       - clamp(t - gamma_i, 0, z_i).
     *
     * Thus t=600 reproduces the model objective
     * sum_i weight_i * (delta_i - z_i), while t=0 is zero.
     */
    private double[] collectObjectiveValueStatistics(
            IloNumVar[] deltaVars,
            IloNumVar[] gammaVars,
            IloNumVar[] serviceVars) throws IloException {

        if (Math.abs(data.getHorizon() - OBJECTIVE_STATISTICS_HORIZON) > 1e-9) {
            throw new IllegalStateException(
                    "Objective statistics assume horizon "
                            + OBJECTIVE_STATISTICS_HORIZON
                            + ", but Data horizon is " + data.getHorizon() + ".");
        }

        int numberOfPoints =
                OBJECTIVE_STATISTICS_HORIZON / OBJECTIVE_STATISTICS_INTERVAL + 1;
        double[] result = new double[numberOfPoints];

        int index = 0;
        for (int t = 0;
             t <= OBJECTIVE_STATISTICS_HORIZON;
             t += OBJECTIVE_STATISTICS_INTERVAL) {

            double objectiveAtT = 0.0;

            for (int i = 1; i < data.getNodeNumber(); i++) {
                double restorationTime = cplex.getValue(deltaVars[i]);
                double towerArrivalTime = cplex.getValue(gammaVars[i]);
                double totalTowerService = cplex.getValue(serviceVars[i]);

                double restorationDurationThroughT =
                        Math.min(restorationTime, (double) t);

                double towerServiceThroughT = Math.max(
                        0.0,
                        Math.min(
                                totalTowerService,
                                (double) t - towerArrivalTime));

                double interruptionThroughT = Math.max(
                        0.0,
                        restorationDurationThroughT - towerServiceThroughT);

                objectiveAtT +=
                        data.getWeights()[i] * interruptionThroughT;
            }

            result[index++] = objectiveAtT;
        }

        // At the full planning horizon the cumulative statistic must equal the
        // MILP objective, up to numerical tolerance.  Use the solver value in
        // the CSV-facing result to avoid tiny floating-point discrepancies.
        double modelObjective = cplex.getObjValue();
        double computedFinal = result[result.length - 1];
        if (Math.abs(computedFinal - modelObjective) > 1e-5) {
            throw new IllegalStateException(
                    "Computed objective at t=600 (" + computedFinal
                            + ") does not match CPLEX objective ("
                            + modelObjective + ").");
        }
        result[result.length - 1] = modelObjective;

        return result;
    }


    private double objVal;
    public void solve() {
        try {
            if(cplex.solve()){
                double v1 = cplex.getObjValue();
                this.objVal = v1;
                if(!BendersCuts.toggle)
                    System.out.println("MIP solved successfully:"+v1);

                getSolution();
            } else {
                if(BendersCuts.toggle) {
                    objVal = 999999999;
                    System.out.println("The model is infeasible.");
                }
                else
                    throw new IllegalArgumentException(Msg.infeasibility);
            }
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }
    private double[][][] crewSolution; private double[][] ySolution; private double zBound;
    private void getSolution() throws IloException {

        for (int k = 0; k < data.getCrewNumber(); k++){
            for (int i = 0; i < data.getTasks(); i++) {
                for (int j = 1; j < data.getTasks()+1; j++) {
                    if(i!=j){
                        double v = cplex.getValue(varXcrews[i][j][k]);
                        if(v > 1e-6)
                            System.out.println("Crew"+k+"["+
                                    i+"->"+j+":"+v+"]");

                    }
                }
            }
        }

        for (int k = 0; k < data.getTowerNumber(); k++){
            for (int i = 0; i < data.getNodeNumber(); i++) {
                for (int j = 1; j < data.getNodeNumber()+1; j++) {
                    if(i!=j){
                        double v = cplex.getValue(varXtowers[i][j][k]);
                        if(v > 1e-6)
                            System.out.println("Tower"+k+"["+
                                    i+"->"+j+":"+v+"]");

                    }
                }
            }
        }


        double totRep = 0.;
        double[] repairTimes = new double[data.getNodeNumber()];
        System.out.println("Repair Times:");
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double v1= cplex.getValue(varDelta[i]);
            repairTimes[i] = v1;
            totRep += v1;
            System.out.println("\t"+i+":"+v1 + "*" + data.getWeights()[i]+"=("+v1*data.getWeights()[i]+")");
        }
        System.out.println("Total Omega:"+totRep);
        double totService = 0;
        System.out.println("Tower Service Durations:");
        for (int i = 1; i < data.getNodeNumber(); i++) {
            for (int k = 0; k < data.getTowerNumber(); k++){
                double v = cplex.getValue(varZ[i][k]);
                totService += v;
                if(v > 1e-6){
                    System.out.println("\t"+i+"by"+k
                            +":"+v+ "*" + data.getWeights()[i]+"=("+v*data.getWeights()[i]+")");
                }
            }
        }
        System.out.println("MIP Z:"+totService);
        zBound = totService;
        System.out.println("Tower ys:");
        for (int i = 1; i < data.getNodeNumber(); i++) {
            for (int k = 0; k < data.getTowerNumber(); k++){
                double v = cplex.getValue(varY[i][k]);
                if(v > 1e-6){
                    System.out.println("\tY:"+i+"by"+k+":"+v);
                    ySolution[i][k] = v;
                }
            }
        }
        ArrayList<HashMap<Integer,Integer>> crewRoutes = new ArrayList<>();
        ArrayList<HashMap<Integer,Integer>> towerRoutes = new ArrayList<>();
        if(!Constants.RELAX_MIP){
            System.out.println("Tower Routes:");
            for (int k = 0; k < data.getTowerNumber(); k++) {
                HashMap<Integer,Integer> map = new HashMap<>();
                for (int i = 0; i < data.getNodeNumber()+1-1; i++) {
                    for (int j = 1; j < data.getNodeNumber()+1; j++) {
                        if(i==j)
                            continue;
                        double v = cplex.getValue(varXtowers[i][j][k]);
                        if(v > 1e-6){
                            map.put(i,j);
                        }
                    }
                }
                towerRoutes.add(map);
            }
            for(HashMap<Integer,Integer> map : towerRoutes){
                Integer from = 0;
                do {
                    System.out.print(from + ",");
                    from = map.get(from);
                } while (map.get(from) != null);
                System.out.println(data.getDepotEnd(false));
            }

            System.out.println("Crew Routes:");
            for (int k = 0; k < data.getCrewNumber(); k++) {
                HashMap<Integer,Integer> map = new HashMap<>();
                for (int i = 0; i < data.getTasks(); i++) {
                    for (int j = 1; j < data.getTasks()+1; j++) {
                        if(i==j || (i==0&&j==data.getDepotEnd(true)))
                            continue;
//                        System.out.println(i+"ll"+j);
                        double v = cplex.getValue(varXcrews[i][j][k]);
                        if(v > 1e-6){
                            map.put(i,j);
                            crewSolution[i][j][k] = v;
                        }
                    }
                }
                crewRoutes.add(map);
            }
            for(HashMap<Integer,Integer> map : crewRoutes){
                Integer from = 0;
                do {
                    System.out.print(from + ",");
                    from = map.get(from);
                } while (map.get(from) != null);
                System.out.println(data.getDepotEnd(true));
            }
        } else {
            for (int k = 0; k < data.getCrewNumber(); k++) {
                System.out.println("Crew"+k+":");
                for (int i = 0; i < data.getTasks(); i++) {
                    for (int j = 1; j < data.getTasks()+1; j++) {
                        if(i!=j){
                            double v =cplex.getValue(varXcrews[i][j][k]);
                            if(v > 1e-6){
                                System.out.println("\tx"+i+","+j+":"+v);
                                crewSolution[i][j][k] = v;
                            }
                        }
                    }
                }
            }


            for (int k = 0; k < data.getTowerNumber(); k++) {
                System.out.println("Tower"+k+":");
                for (int i = 0; i < data.getNodeNumber(); i++) {
                    for (int j = 1; j < data.getNodeNumber()+1; j++) {
                        if(i!=j){
                            double v =cplex.getValue(varXtowers[i][j][k]);
                            if(v > 1e-6){
                                System.out.println("\tx"+i+","+j+":"+v);
                            }
                        }
                    }
                }
            }
        }


        if(varSchedule != null){
            System.out.println("restoration schedule:");
            for (int i = 0; i < data.getNodeNumber(); i++) {
                for (int j = 1; j < data.getNodeNumber()+1; j++) {
                    if(i==0&&j==data.getDepotEnd(false))
                        continue;
                    if(i!=j){
                        double v1 = cplex.getValue(varSchedule[i][j]);
                        if(v1 > 1e-6){
                            System.out.println("\t"+i+"-"+j+":"+v1);
                        }
                    }
                }
            }
            System.out.println("Manual Schedule:");
            System.out.print(0+", ");
            boolean[] checked = new boolean[data.getNodeNumber()];
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double min = Double.MAX_VALUE;
                int k = -1;
                for (int j = 1; j < data.getNodeNumber(); j++) {
                    if(checked[j])
                        continue;
                    if(repairTimes[j] < min){
                        min = repairTimes[j];
                        k = j;
                    }
                }
                System.out.print(k+", ");
                checked[k] = true;
            }
            System.out.println(data.getDepotEnd(false));
        }

        System.out.println("Final cost:"+cplex.getObjValue());
    }


    public double getObjVal() {
        try {
            return cplex.getObjValue();
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }




}

