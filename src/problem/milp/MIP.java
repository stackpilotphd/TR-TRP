package problem.milp;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import lib.*;
import problem.BP.BendersCuts;
import problem.Constants;
import problem.graph.Data;

import java.io.*;
import java.util.*;

public class MIP {

    private IloCplex cplex;
    private IloObjective obj;
    private Data data;

    private IloNumVar[][][] varXtowers, varXcrews;
    private IloNumVar[][] varY, varGamma, varZ, varOmega, varExtra, varSchedule;
    private IloNumVar[] varDelta, varLambda;


    private IloRange limRange;
    private IloNumVar[] varA; private IloNumVar varEta;

    /**
     * When the upper-bounding heuristic is enabled, tower routes may move only
     * between zones in non-increasing order of zone weight. Depot arcs are not
     * restricted because the depot indices do not represent zones.
     */
    private void applyTowerWeightOrdering(IloNumVar[][] towerArcVars) throws IloException {
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



    public void buildAndSolve2index() throws IloException {

        if(Constants.ROBUST){
            buildAndSolveRobustForRepairTimes();
            return;
        }



        this.cplex = new IloCplex();
        cplex.setParam(IloCplex.Param.TimeLimit, Utility.bPCTiLim);
        if(BendersCuts.toggle)
            cplex.setParam(IloCplex.Param.TimeLimit, 60.);
        this.data = Data.getInstance();
        IloNumVar[][] varXtowers = new IloNumVar[data.getNodeNumber()][data.getNodeNumber()+1];
        IloNumVar[][] varXcrews = new IloNumVar[data.getTasks()][data.getTasks()+1];


        for (int i = 0; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber()+1; j++) {
                if(i==0&&j==data.getDepotEnd(false))
                    continue;
                if(i!=j) {
//                    if(Constants.RELAX_MIP)
//                        varXtowers[i][j] = cplex.numVar(0.,1.,"xT_"+ HF.getVarName(List.of(i,j)));
//                    else
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



        if(BendersCuts.toggle){
            {
                for (int i = 1; i < data.getTasks(); i++) {
                    IloNumExpr expr = cplex.numExpr();
                    expr = cplex.sum(expr,varOmega[i]);
                    double min = Double.MAX_VALUE;
                    min = data.getCrewTravelTimeMatrix()[0][i];
                    min+= data.getServiceTimeMatrix()[i];
                    cplex.addGe(expr,min,"RestorationLB,"+i);
                }
            }

            {
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    double max = 0;
                    for(Integer q : data.getZone2tasks()[i]){
                        double v = data.getCrewTravelTimeMatrix()[0][q] + data.getServiceTimeMatrix()[q];
                        if(max < v)
                            max = v;
                    }
                    IloNumExpr expr = cplex.numExpr();
                    expr = cplex.sum(expr,varDelta[i]);
                    cplex.addGe(expr,max,"ZoneRestorationLB,"+i);
                }
            }

            {
                double min = Double.MAX_VALUE;
                for (int k = 0; k < data.getTasks(); k++) {
                    double dist = data.getCrewTravelTimeMatrix()[k][data.getDepotEnd(true)];
                    if(dist < min)
                        min = dist;
                }
                double max = 0;
                for (int i = 0; i < data.getNodeNumber(); i++) {
                    double dist = data.getTowerTravelTimeMatrix()[i][data.getDepotEnd(false)];
                    if(dist > max)
                        max = dist;
                }
                double Q = data.getHorizon() - min + max;
                IloNumExpr expr = cplex.numExpr();
                expr = cplex.sum(expr,varGamma[data.getDepotEnd(false)]);
                cplex.addLe(expr,Q,"validCutForTowers");
            }
        }



        if(BendersCuts.toggle){

            {
                IloNumExpr expr = cplex.numExpr();
                for (int i = 1; i < data.getNodeNumber(); i++)
                    expr = cplex.sum(expr,  varZ[i]);
                cplex.addLe(expr, data.getMaxZ(),"serviceBound");
            }
        }


        if(BendersCuts.toggle) {
            IloNumExpr expr = cplex.numExpr();
            for (int i = 1; i < data.getNodeNumber(); i++)
                expr = cplex.sum(expr,varY[i]);
            limRange = cplex.addRange(StaticSharedValues.limLB,expr,StaticSharedValues.limUB,"'limRange");
            this.varA = varY;
            return;
        }

        //-----------------------------------------

        if(Constants.FIX_CREW_ROUTES) {
            if(true) {
                String filePath = "fixed_crew_routes.txt";
                List<ArrayList<Integer>> patterns = new ArrayList<>();
                try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
                    String line;
                    while ((line = reader.readLine()) != null) {
                        line = line.trim();
                        if (line.startsWith("[") && line.endsWith("]")) {
                            line = line.substring(1, line.length() - 1); // remove brackets
                        }

                        ArrayList<Integer> pattern = new ArrayList<>();
                        if (!line.isEmpty()) {
                            for (String token : line.split(",")) {
                                pattern.add(Integer.parseInt(token.trim()));
                            }
                        }
                        patterns.add(pattern);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
                for(ArrayList<Integer> p : patterns){
                    for (int k1 = 0; k1 < p.size()-1; k1++) {
                        int from = p.get(k1);
                        int k2 = k1+1;
                        int to = p.get(k2);
                        varXcrews[from][to].setLB(1.);
                    }
                }
            }
        }

        String filePath = "";


        if(Constants.COST_OF_PRIORITY){
            System.out.println("cost of Priority/Discrimination");
            for (int i = 1; i < data.getTasks(); i++) {
                for (int j = 1; j < data.getTasks(); j++) {
                    int priorityFrom = data.getTaskPriority()[i];
                    int priotiyTo = data.getTaskPriority()[j];
                    if(priotiyTo > priorityFrom)
                        varXcrews[i][j].setUB(0.);
                }
            }
        }



        if(cplex.solve()){
            if(!BendersCuts.toggle)
                System.out.println("MIP solved successfully:"+cplex.getObjValue());


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


            System.out.println("Final cost:"+cplex.getObjValue());

            LocalWriter.bestObjectiveValue = cplex.getObjValue();
        } else {
            System.out.println("the model is either infeasible or no integer solution was found within the TL...");
        }
    }


    public void buildAndSolveRobustForRepairTimes() throws IloException {
        this.cplex = new IloCplex();
        cplex.setParam(IloCplex.DoubleParam.TiLim, Utility.bPCTiLim);
        this.data = Data.getInstance();
        int budget = StaticSharedValues.budget; //Constants.BUDGET+1
        IloNumVar[][] varXtowers = new IloNumVar[data.getNodeNumber()][data.getNodeNumber()+1];
        IloNumVar[][] varXcrews = new IloNumVar[data.getTasks()][data.getTasks()+1];

        for (int i = 0; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber()+1; j++) {
                if(i==0&&j==data.getDepotEnd(false))
                    continue;
                if(i!=j) {
//                    if(Constants.RELAX_MIP)
//                        varXtowers[i][j] = cplex.numVar(0.,1.,"xT_"+ HF.getVarName(List.of(i,j)));
//                    else
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
        IloNumVar[][] varOmega = new IloNumVar[data.getTasks()+1][budget];
        for (int i = 0; i < data.getNodeNumber(); i++) {
            varExtra[i] = cplex.numVar(0.,Double.MAX_VALUE,"e_"+ HF.getVarName(List.of(i)));
            varGamma[i] = cplex.numVar(0.,Double.MAX_VALUE,"gT_"+ HF.getVarName(List.of(i)));
            varZ[i] = cplex.numVar(0.,Double.MAX_VALUE,"zT_"+ HF.getVarName(List.of(i)));
            if(i==0){
                varZ[i].setUB(0);
                varGamma[i].setUB(0.);
                continue;
            }
            if(Constants.RELAX_MIP && !BendersCuts.toggle)
                varY[i] = cplex.numVar(0.,1.,"yT_"+ HF.getVarName(List.of(i)));
            else
                varY[i] = cplex.boolVar("yT_"+ HF.getVarName(List.of(i)));
        }

        varGamma[data.getDepotEnd(false)] = cplex.numVar(0.,Double.MAX_VALUE,"gT_"+ HF.getVarName(List.of(data.getDepotEnd(false))));

        for (int i = 0; i < data.getTasks()+1; i++) {
            for (int g = 0; g < budget; g++) {
                varOmega[i][g] = cplex.numVar(0.,Double.MAX_VALUE,"oC_"+ HF.getVarName(List.of(i,g)));
                if(i==0) varOmega[0][g].setUB(0);
            }
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
                if(i==0&&j==data.getDepotEnd(true))
                    continue;
                if(i!=j){
                    for (int g = 0; g < budget; g++) {
                        double m1 = data.getHorizon() + data.getCrewTravelTimeMatrix()[i][j]+data.getServiceTimeMatrix()[j]+data.getServiceDeviation()[j];
                        IloNumExpr expr = cplex.numExpr();
                        expr = cplex.sum(expr,varOmega[i][g]);
                        expr = cplex.sum(expr,cplex.prod(-1.,varOmega[j][g]));
                        expr = cplex.sum(expr
                                ,cplex.prod(data.getCrewTravelTimeMatrix()[i][j] + data.getServiceTimeMatrix()[j],varXcrews[i][j]));
                        expr = cplex.sum(expr,cplex.prod(m1, varXcrews[i][j]));
                        cplex.addGe(m1
                                ,expr,"taskTime1_"+HF.getVarName(List.of(i,j,g)));
                    }
                }
            }
        }
        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks()+1; j++) {
                if(i==0&&j==data.getDepotEnd(true))
                    continue;
                if(i!=j){
                    for (int g = 1; g < budget; g++) {
                        double m1 = data.getHorizon() + data.getCrewTravelTimeMatrix()[i][j]+data.getServiceTimeMatrix()[j]+data.getServiceDeviation()[j];;
                        IloNumExpr expr = cplex.numExpr();
                        expr = cplex.sum(expr,varOmega[i][g-1]);
                        expr = cplex.sum(expr,cplex.prod(-1.,varOmega[j][g]));
                        expr = cplex.sum(expr
                                ,cplex.prod(data.getCrewTravelTimeMatrix()[i][j] + data.getServiceTimeMatrix()[j]
                                        + data.getServiceDeviation()[j],varXcrews[i][j]));
                        expr = cplex.sum(expr,cplex.prod(m1, varXcrews[i][j]));
                        cplex.addGe(m1
                                ,expr,"taskTime2_"+HF.getVarName(List.of(i,j,g)));
                    }
                }
            }
        }
        for (int g = 0; g < budget; g++) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varOmega[data.getDepotEnd(true)][g]);
            cplex.addLe(expr,data.getHorizon(),"crewShift");
        }
        for (int i = 1; i < data.getNodeNumber(); i++)
            for(Integer q : data.getZone2tasks()[i]){
                for (int g = 0; g < budget; g++) {
                    IloNumExpr expr = cplex.numExpr();
                    expr = cplex.sum(expr,varDelta[i]);
                    expr = cplex.sum(expr,cplex.prod(-1.,varOmega[q][g]));
                    cplex.addGe(expr,0.,"deltaToMaxTaskTime_"+HF.getVarName(List.of(i,q,g)));
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

        for (int i = 1; i < data.getNodeNumber(); i++) {
            for (int j = 1; j < data.getNodeNumber(); j++) {
                if(i!=j){
                    double m = data.getHorizon() + data.getTowerTravelTimeMatrix()[i][j]+data.getPositionTimeMatrix()[j];
                    IloNumExpr expr = cplex.numExpr();
                    expr = cplex.sum(expr,varGamma[i]);
                    expr = cplex.sum(expr,cplex.prod(-1.,varGamma[j]));
                    expr = cplex.sum(expr,varZ[i]);
                    expr = cplex.sum(expr,cplex.prod(m, varXtowers[i][j]));
                    cplex.addGe(m-data.getTowerTravelTimeMatrix()[i][j]-data.getPositionTimeMatrix()[j]
                            ,expr,"(TowerTimes)"+HF.getVarName(List.of(i,j)));
                }
            }
        }

        {
            int j = data.getDepotEnd(false);
            for (int i = 1; i < data.getNodeNumber(); i++){
                double m2 = data.getHorizon() + data.getTowerTravelTimeMatrix()[i][j];
                IloNumExpr expr = cplex.numExpr();
                expr = cplex.sum(expr,varGamma[i]);
                expr = cplex.sum(expr,cplex.prod(-1.,varGamma[j]));
                expr = cplex.sum(expr,varZ[i]);
                expr = cplex.sum(expr,cplex.prod(m2, varXtowers[i][j]));
                cplex.addGe(m2-data.getTowerTravelTimeMatrix()[i][j],expr,"(TowerTimes)"+HF.getVarName(List.of(i,j)));
            }

        }

        for (int i = 1; i < data.getNodeNumber(); i++) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varGamma[i]);
            expr = cplex.sum(expr,cplex.prod(-1. * (data.getTowerTravelTimeMatrix()[0][i] + data.getPositionTimeMatrix()[i]),varY[i]));
            cplex.addGe(expr,0.,"towerFromDepotTo_"+i);
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

            {
                IloNumExpr expr = cplex.numExpr();
                expr = cplex.sum(expr,varEta);
                obj = cplex.addMinimize(expr);
            }


            IloNumExpr expr = cplex.numExpr();
            for (int i = 1; i < data.getNodeNumber(); i++)
                expr = cplex.sum(expr,varY[i]);
            limRange = cplex.addRange(StaticSharedValues.limLB,expr,StaticSharedValues.limUB,"'limRange");
            this.varA = varY;

        } else {
            {
                IloNumExpr expr = cplex.numExpr();
                for (int i = 1; i < data.getNodeNumber(); i++){
                    expr = cplex.sum(expr,cplex.prod(data.getWeights()[i],varDelta[i]));
                    expr = cplex.sum(expr,cplex.prod(-1.*data.getWeights()[i],varZ[i]));
                }
                cplex.addGe(expr,0.);Msg.todo("251018");
            }
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
            expr = cplex.sum(expr,cplex.prod(-1.*data.getHorizon(),varY[i]));
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
            expr = cplex.sum(expr,cplex.prod(-1.*data.getHorizon(),varY[i]));
            expr = cplex.sum(expr,cplex.prod(-1.,varDelta[i]));
            cplex.addGe(expr,-1.*data.getHorizon(),"(E3)"+HF.getVarName(List.of(i)));
        }

        //-----------------------------------------


        if(BendersCuts.toggle)
            return;

        if(cplex.solve()){
            System.out.println("MIP solved successfully:"+cplex.getObjValue());
            LocalWriter.bestObjectiveValue = cplex.getObjValue();

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

            System.out.println("Repair Times:");
            double[] repairTimes = new double[data.getNodeNumber()];
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v1= cplex.getValue(varDelta[i]);
                repairTimes[i] = v1;
//                System.out.println("\t"+i+":"+v1 + "*" + data.getWeights()[i]+"=("+v1*data.getWeights()[i]+")");
                System.out.println("\t"+i+":"+v1);
            }

            double[] service = new double[data.getNodeNumber()+1];
            System.out.println("Tower Service Durations:");
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v = cplex.getValue(varZ[i]);
                if(v > 1e-6){
                    service[i] = v;
                    System.out.println("\t"+i
                            +":"+v+ "*" + data.getWeights()[i]+"=("+v*data.getWeights()[i]+")");
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
            ArrayList<HashMap<Integer,Integer>> crewRoutes = new ArrayList<>();
            ArrayList<HashMap<Integer,Integer>> towerRoutes = new ArrayList<>();
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
                        }

                        System.out.println("Crew Route " + (r + 1) + ": " + route + " | Total travel time: " + totalTime+"/"+data.getHorizon());
                    }
                }
            }


            System.out.println("Final cost:"+cplex.getObjValue());
        } else {
            System.out.println("The model is either infeasible or no feasible solution was found within the time limit");
//            throw new IllegalArgumentException(Msg.infeasibility);
        }

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

    private void getSolution() throws IloException {

        if(BendersCuts.toggle) {
            int[] aSolution = new int[data.getNodeNumber() + 1];
            BendersCuts.currentYs = new int[data.getNodeNumber()];
            HashSet<Integer> hashSet = new HashSet<>();
            double totalYs = 0;
            double[] as = new double[data.getNodeNumber()];
            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v = cplex.getValue(varA[i]);
                if(v > 1e-6){
                    as[i] = v;
                    totalYs += v;
                    aSolution[i] = 1;
                    BendersCuts.currentYs[i] = 1;

                    hashSet.add(i);
                    BendersCuts.matrixY[i]++;
                }
            }
//            System.out.println("Tower ys:" + totalYs);
//            for (int i = 1; i < data.getNodeNumber(); i++) {
//                double v = as[i];if(v > 1e-6) System.out.println("\tY:"+i+":"+v);
//            }
            BendersCuts.hashSetY.add(hashSet);
            BendersCuts.currentValue = cplex.getObjValue();
            return;
        }

        double[][][] crewSolution = new double[data.getTasks()][data.getTasks() + 1][data.getCrewNumber()];
        double[][] ySolution = new double[data.getNodeNumber() + 1][data.getTowerNumber()];


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


    public void addOptimalityCut(double value) throws IloException {
        {
            int[] array = BendersCuts.currentYs;
            int number = 0;
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr,varEta);
            for (int i = 1; i < data.getNodeNumber(); i++) {
                if(array[i] > 0){
                    expr = cplex.sum(expr,cplex.prod(-1.*value,varA[i]));
                    number++;
                } else {
                    expr = cplex.sum(expr,cplex.prod(value,varA[i]));
                }
            }
            IloRange range = cplex.addGe(expr,value - number*value);
        }
    }


    public double getObjective() {
        return objVal;
    }





}
