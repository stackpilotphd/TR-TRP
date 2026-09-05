package problem.BP;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import lib.*;
import problem.Constants;
import problem.graph.Arc;
import problem.graph.Data;
import problem.multi.ICEAconstraint;
import problem.multi.SharedBounds;

import java.util.*;

public class LP {

    private static int compareIntegerLists(List<Integer> left, List<Integer> right) {
        int commonLength = Math.min(left.size(), right.size());
        for (int i = 0; i < commonLength; i++) {
            int comparison = Integer.compare(left.get(i), right.get(i));
            if (comparison != 0) return comparison;
        }
        return Integer.compare(left.size(), right.size());
    }

    private static int compareVariables(VarR left, VarR right) {
        int routeComparison = compareIntegerLists(
                left.getRoute().getPattern(), right.getRoute().getPattern());
        if (routeComparison != 0) return routeComparison;
        return left.getNumVar().toString().compareTo(right.getNumVar().toString());
    }

    private final int maxTowerRouteLength;
    public LP() {
        this.maxTowerRouteLength = StaticSharedValues.maximumTowerRouteLength;
    }

    private boolean isFifthThread;
    private IloCplex cplex;
    private BNode bNode;
    private double cost;
    //---------------------------------
    private HashMap<Integer, HashMap<ArrayList<Integer>, Integer>> outerColumnHashMap;
    private HashMap<ArrayList<Integer>, HashMap<ArrayList<Boolean>, Integer>> innerColumnHashMap;
    private HashMap<ArrayList<Integer>, HashMap<ArrayList<Boolean>, Integer>> innerColumnHashMapCrew;
    private ArrayList<Constraint> constraints;
    private int[][][] feasibleSchedule_arc;
    private int[][][] arcInitialValue;
    private double[] crewLengthLimits;
    private HashMap<Integer, VarR> towerVariableMap;
    private HashMap<Integer, VarR> crewVariableMap;
    private HashMap<Integer, IloNumVar> variablesLambda;
    private HashMap<Integer, IloNumVar> slaks;
    private ArrayList<VarR> active_Rset;
    private ArrayList<VarR> active_Cset;
    private ArrayList<IloNumVar> active_slaks;
    private ArrayList<VarR>[][] ij2towerVar;
    private ArrayList<Integer>[][] ij2towerCount;
    private ArrayList<VarR>[][] ij2varCrewOuterPattern;
    private ArrayList<Integer>[][] ij2crewCount;
    private ArrayList<VarR>[] i2towerVars;
    private ArrayList<VarR>[] i2CrewVars;
    private ArrayList<Integer>[] i2towerCount;
    private ArrayList<Integer>[] i2CrewCount;
    private ArrayList<Double>[] i2CrewArrivals;
    private HashMap<Long, ArrayList<VarR>> long2crewVariableList;

    private ArrayList<VarR> modifiedVars;
    HashMap<Integer, HashMap<Integer, ArrayList<VarR>>> i2j2crewVarsMap;
    HashMap<Integer, HashMap<Integer, ArrayList<VarR>>> i2j2towerMap;
    //---------------------------------
    private Data data;

    //---------------------------------
    private IloObjective obj;
    private boolean fixedRouteFlag;
    private ArrayList<VarR> fixedVars;
    private final double bigM = Constants.M;
    private IloNumVar[] varService;
    private IloNumVar[] varArrival;
    private IloNumVar[] varY;
    private IloNumVar[][] var_edge;

    private boolean shouldConvert;
    private HashMap<Long, ArrayList<Integer>> long2crewRouteVars;
    private double lrangeUB;

    public void construct(BNode bNode) {

        this.bNode = bNode;
        this.data = Data.getInstance();
        try {
            initializeDataStructures();
            initializeCplex();
            initializeConstraints();
            initializeMustVisitCrewNodes();
            if (!shouldConvert)
                addColumns(bNode.getReliefRoutes(), bNode.getCrewRoutes());
            if (!bNode.hasInitialSolution())
                initializeSlacks();
            if (BendersCuts.toggle) {
                addColumns(null, bNode.getFixedCrewRoutes());
            }
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }

    private boolean[] mustVisitNodesDueToBranching;
    private boolean disableNodeEliminationRule;

    private void initializeMustVisitCrewNodes() {
//        disableNodeEliminationRule = false;
        boolean[] must = new boolean[data.getTasks() + 1];
        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 1; j < data.getTasks() + 1; j++) {
                if (must[j])
                    continue;
                int value = bNode.getOuterArcValue(i, j, true);
                if (value != 0) {
                    must[i] = true;
                    must[j] = true;
                    disableNodeEliminationRule = true;
                }
            }
        }
        mustVisitNodesDueToBranching = must;
    }

    public boolean[] getMustVisitNodesDueToBranching() {
        return mustVisitNodesDueToBranching;
    }

    private void initializeDataStructures() {
        slaks = new HashMap<>();
        outerColumnHashMap = new HashMap<>();
        outerColumnHashMap.put(1, new HashMap<>());
        outerColumnHashMap.put(2, new HashMap<>());

        innerColumnHashMap = new HashMap<>();
        innerColumnHashMapCrew = new HashMap<>();
        active_Rset = new ArrayList<>();
        active_Cset = new ArrayList<>();
        active_slaks = new ArrayList<>();
        towerVariableMap = new HashMap<>();
        variablesLambda = new HashMap<>();
        crewVariableMap = new HashMap<>();
        long2crewVariableList = new HashMap<>();
        ij2towerVar = new ArrayList[data.getNodeNumber()][data.getNodeNumber() + 1];
        ij2varCrewOuterPattern = new ArrayList[data.getTasks()][data.getTasks() + 1];
        ij2crewCount = new ArrayList[data.getTasks()][data.getTasks() + 1];
        ij2towerCount = new ArrayList[data.getNodeNumber()][data.getNodeNumber() + 1];
        feasibleSchedule_arc = new int[2][Math.max(data.getNodeNumber(), data.getTasks())][Math.max(data.getNodeNumber() + 1, data.getTasks() + 1)];
        arcInitialValue = new int[2][][];
        for (int i = 0; i < data.getNodeNumber(); i++) {
            for (int j = 0; j < data.getNodeNumber() + 1; j++) {
                ij2towerVar[i][j] = new ArrayList<>();
                ij2towerCount[i][j] = new ArrayList<>();
            }
        }
        for (int i = 0; i < data.getTasks(); i++) {
            for (int j = 0; j < data.getTasks() + 1; j++) {
                ij2varCrewOuterPattern[i][j] = new ArrayList<>();
                ij2crewCount[i][j] = new ArrayList<>();
            }
        }

        i2j2towerMap = new HashMap<>();
        for (int i = 0; i < data.getNodeNumber() - 1; i++) {
            i2j2towerMap.put(i, new HashMap<>());
            for (int j = i + 1; j < data.getNodeNumber(); j++) {
                i2j2towerMap.get(i).put(j, new ArrayList<>());
            }
        }
        i2j2crewVarsMap = new HashMap<>();
        for (int i = 0; i < data.getTasks() - 1; i++) {
            i2j2crewVarsMap.put(i, new HashMap<>());
            for (int j = i + 1; j < data.getTasks() + 1; j++) {
                i2j2crewVarsMap.get(i).put(j, new ArrayList<>());
            }
        }
        i2towerVars = new ArrayList[data.getNodeNumber() + 1];
        i2CrewVars = new ArrayList[data.getTasks() + 1];
        i2towerCount = new ArrayList[data.getNodeNumber() + 1];
        i2CrewCount = new ArrayList[data.getTasks() + 1];
        i2CrewArrivals = new ArrayList[data.getTasks() + 1];
        for (int i = 0; i < data.getNodeNumber() + 1; i++) {
            i2towerVars[i] = new ArrayList<>();
            i2towerCount[i] = new ArrayList<>();
        }
        for (int i = 0; i < data.getTasks() + 1; i++) {
            i2CrewVars[i] = new ArrayList<>();
            i2CrewCount[i] = new ArrayList<>();
            i2CrewArrivals[i] = new ArrayList<>();
        }
        for (int k = 0; k < 2; k++) {
            if (k == 0) {
                arcInitialValue[k] = new int[data.getNodeNumber()][data.getNodeNumber() + 1];
                for (int i = 0; i < data.getNodeNumber(); i++) {
                    for (int j = 0; j < data.getNodeNumber() + 1; j++) {
                        arcInitialValue[k][i][j] = bNode.getFeasibleSchedule_arc()[k][i][j];
                    }
                }
            } else {
                arcInitialValue[k] = new int[data.getTasks()][data.getTasks() + 1];
                for (int i = 0; i < data.getTasks(); i++) {
                    for (int j = 0; j < data.getTasks() + 1; j++) {
                        arcInitialValue[k][i][j] = bNode.getFeasibleSchedule_arc()[k][i][j];
                    }
                }
            }

        }
        modifiedVars = new ArrayList<>();

    }

    public double computeTowerReducedCostUsingColumnLogic(Route route) {

        ArrayList<Integer> schedule = route.getPattern();
        ArrayList<Boolean> waitList = route.getWaitBooleans();
        ArrayList<Double> arrivals = route.getArrivals();

        if (arrivals == null || arrivals.size() != schedule.size()) {
            arrivals = computeTowerArrivals(schedule, waitList);
        }

        double c = computeTowerReducedCostUsingColumnLogic(schedule, waitList, arrivals);

//        processColumnsTowerElementary(new ArrayList<>(List.of(route)));
        return c;
    }
    public double computeTowerReducedCostUsingColumnLogic(
            ArrayList<Integer> schedule,
            ArrayList<Boolean> waitList,
            ArrayList<Double> arrivals
    ) {

        if (schedule == null || waitList == null || arrivals == null) {
            throw new IllegalArgumentException("schedule, waitList, and arrivals cannot be null.");
        }

        if (schedule.size() != waitList.size()) {
            throw new IllegalArgumentException(
                    "schedule and waitList have different sizes: "
                            + schedule.size() + " vs " + waitList.size()
            );
        }

        if (schedule.size() != arrivals.size()) {
            throw new IllegalArgumentException(
                    "schedule and arrivals have different sizes: "
                            + schedule.size() + " vs " + arrivals.size()
            );
        }

        HashSet<Integer> demands = new HashSet<>();
        HashMap<Integer, Integer> demandNodeVisitCount = new HashMap<>();
        HashMap<Integer, Double> arrivalCount = new HashMap<>();
        HashMap<Integer, Double> waitCount = new HashMap<>();

        for (int node : schedule) {
            demandNodeVisitCount.merge(node, 1, Integer::sum);
        }

        for (int pos = 1; pos < arrivals.size() - 1; pos++) {
            int node = schedule.get(pos);

            arrivalCount.merge(node, arrivals.get(pos), Double::sum);
            demands.add(node);
        }

        for (int pos = 1; pos < waitList.size() - 1; pos++) {
            int node = schedule.get(pos);

            if (Boolean.TRUE.equals(waitList.get(pos))) {
                waitCount.merge(node, -data.getServiceM(), Double::sum);
            }
        }

        /*
         * ------------------------------------------------------------
         * Now compute the pricing value using the same coefficients.
         * ------------------------------------------------------------
         */
        double reducedCost = getDualVisitR()[0];
        System.out.println("dualVisit[0] = " + reducedCost);

        for (Integer node : demands) {

            double arrivalCoeff = arrivalCount.getOrDefault(node, 0.0);
            int visitCoeff = demandNodeVisitCount.getOrDefault(node, 0);

            reducedCost += getDualSigma()[node] * arrivalCoeff;
            reducedCost += getDualVisitR()[node] * visitCoeff;

            System.out.println(node+":[A="+arrivalCoeff+";V="+visitCoeff+"]");
        }

        for (Integer node : waitCount.keySet()) {
            double waitCoeff = waitCount.get(node);

            reducedCost += getDualPhi()[node] * waitCoeff;
            System.out.println(node+":[W="+waitCoeff);
        }


        System.out.println("Reduced Cost (Manual)="+reducedCost);
        return reducedCost;
    }

    public double computeCrewReducedCostUsingColumnLogic(Route route) {

        if (route == null) {
            throw new IllegalArgumentException("route cannot be null.");
        }

        ArrayList<Integer> schedule = route.getPattern();

        if (schedule == null || schedule.isEmpty()) {
            throw new IllegalArgumentException("crew route schedule cannot be null or empty.");
        }

        ArrayList<Double> arrivals = route.getArrivals();

        if (arrivals == null || arrivals.size() != schedule.size()) {
            arrivals = computeCrewArrivals(schedule);
        }

        if (schedule.size() != arrivals.size()) {
            throw new IllegalArgumentException(
                    "crew route schedule and arrivals have different sizes: "
                            + schedule.size() + " vs " + arrivals.size()
            );
        }

        if (dual2_visitCrew == null || dual0_psi == null) {
            throw new IllegalStateException(
                    "Crew reduced cost requires current LP duals. Call solve() before computing it."
            );
        }

        HashSet<Integer> demands = new HashSet<>();
        HashMap<Integer, Integer> demandNodeVisitCount = new HashMap<>();
        HashMap<Integer, Double> arrivalCount = new HashMap<>();

        for (int node : schedule) {
            demandNodeVisitCount.merge(node, 1, Integer::sum);
        }

        for (int pos = 1; pos < arrivals.size() - 1; pos++) {
            int node = schedule.get(pos);

            arrivalCount.merge(node, arrivals.get(pos), Double::sum);
            demands.add(node);
        }

        double[] arrayForCrews = computeArrayForCrewReducedCosts();
        double reducedCost = getDualVisitC()[0];

        for (Integer node : demands) {
            int visitCoeff = demandNodeVisitCount.getOrDefault(node, 0);
            double arrivalCoeff = arrivalCount.getOrDefault(node, 0.0);

            reducedCost += getDualVisitC()[node] * visitCoeff;
            reducedCost += arrayForCrews[node] * arrivalCoeff;
        }


        long vI = route.getLong();
        if (vI == 0L) {
            vI = computeCrewRouteLong(schedule);
        }




        return reducedCost;
    }

    private ArrayList<Double> computeCrewArrivals(ArrayList<Integer> schedule) {
        ArrayList<Double> arrivals = new ArrayList<>(schedule.size());
        int from = schedule.getFirst();

        if (Constants.ROBUST) {
            double[][] gammas = new double[data.getTasks() + 1][StaticSharedValues.budget];
            arrivals.add(0.0);

            for (int pos = 1; pos < schedule.size(); pos++) {
                int to = schedule.get(pos);
                double t = data.getCrewTravelTimeMatrix()[from][to] + data.getServiceTimeMatrix()[to];

                gammas[to][0] = gammas[from][0] + t;
                double max = gammas[to][0];

                for (int k = 1; k < StaticSharedValues.budget; k++) {
                    if (Constants.R_TOGGLE == 0) {
                        gammas[to][k] = Math.max(
                                gammas[from][k] + t,
                                gammas[from][k - 1] + t + data.getServiceDeviation()[to]
                        );
                    } else {
                        gammas[to][k] = Math.max(
                                gammas[from][k] + t,
                                gammas[from][k - 1] + t + data.getCrewTravelTimeDeviations()[from][to]
                        );
                    }

                    max = Math.max(max, gammas[to][k]);
                }

                arrivals.add(max);
                from = to;
            }

            return arrivals;
        }

        double time = 0.0;

        for (Integer to : schedule) {
            if (to != from) {
                time += data.getCrewTravelTimeMatrix()[from][to] + data.getServiceTimeMatrix()[to];
                from = to;
            }

            arrivals.add(time);
        }

        return arrivals;
    }

    private double[] computeArrayForCrewReducedCosts() {
        double[] arrayForCrews = new double[data.getTasks() + 1];

        for (int zone = 1; zone < data.getNodeNumber(); zone++) {
            for (Integer task : data.getZone2tasks()[zone]) {
                arrayForCrews[task] += getDual0Psi()[zone][task];
            }
        }

        return arrayForCrews;
    }


    private long computeCrewRouteLong(ArrayList<Integer> schedule) {
        long vI = 0x00L;
        int depotEnd = data.getDepotEnd(true);

        for (Integer node : schedule) {
            vI |= (1L << node);
        }

        vI &= ~(1L << 0);
        vI &= ~(1L << depotEnd);

        return vI;
    }



    public void addColumns(ArrayList<Route> columnsR, ArrayList<Route> columnsC) {
        double start = System.nanoTime();
        if (columnsR != null) {
            double start1 = System.nanoTime();
            processColumnsTower(columnsR);
            StaticSharedValues.lp_columnTowerCPU += (System.nanoTime() - start1);
        }
        if (columnsC != null) {
            double start2 = System.nanoTime();
            processColumnsCrew(columnsC);
            StaticSharedValues.lp_columnCrewCPU += (System.nanoTime() - start2);
        }
        StaticSharedValues.lp_columnCPU += (System.nanoTime() - start);
    }




    private HashMap<ArrayList<Integer>, ArrayList<IloNumVar>> route2Towervars;
    private HashMap<ArrayList<Integer>, IloNumVar> towerRep2var;

    private void processColumnsTower(ArrayList<Route> columns) {
        int var_count = towerVariableMap.size();
        for (Route route : columns) {
            ArrayList<Integer> schedule = route.getPattern();



            ArrayList<Boolean> waitList = route.getWaitBooleans();
            ArrayList<Double> arrivals = route.getArrivals();

            if (schedule.size() != waitList.size()) {
                throw new IllegalArgumentException(
                        "Tower route schedule and waitList have different sizes: "
                                + schedule.size() + " vs " + waitList.size()
                );
            }

            if (arrivals == null || arrivals.size() != schedule.size()) {
                arrivals = computeTowerArrivals(schedule, waitList);
            }

            HashSet<Integer> demands = new HashSet<>();
            HashMap<Integer, Integer> demandNodeVisitCount = new HashMap<>();
            for (int i : schedule)
                demandNodeVisitCount.merge(i, 1, Integer::sum);


            HashMap<Integer, Double> arrivalCount = new HashMap<>();
            for (int i = 1; i < arrivals.size() - 1; i++) {
                arrivalCount.merge(schedule.get(i), arrivals.get(i), Double::sum);
                demands.add(schedule.get(i));
            }

            HashMap<Integer, Double> waitCount = new HashMap<>();
            for (int i = 1; i < waitList.size() - 1; i++)
                if (waitList.get(i)) {
                    waitCount.merge(schedule.get(i), -1. * data.getServiceM(), Double::sum);
                }


            ArrayList<Integer> path = schedule;
            int from = 0;
            int[][] visitCount = new int[data.getNodeNumber()][data.getNodeNumber() + 1];
            for (int i = 1; i < path.size(); i++) {
                int to = path.get(i);
                visitCount[from][to]++;
                if (bNode.getFeasibleSchedule_arc()[0][from][to] == -1)
                    Msg.infeasibleArc();
                from = to;
            }

            try {
                IloColumn col = cplex.column(constraints.get(3).getRange(0), 1.);

                for (Integer key : new TreeSet<>(demands)) {
                    col = add_column(col, constraints.get(5).getRange(key), arrivalCount.get(key));
                    col = add_column(col, constraints.get(1).getRange(key), demandNodeVisitCount.get(key));
                }


                for (Integer key : new TreeSet<>(waitCount.keySet())) {
                    double t = waitCount.get(key);
                    col = add_column(col, constraints.get(6).getRange(key), t);
                }




                IloNumVar numVar = cplex.numVar(col, 0.0, 1e16, IloNumVarType.Float, "R," + var_count);

                if (shouldConvert) {
                    route2Towervars.computeIfAbsent(schedule, k -> new ArrayList<>());
                    route2Towervars.get(schedule).add(numVar);
                    if (towerRep2var.get(schedule) == null)
                        towerRep2var.put(schedule, cplex.numVar(0.0, 1., IloNumVarType.Bool, "R2," + var_count));
                }


                VarR var = new VarR();
                var.setRoute(route);
                var.setTheta(numVar);
                towerVariableMap.put(var_count, var);

                outerColumnHashMap.get(1).put(schedule, var_count);
                innerColumnHashMap.computeIfAbsent(schedule, k -> new HashMap<>()).put(waitList, var_count);


                for (int i = 0; i < data.getNodeNumber(); i++) {
                    for (int j = 1; j < data.getNodeNumber() + 1; j++) {
                        if (visitCount[i][j] > 0) {
                            ij2towerVar[i][j].add(var);
                            ij2towerCount[i][j].add(visitCount[i][j]);
                            if (i < j) {

                                if (i != 0 && j != data.getDepotEnd(false))
                                    i2j2towerMap.get(i).get(j).add(var);

                            } else {
                                if (visitCount[j][i] < 1e-6)
                                    i2j2towerMap.get(j).get(i).add(var);
                            }
                        }
                    }
                }
                for (Integer i : new TreeSet<>(demandNodeVisitCount.keySet())) {
                    i2towerVars[i].add(var);
                    i2towerCount[i].add(demandNodeVisitCount.get(i));
                }
            } catch (IloException e) {
                throw new RuntimeException(e);
            }
            var_count++;
            StaticSharedValues.towerColumnCount++;
        }
    }





    private ArrayList<Double> computeTowerArrivals(
            ArrayList<Integer> schedule,
            ArrayList<Boolean> waitList
    ) {

        if (schedule.size() != waitList.size()) {
            throw new IllegalArgumentException(
                    "Cannot compute arrivals: schedule and waitList sizes differ."
            );
        }

        ArrayList<Double> arrivals = new ArrayList<>(schedule.size());

        double time = 0.0;
        arrivals.add(time);

        for (int pos = 1; pos < schedule.size(); pos++) {

            int from = schedule.get(pos - 1);
            int to = schedule.get(pos);

            time += data.getTowerTravelTimeMatrix()[from][to];
            time += data.getPositionTimeMatrix()[to];

            arrivals.add(time);

            if (Boolean.TRUE.equals(waitList.get(pos))) {
                time += data.getServiceM();
            }
        }

        return arrivals;
    }






    private void processColumnsCrew(ArrayList<Route> columns) {

        int var_count = crewVariableMap.size();
        for (Route route : columns) {
            ArrayList<Integer> schedule = route.getPattern();

            if(Constants.COST_OF_PRIORITY_CREW){
                int from = schedule.get(1);
                for (int k = 2; k < schedule.size() - 1; k++) {
                    int to = schedule.get(k);
                    if (data.getTaskPriority()[to] > data.getTaskPriority()[from])
                        throw new IllegalArgumentException("infeasible route:" + schedule.toString());
                    from = to;
                }
            }



            HashSet<Integer> demands = new HashSet<>();
            HashMap<Integer, Integer> demandNodeVisitCount = new HashMap<>();
            for (int i : schedule)
                demandNodeVisitCount.merge(i, 1, Integer::sum);




            ArrayList<Double> arrivals = route.getArrivals();
            HashMap<Integer, Double> arrivalCount = new HashMap<>();
            for (int i = 1; i < arrivals.size() - 1; i++) {
                arrivalCount.merge(schedule.get(i), arrivals.get(i), Double::sum);
                demands.add(schedule.get(i));
            }

            HashMap<ArrayList<Integer>, Integer> list2value = new HashMap<>();

            int from = 0;
            int[][] visitCount = new int[data.getTasks()][data.getTasks() + 1];
            for (int i = 1; i < schedule.size(); i++) {
                int to = schedule.get(i);
                visitCount[from][to]++;
                if (bNode.getFeasibleSchedule_arc()[1][from][to] == -2)
                    Msg.infeasibleArc();
                from = to;
            }



            try {


                IloColumn col = cplex.column(constraints.get(4).getRange(0), 1.);
                for (Integer key : demands) {
                    col = add_column(col, constraints.get(2).getRange(key), demandNodeVisitCount.get(key));
                    double t = arrivalCount.get(key);
                    {
                        for (Integer zoneID : data.getTask2zones()[key]) {
                            col = add_column(col, constraints.get(0).getRange(zoneID, key), t);
                        }
                    }
                }




                IloNumVar numVar;

                if (shouldConvert) {
                    numVar = cplex.numVar(col, 0.0, 1., IloNumVarType.Bool, "C," + var_count);
                } else numVar = cplex.numVar(col, 0.0, 1e16, IloNumVarType.Float, "C," + var_count);



                VarR var = new VarR();
                var.setRoute(route);
                var.setTheta(numVar);
                crewVariableMap.put(var_count, var);

                outerColumnHashMap.get(2).put(schedule, var_count);


                for (int i = 0; i < data.getTasks(); i++) {
                    for (int j = 0; j < data.getTasks() + 1; j++) {
                        if (visitCount[i][j] > 0) {
                            ij2varCrewOuterPattern[i][j].add(var);
                            ij2crewCount[i][j].add(visitCount[i][j]);
                            if (i < j) {
                                if (i != 0 && j != data.getDepotEnd(true))
                                    i2j2crewVarsMap.get(i).get(j).add(var);
                            } else {
                                if (visitCount[j][i] < 1e-6) {
                                    i2j2crewVarsMap.get(j).get(i).add(var);
                                }
                            }
                        }
                    }
                }


                for (Integer i : new TreeSet<>(demandNodeVisitCount.keySet())) {
                    i2CrewVars[i].add(var);
                    i2CrewCount[i].add(demandNodeVisitCount.get(i));
                }


                for (Integer key : new TreeSet<>(arrivalCount.keySet())) {
                    if (key != 0 && key != data.getDepotEnd(true)) {
                        double t = arrivalCount.get(key);
                        i2CrewArrivals[key].add(t);
                    }
                }


            } catch (IloException e) {
                throw new RuntimeException(e);
            }
            var_count++;
            StaticSharedValues.crewColumnCount++;
        }
    }




    public void initializeSlacks() {
        if (shouldConvert)
            return;
        try {
            int slack_cnt = 0;
            for (Constraint c : constraints) {
                for (IloRange range : c.getRanges()) {
                    IloColumn col = cplex.column(obj, bigM);
                    if (range.getUB() <= 0. && range.getLB() < -1.) {
                        col = add_column(col, range, -1.);
                    } else
                        col = add_column(col, range, Math.max(range.getLB(), 1.));
                    slaks.put(slack_cnt, cplex.numVar(col, 0., bigM, IloNumVarType.Float, "D" + range.getName()));
                    slack_cnt++;
                }
            }

        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }

    private IloColumn add_column(IloColumn column, IloRange range, double v) throws IloException {
        if (range == null) {
            throw new IllegalArgumentException("null range");
        }
        return column.and(cplex.column(range, v));
    }

    private void initializeCplex() {
        //---------MODEL--------
        try {
            cplex = new IloCplex();
            cplex.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Dual);
            cplex.setParam(IloCplex.IntParam.Threads, 1);

            cplex.setParam(IloCplex.BooleanParam.NumericalEmphasis, true);
            cplex.setParam(IloCplex.Param.Simplex.Tolerances.Feasibility, 1e-7);
            cplex.setParam(IloCplex.DoubleParam.EpOpt, 1e-7);
            cplex.setParam(IloCplex.DoubleParam.EpInt, 1e-8);
            cplex.setParam(IloCplex.IntParam.ScaInd, 1);
            cplex.setParam(IloCplex.IntParam.MIPEmphasis, IloCplex.MIPEmphasis.Optimality);

//            cplex.setParam(IloCplex.IntParam.SimDisplay,0);
//            cplex.setParam(IloCplex.IntParam.MIPDisplay,0);
//            cplex.setParam(IloCplex.IntParam.TuningDisplay,0);
            if (Utility.algo == 29) {
                cplex.setParam(IloCplex.Param.Threads,1);
            }
            cplex.setOut(null);
            cplex.setWarning(null);
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }



    private IloNumVar[] varDelta;

    private void initializeConstraints() throws IloException {
        constraints = new ArrayList<>();
            //----------OBJECTIVE---------------------
            {
                IloNumExpr expr = cplex.linearNumExpr();
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    variablesLambda.put(i, cplex.numVar(0., 1e16, "L," + i));
                    expr = cplex.sum(expr, cplex.prod(data.getWeights()[i], variablesLambda.get(i)));
                }
                obj = cplex.addMinimize(expr);
            }

            //----------------------------------------
            //------Lambda Bounding-------------------------
            {
                varService = new IloNumVar[data.getNodeNumber()];
                varDelta = new IloNumVar[data.getNodeNumber()];
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    varService[i] = cplex.numVar(0., Double.MAX_VALUE, "r," + i);
                    varDelta[i] = cplex.numVar(0., Double.MAX_VALUE, "delta," + i);
                }
                //This cut is currently set to true
                if (Constants.SERVICE_BOUND_CUT) {
                    for (int i = 1; i < data.getNodeNumber(); i++) {
                        IloNumExpr expr = cplex.numExpr();
                        expr = cplex.sum(expr, varDelta[i]);
                        double minDistanceToDepot = Double.MAX_VALUE;
                        for (Integer q : data.getZone2tasks()[i])
                            minDistanceToDepot = Math.min(minDistanceToDepot, data.getCrewTravelTimeMatrix()[q][data.getDepotEnd(true)]);
                        double ub = data.getHorizon() - minDistanceToDepot;
                        cplex.addLe(expr, ub, "upperBoundOnRestorationTime," + i);
                    }
                }

                for (int i = 1; i < data.getNodeNumber(); i++) {
                    IloNumExpr expr = cplex.numExpr();
                    expr = cplex.sum(expr, varDelta[i]);
                    expr = cplex.sum(expr, cplex.prod(-1., variablesLambda.get(i)));
                    expr = cplex.sum(expr, cplex.prod(-1., varService[i]));
                    cplex.addLe(expr, 0., "delta2z2lamba," + i);
                }

                if (Constants.SERVICE_BOUND_CUT) {
                    IloNumExpr expr = cplex.numExpr();
                    for (int i = 1; i < data.getNodeNumber(); i++)
                        expr = cplex.sum(expr, varService[i]);
                    cplex.addLe(expr, data.getMaxZ(), "serviceBound");
                }

                //------0 deltas-------------------------
                {
                    Constraint cons = new Constraint(Double.NEGATIVE_INFINITY, 0., "delta,");
                    constraints.add(cons);
                    for (int i = 1; i < data.getNodeNumber(); i++) {
                        for (Integer k : data.getZone2tasks()[i]) {
                            cons.initialize(cplex, i, k);
                            cons.getRange(i, k).setExpr(cplex.sum(cplex.numExpr(), cplex.prod(-1., varDelta[i])));
                        }
                    }
                }
            }
            //---------1 Packing via the binary y variables
            {
                varY = new IloNumVar[data.getNodeNumber()];
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    double lb = bNode.getPackingLB(i);
                    double ub = bNode.getPackingUB(i);
                    if (shouldConvert) {
                        //shouldConvert is set to true only when running a MIP heuristic
                        varY[i] = cplex.boolVar("y_" + HF.getVarName(List.of(i)));
                        varY[i].setLB(lb);
                        varY[i].setUB(ub);
                    } else
                        varY[i] = cplex.numVar(lb, ub, "y_" + HF.getVarName(List.of(i)));
                }
                Constraint cons = new Constraint(0., 0., "pack,");
                constraints.add(cons);
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    cons.initialize(cplex, i);
                    cons.getRange(i).setExpr(cplex.sum(cplex.numExpr(), cplex.prod(-1., varY[i])));
                }
            }
            //------2 Covering-------------------------
            {
                Constraint cons;
                cons = new Constraint(1., 1., "cover,");
                constraints.add(cons);
                for (int i = 1; i < data.getTasks(); i++) {
                    cons.initialize(cplex, i);
                }

            }
            //------3 Tower Fleet-------------------------
            {
                Constraint cons = new Constraint(0, data.getTowerNumber(), "fleetR,");
                constraints.add(cons);
                cons.initialize(cplex, 0);

            }
            //------4 Crew Fleet-------------------------
            {
                Constraint cons = new Constraint(0, data.getCrewNumber(), "fleetC,");
                constraints.add(cons);
                cons.initialize(cplex, 0);
            }
            //------5 Waiting-------------------------
            {
                Constraint cons = new Constraint(Double.NEGATIVE_INFINITY, 0., "wait,");
                constraints.add(cons);
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    cons.initialize(cplex, i);
                    cons.getRange(i).setExpr(cplex.sum(cplex.numExpr(), cplex.prod(-1., varDelta[i])));
                }
            }
            //------6 Tower Service to varService-------------------------
            {
                Constraint cons = new Constraint(Double.NEGATIVE_INFINITY, 0., "service,");
                constraints.add(cons);
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    cons.initialize(cplex, i);
                    cons.getRange(i).setExpr(cplex.sum(cplex.numExpr(), varService[i]));
                }
            }

            //------7 Crew Edge Vars-------------------------
            {
                Constraint cons = new Constraint(0., 0., "edge,");
                constraints.add(cons); //these are redundant constraints, they are not active
            }
            //------LIMITATION CUT-------------------------
            {
                int[] nodeTotalBounds = bNode.getTotsBounds();
                boolean hasNodeTotalBranch = nodeTotalBounds != null
                        && (nodeTotalBounds[0] > 0 || nodeTotalBounds[1] < data.getNodeNumber());
                if (Utility.algo != 1 || hasNodeTotalBranch) {
                    IloNumExpr expr = cplex.numExpr();
                    for (int i = 1; i < data.getNodeNumber(); i++)
                        expr = cplex.sum(expr, varY[i]);
                    IloRange limRange;
                    if (Utility.algo == 29&&!Constants.UPPER_BOUNDING_HEURISTIC) {
                        double lb = iceaConstrain.lb;
                        double ub = iceaConstrain.ub;
                        if (nodeTotalBounds != null) {
                            lb = Math.max(lb, nodeTotalBounds[0]);
                            ub = Math.min(ub, nodeTotalBounds[1]);
                        }
                        if(iceaConstrain.id <= 1)
                            lb = 0;
                        limRange = cplex.addRange(lb, expr, ub, "LimCut,");
                        this.lrangeUB = ub;
                    } else if (Utility.algo == 1) {
                        limRange = cplex.addRange(nodeTotalBounds[0], expr, nodeTotalBounds[1], "LimCut,");
                        this.lrangeUB = nodeTotalBounds[1];
                    } else {
                        double lb = StaticSharedValues.limLB;
                        double ub = StaticSharedValues.limUB;
                        if (nodeTotalBounds != null) {
                            lb = Math.max(lb, nodeTotalBounds[0]);
                            ub = Math.min(ub, nodeTotalBounds[1]);
                        }
                        limRange = cplex.addRange(lb, expr, ub, "LimCut,");
                        this.lrangeUB = ub;
                    }
                }
            }





        //----------------------------------------
    }


    public boolean solve() throws IloException {
        double start = System.nanoTime();
        boolean ter = cplex.solve();
        if (ter) {
            cost = cplex.getObjValue();
            setValues();
            setDuals();
        }
        StaticSharedValues.lp_solvingCPU += (System.nanoTime() - start);
        return ter;
    }


    public void modifyA(int i, int lb, int ub) {
        try {
            varY[i].setLB(lb);
            varY[i].setUB(ub);
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }


    public void addSlacks(int constraintID, int rangeID) {
        IloRange range = constraints.get(constraintID).getRange(rangeID);
        double v = 1e9;
        IloColumn col = null;
        try {
            int slack_cnt = slaks.size();
            col = cplex.column(obj, v);
            col = add_column(col, range, Math.max(range.getLB(), 1.));
            slaks.put(slack_cnt, cplex.numVar(col, 0., bigM, IloNumVarType.Float, "Dummy(" + constraintID + ")" + slack_cnt));
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }



    private double[][] dual0_psi;
    private double[] dual1_visitRtower;
    private double[] dual2_visitCrew;
    private double[] dual6_phi;
    private double[] dual5_sigma;


    private void setDuals() throws IloException {
        dual0_psi = new double[data.getNodeNumber()][data.getTasks()];
        dual1_visitRtower = new double[data.getNodeNumber() + 1];
        dual1_visitRtower[0] = cplex.getDual(constraints.get(3).getRange(0));
        dual2_visitCrew = new double[data.getTasks() + 1];
        dual2_visitCrew[0] = cplex.getDual(constraints.get(4).getRange(0));
        dual5_sigma = new double[data.getNodeNumber() + 1];
        dual6_phi = new double[data.getNodeNumber() + 1];
         {
            for (int i = 1; i < data.getNodeNumber(); i++) {
                dual1_visitRtower[i] = cplex.getDual(constraints.get(1).getRange(i));
                dual5_sigma[i] = cplex.getDual(constraints.get(5).getRange(i));
                dual6_phi[i] = cplex.getDual(constraints.get(6).getRange(i));
                for (Integer q : data.getZone2tasks()[i])
                    dual0_psi[i][q] = cplex.getDual(constraints.get(0).getRange(i, q));
            }
        }





        for (int q = 1; q < data.getTasks(); q++) {
            dual2_visitCrew[q] = cplex.getDual(constraints.get(2).getRange(q));
        }


    }









    public double[] getDualSigma() {
        return dual5_sigma;
    }

    public double[] getDualVisitC() {
        return dual2_visitCrew;
    }

    public double[] getDualVisitR() {
        return dual1_visitRtower;
    }

    public double[] getDualPhi() {
        return dual6_phi;
    }






    private double[] active_a_values;

    private void setValues() throws IloException {

        active_Rset.clear();
        active_Cset.clear();
        active_slaks.clear();

        active_a_values = new double[data.getNodeNumber()];

        if (varY != null) {
            for (int i = 1; i < data.getNodeNumber(); i++)
                active_a_values[i] = cplex.getValue(varY[i]);
        }


        for (VarR var : towerVariableMap.values()) {
            double v = cplex.getValue(var.getNumVar());
            var.setValue(v);
            if (v > 1e-6)
                active_Rset.add(var);
        }
        for (VarR var : crewVariableMap.values()) {
            double v = cplex.getValue(var.getNumVar());
            var.setValue(v);
            if (v > 1e-6)
                active_Cset.add(var);
        }
        for (IloNumVar var : slaks.values()) {
            double v = cplex.getValue(var);
            if (v > 1e-6)
                active_slaks.add(var);
        }


    }





    public boolean hasActiveSlaks() {
        return !active_slaks.isEmpty();
    }

    public double getCost() {
        return cost;
    }


    public Integer r2i(ArrayList<Integer> schedule, ArrayList<Boolean> waiting) {

        if (waiting == null || waiting.isEmpty()) {
            Map<ArrayList<Integer>, Integer> crewMap = outerColumnHashMap.get(2);

            if (crewMap == null) {
                return null;
            }

            return crewMap.get(schedule);
        }

        Map<ArrayList<Boolean>, Integer> patternMap = innerColumnHashMap.get(schedule);

        if (patternMap == null) {
            return null;
        }

        return patternMap.get(waiting);
    }



    public Integer r2i(ArrayList<Integer> schedule, ArrayList<Boolean> waiting, boolean isCrew) {
        if (isCrew) {
            if (outerColumnHashMap.get(3).containsKey(schedule))
                return innerColumnHashMapCrew.get(schedule).get(waiting);
            return null;
        } else {
            if (outerColumnHashMap.get(1).containsKey(schedule))
                return innerColumnHashMap.get(schedule).get(waiting);

            return null;
        }
    }


    private String solutionString;

    public String getSolutionString() {
        return solutionString;
    }

    public void printSolution() throws IloException {

        if (true) {
            printSimpleSolution();
            return;
        }
        double[] specificZs = new double[data.getNodeNumber()];
        double[] specificGammas = new double[data.getNodeNumber() + 1];
        double[] omegaMap = new double[data.getTasks() + 1];
        try {
            double cst = getCost();
            String yds = "Cost:" + cst;
            StringBuilder solution = new StringBuilder();
            if (bNode.getBranchStrig() != null)
                solution.append(bNode.getBranchStrig()).append(System.lineSeparator());
            solution.append(yds).append(System.lineSeparator());
            StringBuilder stringBuilder = new StringBuilder();
            double totValue = 0.;
            int cc_count = active_Rset.size();
            double totalZ = 0.;

            for (VarR var : active_Rset) {
                Route route = var.getRoute();
                double v = HF.truncate(cplex.getValue(var.getNumVar()), 1000000);
                stringBuilder.append("\t" + var.getNumVar().getName());
                stringBuilder.append("(" + v + "):"); //HF.truncate((cplex.getValue(var.getNumVar()) * 10e6),100)
                ArrayList<Integer> pattern = route.getPattern();
                stringBuilder.append(pattern.toString() + ";");
                stringBuilder.append(route.getWaitBooleans().toString());
                int nom = 0;
                for (Boolean i : route.getWaitBooleans()) {
                    if (i) {
                        totalZ += v * data.getServiceM();
                        specificZs[pattern.get(nom)] += v * data.getServiceM();
                    }
                    specificGammas[pattern.get(nom)] += v * route.getArrivals().get(nom);
                    nom++;
                }
                if (cc_count > 1)
                    stringBuilder.append(System.lineSeparator());
                cc_count--;
                totValue += v;
            }

            if (true) {
                totalZ = 0;
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    totalZ += cplex.getValue(varService[i]);
                }
            }

            stringBuilder.append(System.lineSeparator()).append("Total Z:").append(totalZ);

            String d = "Tower Routes,V:" + totValue;
            solution.append(d).append(System.lineSeparator());
            solution.append(stringBuilder.toString());

            String ds;
            if (active_Cset.isEmpty())
                ds = "";
            else {
                ds = "\nCrew Routes:";
                solution.append(ds).append(System.lineSeparator());
            }
            //System.out.println(ds);
            int cccccc = active_Cset.size();
            stringBuilder = new StringBuilder();

            for (VarR varR : active_Cset) {
                Route route = varR.getRoute();
                double v = varR.getValue();
                ArrayList<Integer> path = route.getPattern();
                ArrayList<Double> ats = route.getArrivals();
                for (int i = 1; i < path.size(); i++) {
                    int to = path.get(i);
                    double at = ats.get(i);
                    omegaMap[to] += at * v;
                }
            }
            for (VarR var : active_Cset) {
                stringBuilder.append("\t" + var.getNumVar().getName());
                stringBuilder.append("(" + cplex.getValue(var.getNumVar()) + "):");
                ArrayList<Integer> pattern = var.getRoute().getPattern();
                stringBuilder.append(pattern.toString());
                if (cccccc > 1)
                    stringBuilder.append(System.lineSeparator());
                cccccc--;
            }
            //System.out.println(stringBuilder);
            solution.append(stringBuilder.toString());
            if (hasActiveSlaks()) {
                stringBuilder = new StringBuilder();
                String g = "Active Slacks:";
                //System.out.println(g);
                stringBuilder.append(g);
                for (IloNumVar var : active_slaks) {
                    stringBuilder.append("\t" + var.getName());
                    stringBuilder.append("(" + cplex.getValue(var) + "):");
                }
                stringBuilder.append(System.lineSeparator());
                //System.out.println(stringBuilder.toString());
                solution.append(stringBuilder);
            }

            this.solutionString = solution.toString();



            if (Constants.CONSOLE) {
                if (active_a_values != null) {
                    solution.append("\nY valyes:").append(System.lineSeparator());
                    solution.append(Arrays.toString(active_a_values));
                }


                solutionString = solution.toString();
                System.out.println(solutionString);

            }
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
        System.out.println("Cost:" + cost);
    }



    private void printSimpleSolution() throws IloException {
        System.out.println("LP solution Bound:" + getCost());
        System.out.println("Tower Routes:");
        for (VarR var : active_Rset) {
            Route route = var.getRoute();
            double v = cplex.getValue(var.getNumVar());
            System.out.println("\t" + var.getNumVar().getName() + "(" + v + ")"
                    + ":" + route.getPattern().toString()
                    + ";" + route.getWaitBooleans().toString());
        }
        System.out.println("Crew Routes:");
        for (VarR var : active_Cset) {
            Route route = var.getRoute();
            double v = cplex.getValue(var.getNumVar());
            System.out.println("\t" + var.getNumVar().getName() + "(" + v + ")"
                    + ":" + route.getPattern().toString());
        }
        System.out.println("Y valyes:");
        System.out.println("\t" + Arrays.toString(active_a_values));
        double t_y = 0;
        for (int i = 0; i < data.getNodeNumber(); i++) t_y += active_a_values[i];
        System.out.println("Total Y:" + t_y);

        if(true){
            System.out.println("Z values:");
            System.out.print("\t");
            for (int i = 1; i < data.getNodeNumber(); i++) {
                System.out.print(cplex.getValue(varService[i])+",");
            }
            System.out.println();
        }

        System.out.println("Active slaks:");
        for (IloNumVar varR : active_slaks)
            System.out.println(varR.getName());



        System.out.println("Cost:" + getCost());
    }




    public boolean isFeasible() throws IloException {
        try {
            for (VarR var : active_Cset) {
                double v = cplex.getValue(var.getNumVar());
                if (v < 1. - 1e-6)
                    return false;
            }
            for (int i = 1; i < data.getNodeNumber(); i++) {
                if (HF.is_fractional(cplex.getValue(varY[i])))
                    return false;
            }



            boolean isRouteFeasible = true;
            for (VarR var : active_Rset) {
                double v = cplex.getValue(var.getNumVar());
                if (v < 1. - 1e-6) {
                    isRouteFeasible = false;
                    break;
                }
            }
            if (!isRouteFeasible) {
                HashMap<ArrayList<Integer>, Double> map = new HashMap<>();
                for (VarR var : active_Rset) {
                    double v = cplex.getValue(var.getNumVar());
                    map.merge(var.getRoute().getPattern(), v, Double::sum);
                }
                for (Double v : map.values()) {
                    if (v < 1. - 1e-6)
                        return false;
                }
            }

        } catch (IloException e) {
            throw new RuntimeException(e);
        }
        return true;
    }





    public double getTotalActiveY() {
        double total = 0.0;

        if (active_a_values == null) {
            return total;
        }

        for (int i = 0; i < data.getNodeNumber(); i++) {
            total += active_a_values[i];
        }

        return total;
    }

    public ArrayList<Route> getReliefRoutes() {
        ArrayList<Route> routes = new ArrayList<>();
        for (VarR var : active_Rset) {
            routes.add(var.getRoute());
        }
        return routes;
    }

    public ArrayList<Double> getRouteValues() {
        ArrayList<Double> values = new ArrayList<>();
        for (VarR var : active_Rset) {
            values.add(var.getValue());
        }
        return values;
    }

    public ArrayList<ArrayList<Boolean>> getRouteWaites() {
        ArrayList<ArrayList<Boolean>> values = new ArrayList<>();
        for (VarR var : active_Rset) {
            values.add(var.getRoute().getWaitBooleans());
        }
        return values;
    }

    public ArrayList<Route> getCrewRoutes() {
        ArrayList<Route> routes = new ArrayList<>();
        for (VarR var : active_Cset) {
            routes.add(var.getRoute());
        }
        return routes;
    }

    public ArrayList<Route> getCrewRoutes(ArrayList<Double> vals) {
        ArrayList<Route> routes = new ArrayList<>();
        for (VarR var : active_Cset) {
            routes.add(var.getRoute());
            vals.add(var.getValue());
        }
        return routes;
    }

    public ArrayList<VarR> getActive_Cset() {
        return active_Cset;
    }

    public ArrayList<VarR> getActive_Rset() {
        return active_Rset;
    }




    public void getMap(double[][] map, boolean isCrew) throws IloException {
        if (isCrew) {
            for (VarR var : active_Cset) {
                Route route = var.getRoute();
                ArrayList<Integer> schedule = route.getPattern();
                int from = 0;
                for (int i = 1; i < schedule.size(); i++) {
                    int to = schedule.get(i);
                    map[from][to] += var.getValue();
                    from = to;
                }
            }
        } else {
            for (VarR var : active_Rset) {
                Route route = var.getRoute();
                ArrayList<Integer> path = route.getPattern();
                int from = 0;
                for (int i = 1; i < path.size(); i++) {
                    int to = path.get(i);
                    map[from][to] += var.getValue();
                    from = to;
                }
            }
        }
    }



    public void modifyArc(Arc arc, int lb, int ub, boolean isCrew) throws IloException {
        if (isCrew) {
            if (ub < 1e-6) {
                for (VarR var : ij2varCrewOuterPattern[arc.from()][arc.to()]) {
                    var.getNumVar().setUB(0.);
                }
                feasibleSchedule_arc[1][arc.from()][arc.to()] = -2;
            } else if (lb > 1e-6) {
                if (arc.to() != data.getDepotEnd(isCrew)) {
                    for (int i = 0; i < data.getTasks(); i++) {
                        if (i != arc.from()) {
                            for (VarR var : ij2varCrewOuterPattern[i][arc.to()]) {
                                var.getNumVar().setUB(0.);
                            }
                            feasibleSchedule_arc[1][i][arc.to()] = -2;
                        }
                    }
                }
                if (arc.from() != 0) {
                    for (int j = 0; j < data.getTasks() + 1; j++) {
                        if (j != arc.to()) {
                            for (VarR var : ij2varCrewOuterPattern[arc.from()][j]) {
                                var.getNumVar().setUB(0.);
                            }
                            feasibleSchedule_arc[1][arc.from()][j] = -2;
                        }
                    }
                }
                feasibleSchedule_arc[1][arc.from()][arc.to()] = 2;
            }
        } else {
            if (ub < 1e-6) {
                for (VarR var : ij2towerVar[arc.from()][arc.to()]) {
                    var.getNumVar().setUB(0.);
                }
                feasibleSchedule_arc[0][arc.from()][arc.to()] = -1;
            } else if (lb > 1e-6) {

                feasibleSchedule_arc[0][arc.from()][arc.to()] = 1;

                if (arc.to() != data.getDepotEnd(isCrew)) {
                    for (int i = 0; i < data.getNodeNumber(); i++) {
                        if (i != arc.from()) {
                            for (VarR var : ij2towerVar[i][arc.to()]) {
                                var.getNumVar().setUB(0.);
                            }
                            feasibleSchedule_arc[0][i][arc.to()] = -1;
                        }
                    }
                }
                if (arc.from() != 0) {
                    for (int j = 0; j < data.getNodeNumber() + 1; j++) {
                        if (j != arc.to()) {
                            for (VarR var : ij2towerVar[arc.from()][j]) {
                                var.getNumVar().setUB(0.);
                            }
                            feasibleSchedule_arc[0][arc.from()][j] = -1;
                        }
                    }
                }
            }
        }
    }

    public boolean isExtraBound() {
        return isExtraBound;
    }



    public static class ArcModificationState {
        private final ArrayList<IloNumVar> vars = new ArrayList<>();
        private final ArrayList<Double> lbs = new ArrayList<>();
        private final ArrayList<Double> ubs = new ArrayList<>();
        private final ArrayList<int[]> arcEntries = new ArrayList<>();
        private final ArrayList<Integer> arcValues = new ArrayList<>();
        private final IdentityHashMap<IloNumVar, Boolean> seenVars = new IdentityHashMap<>();
        private final HashSet<String> seenArcEntries = new HashSet<>();

        private void addVar(IloNumVar var) throws IloException {
            if (var == null || seenVars.containsKey(var)) {
                return;
            }
            seenVars.put(var, Boolean.TRUE);
            vars.add(var);
            lbs.add(var.getLB());
            ubs.add(var.getUB());
        }

        private void addArcEntry(int layer, int from, int to, int value) {
            String key = layer + ":" + from + ":" + to;
            if (!seenArcEntries.add(key)) {
                return;
            }
            arcEntries.add(new int[]{layer, from, to});
            arcValues.add(value);
        }
    }

    public ArcModificationState snapshotArcModification(
            Arc arc,
            int lb,
            int ub,
            boolean isCrew
    ) throws IloException {
        ArcModificationState state = new ArcModificationState();

        if (isCrew) {
            if (ub < 1e-6) {
                snapshotCrewArcEntry(state, arc.from(), arc.to());
            } else if (lb > 1e-6) {
                if (arc.to() != data.getDepotEnd(true)) {
                    for (int i = 0; i < data.getTasks(); i++) {
                        if (i != arc.from()) {
                            snapshotCrewArcEntry(state, i, arc.to());
                        }
                    }
                }
                if (arc.from() != 0) {
                    for (int j = 0; j < data.getTasks() + 1; j++) {
                        if (j != arc.to()) {
                            snapshotCrewArcEntry(state, arc.from(), j);
                        }
                    }
                }
                snapshotCrewArcEntry(state, arc.from(), arc.to());
            }
        } else {


            if (ub < 1e-6) {
                snapshotTowerArcEntry(state, arc.from(), arc.to());
            } else if (lb > 1e-6) {

                snapshotTowerArcEntry(state, arc.from(), arc.to());

                if (arc.to() != data.getDepotEnd(false)) {
                    for (int i = 0; i < data.getNodeNumber(); i++) {
                        if (i != arc.from()) {
                            snapshotTowerArcEntry(state, i, arc.to());
                        }
                    }
                }
                if (arc.from() != 0) {
                    for (int j = 0; j < data.getNodeNumber() + 1; j++) {
                        if (j != arc.to()) {
                            snapshotTowerArcEntry(state, arc.from(), j);
                        }
                    }
                }
            }
        }

        return state;
    }

    private void snapshotCrewArcEntry(
            ArcModificationState state,
            int from,
            int to
    ) throws IloException {
        for (VarR var : ij2varCrewOuterPattern[from][to]) {
            state.addVar(var.getNumVar());
        }
        state.addArcEntry(1, from, to, feasibleSchedule_arc[1][from][to]);
    }

    private void snapshotTowerArcEntry(
            ArcModificationState state,
            int from,
            int to
    ) throws IloException {
        for (VarR var : ij2towerVar[from][to]) {
            state.addVar(var.getNumVar());
        }
        state.addArcEntry(0, from, to, feasibleSchedule_arc[0][from][to]);
    }

    public void restoreArcModification(ArcModificationState state) throws IloException {
        for (int idx = 0; idx < state.vars.size(); idx++) {
            IloNumVar var = state.vars.get(idx);
            var.setLB(state.lbs.get(idx));
            var.setUB(state.ubs.get(idx));
        }

        for (int idx = 0; idx < state.arcEntries.size(); idx++) {
            int[] entry = state.arcEntries.get(idx);
            feasibleSchedule_arc[entry[0]][entry[1]][entry[2]] = state.arcValues.get(idx);
        }
    }





    public boolean isFeasibleArc(int i, int j, boolean isCrew) {
        if (isCrew) {

            return bNode.getOuterArcValue(i, j, true) > -2 && feasibleSchedule_arc[1][i][j] > -2;
        }

        if (bNode.getPackingUB(j) < 1e-6 || bNode.getPackingUB(i) < 1e-6) {
            return false;
        }
        return bNode.getOuterArcValue(i, j, false) > -1 && feasibleSchedule_arc[0][i][j] > -1;
    }



    public HashMap<Integer, VarR> getTowerVariables() {
        return towerVariableMap;
    }

    public HashMap<Integer, VarR> getCrewVariables() {
        return crewVariableMap;
    }







    public BNode getBNode() {
        return bNode;
    }






    public double reconstructMIP() throws IloException {
        LP mother = this;
        LP lp = new LP();
        lp.setICEAconsraint(iceaConstrain, isFifthThread);
        lp.setSharedBound(sharedBound);
        lp.setConverstion(true);
        lp.construct(mother.getBNode());
        lp.cplex.setParam(IloCplex.Param.TimeLimit, 30.);
        ArrayList<Route> crews = new ArrayList<>();
        for (VarR varR : mother.getCrewVariables().values()) crews.add(varR.getRoute());
        ArrayList<Route> towers = new ArrayList<>();
        for (VarR varR : mother.getTowerVariables().values()) towers.add(varR.getRoute());
        lp.addColumns(towers, crews);
        lp.addConvexCombinationConstraints();
        boolean fes = lp.solveMIP();
        if (fes) {
            lp.printReconstructedSolution();
            lp.clear();
            mipTowers = lp.getMipTowers();
            mipCrews = lp.getMipCrews();
            return lp.getCost();
        } else {
            lp.clear();
            return Double.MAX_VALUE;
        }
    }

    private ArrayList<Route> mipCrews;
    private ArrayList<Route> mipTowers;

    public ArrayList<Route> getMipCrews() {
        return mipCrews;
    }

    public ArrayList<Route> getMipTowers() {
        return mipTowers;
    }

    private void printReconstructedSolution() throws IloException {
        mipCrews = new ArrayList<Route>();
        mipTowers = new ArrayList<Route>();

        if (Constants.CONSOLE) {
            System.out.println("MIP solution Bound:" + getCost());
            System.out.println("Convex Tower Routes:");
        }

        for (VarR var : towerVariableMap.values()) {
            double v = cplex.getValue(var.getNumVar());
            if (v > 1e-6) {

                if (Constants.CONSOLE) {
                    Route route = var.getRoute();
                    System.out.println("\t" + var.getNumVar().getName() + "(" + v + ")"
                            + ":" + route.getPattern().toString()
                            + ";" + route.getWaitBooleans().toString());
                }
            }
        }
        if (Constants.CONSOLE) System.out.println("Binary Tower Routes:");
        for (ArrayList<Integer> schedule : towerRep2var.keySet()) {
            IloNumVar var = towerRep2var.get(schedule);
            double v = cplex.getValue(var);
            if (v > 1e-6) {
                ArrayList<Boolean> b2 = new ArrayList<>();
                b2.add(false);
                for (int i = 1; i < schedule.size() - 1; i++) {
                    b2.add(true);
                }
                b2.add(false);
                Route r1 = new Route();
                r1.create(schedule, b2);
                this.mipTowers.add(r1);
                if (Constants.CONSOLE) System.out.println("\t" + var.getName() + "(" + v + ")"
                        + ":" + schedule);
            }
        }
        if (Constants.CONSOLE) System.out.println("Crew Routes:");
        for (VarR var : crewVariableMap.values()) {
            double v = cplex.getValue(var.getNumVar());
            if (v > 1e-6) {
                Route route = var.getRoute();
                this.mipCrews.add(route);
                if (Constants.CONSOLE) System.out.println("\t" + var.getNumVar().getName() + "(" + v + ")"
                        + ":" + route.getPattern().toString());
            }
        }

        if (Constants.CONSOLE) {
            active_a_values = new double[data.getNodeNumber()];
            for (int i = 1; i < data.getNodeNumber(); i++) active_a_values[i] = cplex.getValue(varY[i]);
            System.out.println("Y valyes:");
            System.out.println("\t" + Arrays.toString(active_a_values));
            double t_y = 0;
            for (int i = 0; i < data.getNodeNumber(); i++) t_y += active_a_values[i];
            System.out.println("Total Y:" + t_y);
            System.out.println("MIP Cost:" + getCost());
        }
    }

    private void addConvexCombinationConstraints() throws IloException {
        ArrayList<ArrayList<Integer>> sortedSchedules =
                new ArrayList<>(route2Towervars.keySet());
        sortedSchedules.sort(LP::compareIntegerLists);
        for (ArrayList<Integer> schedule : sortedSchedules) {
            IloNumExpr expr = cplex.numExpr();
            expr = cplex.sum(expr, towerRep2var.get(schedule));
            for (IloNumVar var : route2Towervars.get(schedule)) expr = cplex.sum(expr, cplex.prod(-1., var));
            cplex.addEq(expr, 0);
        }
    }

    private boolean solveMIP() throws IloException {
        cplex.setParam(IloCplex.Param.TimeLimit, 30);
        boolean fes = false;
        if (cplex.solve()) {
            fes = true;
            cost = cplex.getObjValue();
        }
        return fes;
    }

    private void setConverstion(boolean b) {
        shouldConvert = b;
        route2Towervars = new HashMap<>();
        towerRep2var = new HashMap<>();
    }

    public void convert() throws IloException {
        System.out.println("Converting the variables...");
        for (VarR var : getCrewVariables().values()) {
            cplex.add(cplex.conversion(var.getNumVar(), IloNumVarType.Int));
        }
        if (true) {
            for (int i = 0; i < data.getNodeNumber(); i++) {
                for (int j = 1; j < data.getNodeNumber() + 1; j++) {
                    if (i != j)
                        cplex.add(cplex.conversion(var_edge[i][j], IloNumVarType.Bool));
                }
            }
            for (int i = 1; i < data.getNodeNumber(); i++) {
                cplex.add(cplex.conversion(varY[i], IloNumVarType.Bool));
            }
        }
    }

    public void printAllColumns() {
        System.out.println("*".repeat(50));
        System.out.println("Printing all columns in the LP...");
        System.out.println("Crew Vars:");
        for (VarR var : getCrewVariables().values()) {
            System.out.println("\t" + var.getRoute().getPattern());
        }
        System.out.println("_".repeat(30));
        System.out.println("Tower Vars:");
        for (VarR var : getTowerVariables().values()) {
            System.out.println("\t" + var.getRoute().getPattern() + var.getRoute().getWaitBooleans());
        }
        System.out.println("_".repeat(30));
        System.out.println("*".repeat(50));
    }






    public void clear() {
        try {
            cplex.clearModel();
            cplex.endModel();
            cplex.end();

            if (outerColumnHashMap != null) {
                outerColumnHashMap.clear();
            }

            if (innerColumnHashMap != null) {
                innerColumnHashMap.clear();
            }

            if (innerColumnHashMapCrew != null)
                innerColumnHashMapCrew.clear();

            if (constraints != null) {
                constraints.clear();
            }

        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }






    public double count() {
        double[] count = new double[data.getNodeNumber() + 1];
        for (VarR var : towerVariableMap.values()) {
            Route route = var.getRoute();
            double v = var.getValue();
            ArrayList<Integer> schedule = route.getPattern();
            for (Integer i : schedule)
                count[i] += v;
        }
        double tots = 0.;
        for (int i = 1; i < data.getNodeNumber(); i++) {
//            System.out.println("y"+i+":"+count[i]);
            tots += count[i];
        }
        if (Constants.CONSOLE)
            System.out.println("Total Y:" + tots);
//        totsY = tots;
        return tots;
    }






    public void resetSolutionString() {
        solutionString = null;
    }






    public double[][] getDual0Psi() {
        return dual0_psi;
    }









    public boolean getMustToggle() {
        return disableNodeEliminationRule;
    }

    public void checkBranchingConstraints() throws IloException {
        {
            double[][] map = new double[data.getTasks()][data.getTasks() + 1];
            getMap(map, true);
            for (int i = 0; i < data.getTasks(); i++) {
                for (int j = 0; j < data.getTasks() + 1; j++) {
                    double v = map[i][j];
                    int outerValue = bNode.getOuterArcValue(i, j, true);
                    if (outerValue < 0) {
                        if (v > 1e-6) {
//                            Constants.CONSOLE = true;
                            printSolution();
                            bNode.printBranchStrings();
                            throw new IllegalArgumentException("infeasible arc traversal:" + i + "_" + j);
                        }
                    } else if (outerValue > 0) {
                        if (v < 1e-6) {
//                            Constants.CONSOLE = true;
                            printSolution();
                            bNode.printBranchStrings();
                            throw new IllegalArgumentException("required arc not traversed:" + i + "_" + j);
                        }
                    }
                }
            }
        }
        //--------------------------------------
        {
            double[][] map = new double[data.getNodeNumber()][data.getNodeNumber() + 1];
            getMap(map, false);
            for (int i = 0; i < data.getNodeNumber(); i++) {
                for (int j = 0; j < data.getNodeNumber() + 1; j++) {
                    double v = map[i][j];
                    int outerValue = bNode.getOuterArcValue(i, j, false);
                    if (outerValue < 0) {
                        if (v > 1e-6) {

                            printSolution();
                            bNode.printBranchStrings();
                            throw new IllegalArgumentException("infeasible arc traversal:" + i + "_" + j);
                        }
                    } else if (outerValue > 0) {
                        if (v < 1e-6) {

                            printSolution();
                            bNode.printBranchStrings();
                            throw new IllegalArgumentException("required arc not traversed:" + i + "_" + j);
                        }
                    }
                }
            }


            for (int i = 1; i < data.getNodeNumber(); i++) {
                double v = 0.;
                for (int j = 0; j < data.getNodeNumber() + 1; j++) {
                    v += map[i][j];
                }
                double lb = bNode.getPackingLB(i);
                double ub = bNode.getPackingUB(i);
                if (lb > 1e-6) {
                    if (v < 0.99) {

                        printSolution();
                        bNode.printBranchStrings();
                        throw new IllegalArgumentException("required zone not visited:" + i);
                    }
                }
                if (ub <= 1e-6) {
                    if (v > 1e-6) {

                        printSolution();
                        bNode.printBranchStrings();
                        throw new IllegalArgumentException("banned zone visited:" + i);
                    }
                }
            }
        }
        //--------------------------------------
        //--------------------------------------
    }





    public void checkRestoration() throws IloException {

        double[] taksCompletion = new double[data.getTasks() + 1];
        double[] restoration = new double[data.getNodeNumber()];
        for (VarR var : active_Cset) {
            Route route = var.getRoute();
            for (int j = 1; j < route.getPattern().size() - 1; j++) {
                int key = route.getPattern().get(j);
                taksCompletion[key] += (route.getArrivals().get(j) * var.getValue());
            }
        }
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double max = 0;
            for (Integer k : data.getZone2tasks()[i])
                max = Math.max(taksCompletion[k], max);
            restoration[i] = max;
        }

        boolean flag = false;
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double max = 0;
            for (Integer q : data.getZone2tasks()[i]) {
                double v = data.getCrewTravelTimeMatrix()[0][q] + data.getServiceTimeMatrix()[q];
                if (max < v)
                    max = v;
            }
            if (restoration[i] + 1e-6 < max) {
                System.out.println("Zone" + i + ":" + restoration[i] + "vs" + max);
                flag = true;
            }
        }

        for (int i = 1; i < data.getTasks(); i++) {
            double min;
            min = data.getCrewTravelTimeMatrix()[0][i];
            min += data.getServiceTimeMatrix()[i];
            if (taksCompletion[i] + 1e-6 < min) {
                System.out.println("Task" + i + ":" + restoration[i] + "vs" + min);
                flag = true;
            }
        }

        if (flag)
            throw new IllegalArgumentException("found cut!!!!");
    }


    public void checkPriorities() {
        if(Constants.COST_OF_PRIORITY_CREW) {
            for (VarR var : active_Cset) {
                Route route = var.getRoute();
                int from = route.getPattern().get(1);
                for (int k = 2; k < route.getPattern().size() - 1; k++) {
                    int to = route.getPattern().get(k);
                    if (data.getTaskPriority()[to] > data.getTaskPriority()[from])
                        throw new IllegalArgumentException("infeasible solution:" + route.getPattern().toString());
                    from = to;
                }
            }
        }


        if(Constants.COST_OF_PRIORITY_TOWER) {
            for (VarR var : active_Rset) {
                Route route = var.getRoute();
                int from = route.getPattern().get(1);
                for (int k = 2; k < route.getPattern().size() - 1; k++) {
                    int to = route.getPattern().get(k);
                    if (data.getZonePriority()[to] > data.getZonePriority()[from])
                        throw new IllegalArgumentException("infeasible solution:" + route.getPattern().toString());
                    from = to;
                }
            }
        }
    }

    private ICEAconstraint iceaConstrain;

    public void setICEAconsraint(ICEAconstraint uceaConstraintl) {
        boolean isThreadFiveOrGreater = uceaConstraintl != null && uceaConstraintl.id >= 5;
        setICEAconsraint(uceaConstraintl, isThreadFiveOrGreater);
    }

    public void setICEAconsraint(ICEAconstraint uceaConstraintl, boolean isThreadFiveOrGreater) {
        this.iceaConstrain = uceaConstraintl;
        this.isFifthThread = isThreadFiveOrGreater;
    }

    public boolean isFifthThread() {
        return isFifthThread;
    }

    public ICEAconstraint getIceaConstrain() {
        return iceaConstrain;
    }

    private boolean isExtraBound;

    public void setExtraBound(boolean extraBound) {
        isExtraBound = extraBound;
    }


    public double getLrangeUB() {
        return lrangeUB;
    }




    private SharedBounds sharedBound;
    public SharedBounds getSharedBound() {
        return sharedBound;
    }

    public void setSharedBound(SharedBounds sharedBound) {
        this.sharedBound = sharedBound;
    }

    //USED TO DISTINGUISH BETWEEN STRONG BRANCHING CHECKS AND ACTUAL COLUMN GENERATION
    private boolean exactGeneration;
    public void setExactGeneration(boolean b) {
        exactGeneration = b;
    }

    public boolean isExactGeneration(){
        return exactGeneration;
    }

    private boolean isFirstHeuristicPass;
    public void setFirstHeuristicPass(boolean b) {
        isFirstHeuristicPass = b;
    }

    public boolean isFirstHeuristicPass(){
        return isFirstHeuristicPass;
    }

    private double maxFTime, maxBTime, maxJTime;

    public double getMaxBTime() {
        return maxBTime;
    }

    public void setMaxBTime(double maxBTime) {
        this.maxBTime = maxBTime;
    }

    public double getMaxFTime() {
        return maxFTime;
    }

    public void setMaxFTime(double maxFTime) {
        this.maxFTime = maxFTime;
    }

    public double getMaxJTime() {
        return maxJTime;
    }

    public void setMaxJTime(double maxJTime) {
        this.maxJTime = maxJTime;
    }

    public double getYValue(int i) {
        if (active_a_values == null) {
            throw new IllegalStateException(
                    "active_a_values is null. Call lp.solve() before reading y values."
            );
        }

        return active_a_values[i];
    }

}
