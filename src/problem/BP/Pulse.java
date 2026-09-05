package problem.BP;

import lib.*;
import problem.Constants;
import problem.graph.Data;

import java.util.*;

public class Pulse {

    private double[][] matrix;
    private int depotEnd;
    private Data data;
    private LP lp;
    private ArrayList<Integer> gbl_p;
    private double gbl_best_cost;
    private HashSet<ArrayList<Integer>> reduced_cost_paths;
    private int max_col;
    private boolean termination;
    private double[] arrayForCrews;
    private long[][] i2t2nI;
    private boolean heuristic;
    private ArrayList<Boolean> w2;


    private double overallBestCost;
    private ArrayList<Integer> overallBestPath;

    private long[][] matrixForNodeElimination;

    private double[] nodeRestorationSplitDuals;




    /*
     * When true, the pulse is computing the matrix bound.
     * In that case we deliberately exclude node-restoration split duals.
     */
    private boolean buildingBound;


    private TimerHelper timerHelper;
    public ArrayList<Route> run(LP lp, boolean isHeuristic) {
//        System.out.println("Started crew pulse");
        data = Data.getInstance();
        if(data.getCrewNumber() <=0 || Utility.algo == 22)
            return new ArrayList<>();
        this.w2 = new ArrayList<>();
        this.lp = lp;
        this.heuristic = isHeuristic;
        if(Utility.algo == 1) {
            timerHelper = TimerHelper.getInstance();
        }
        depotEnd = data.getDepotEnd(true);
        reduced_cost_paths = new HashSet<>();
        this.max_col = Constants.MAX_COL;
        arrayForCrews = new double[data.getTasks()+1];
        for (int i = 1; i < data.getNodeNumber(); i++) for(Integer q : data.getZone2tasks()[i]) arrayForCrews[q] += lp.getDual0Psi()[i][q];


        //TIME PROCESSING FOR DP
        if(isHeuristic){
            double start = System.nanoTime();
            int step = 1;
            int horizon = (int) (data.getHorizon()*step);
            i2t2nI = new long[data.getTasks()+1][horizon+1];
            for (int i = 1; i < data.getTasks(); i++) {
                long[] longsOfI = new long[horizon+1];
                for (int t = 1; t < horizon+1; t++) {
                    long unreachable = 0x00L | (1L << i);
                    for (int j = 1; j < data.getTasks(); j++) {
                        if(lp.getMustVisitNodesDueToBranching()[j])
                            continue;
                        double v1 = lp.getDualVisitC()[j];
                        double v2 = arrayForCrews[j];
                        if(v2 < -1e-6){
                            double real = t * step + data.getCrewTravelTimeMatrix()[i][j] + data.getServiceTimeMatrix()[j];
                            double optimisticGain = v1 + v2 * real;

                            if (optimisticGain < -1e-6) {
                                unreachable = unreachable | (1L << j);
                            }
                        }
//                        }
                    }
                    longsOfI[t] = unreachable | data.getCrewUnreachableI2T2set()[i][t];
                }
                i2t2nI[i] = longsOfI;
            }
            double passed = (System.nanoTime() - start)*1e-9;
        } else
            i2t2nI = data.getCrewUnreachableI2T2set();

        bound();
        double cost = lp.getDualVisitC()[0];
        double time = 0;
        gbl_p = new ArrayList<>();
//        gbl_best_cost = -1. * Double.MAX_VALUE;
        gbl_best_cost = 0; Msg.todo("25,08,22");
        for (int i = 1; i < data.getTasks(); i++) {
            if(Utility.algo==1){
                if(timerHelper.hasTimedOut())
                    break;
            }

            ArrayList<Integer> p = new ArrayList<>();
            p.add(0);
            if(Constants.ROBUST) {
                pulseRobust(
                        i,
                        cost,
                        time,
                        new double[StaticSharedValues.budget],
                        p,
                        0x00L

                );
            }
            else {
                pulse(i, cost, time, p, 0x00L);
            }
        }




        StaticSharedValues.pulse_iterations_crew++;

//        System.out.println("Over crew pulse");
        {
            //BASE CASE
            HashSet<ArrayList<Integer>> scheduleHash = new HashSet<>();
            ArrayList<Boolean> waitBooleans = new ArrayList<>();
            ArrayList<Route> routes = new ArrayList<>();
            for(ArrayList<Integer> schedule : reduced_cost_paths) {
                if(!scheduleHash.contains(schedule)) {
                    scheduleHash.add(schedule);
                    if(lp.r2i(schedule,waitBooleans) == null){
                        Route route = new Route();
                        route.create(schedule,waitBooleans);
                        routes.add(route);
                    }
                }
            }
            return routes;
        }
    }




    private void bound() {
        buildingBound = true;

        try {
            int steps = (int) Math.ceil((data.getHorizon() / Constants.DELTA_PULSE));
            matrix = new double[data.getTasks()][steps + 1];

            for (int i = 1; i < data.getTasks(); i++) {
                for (int j = 0; j < matrix[i].length; j++) {
                    matrix[i][j] = Double.MAX_VALUE;
                }
            }

            matrixForNodeElimination = new long[data.getTasks()][steps + 1];

            int t = (int) data.getHorizon();

            while (t > Constants.LOWER_BOUND_PULSE) {
                t = t - Constants.DELTA_PULSE;
                StaticSharedValues.bound_iterations_crew++;

                for (int i = 1; i < data.getTasks(); i++) {
                    if (Utility.algo == 1) {
                        if (timerHelper.hasTimedOut()) {
                            return;
                        }
                    }

                    gbl_p = new ArrayList<>();
                    gbl_best_cost = -1. * 99999999;

                    double cost = 0;
                    double time = t;
                    long v1 = (1L << i);

                    for (int j = 1; j < data.getTasks() + 1; j++) {
                        ArrayList<Integer> p = new ArrayList<>();
                        p.add(i);

                        if (Constants.ROBUST) {
                            pulseRobust(
                                    j,
                                    cost,
                                    time,
                                    new double[StaticSharedValues.budget],
                                    p,
                                    v1
                            );
                        } else {
                            pulse(j, cost, time, p, v1);
                        }
                    }

                    int index = (int) Math.floor(t / (double) Constants.DELTA_PULSE);
                    matrix[i][index] = gbl_best_cost;

                    if (Constants.USE_NODE_ELIMINATION_CREW_PULSE) {
                        if (!lp.getMustToggle()) {
                            long unreachable = 0x00L;

                            for (int j = 1; j < data.getTasks(); j++) {
                                if (i == j) {
                                    continue;
                                }

                                double val1 = lp.getDualVisitC()[j];
                                double v2 = arrayForCrews[j];
                                double real =
                                        t
                                                + data.getCrewTravelTimeMatrix()[i][j]
                                                + data.getServiceTimeMatrix()[j];

                                /*
                                 * Important:
                                 * A task with bad ordinary reduced-cost contribution
                                 * may still unlock a positive split reward.
                                 */
                                double optimisticTaskGain =
                                        val1
                                                + v2 * real
                                                ;

                                if (optimisticTaskGain < -1e-6) {
                                    unreachable = unreachable | (1L << j);
                                }
                            }

                            matrixForNodeElimination[i][index] = unreachable;
                        }
                    }
                }
            }
        } finally {
            buildingBound = false;
        }
    }

    private void pulse(
            int w,
            double cost,
            double time,
            ArrayList<Integer> p,
            long vI
    ) {
        if (termination) {
            return;
        }

        if ((vI & (1L << w)) != 0) {
            return;
        }

        int v = p.getLast();

        if (!(lp.isFeasibleArc(v, w, true))) {
            return;
        }

        if (Utility.algo == 41 || Constants.COST_OF_PRIORITY || Constants.COST_OF_PRIORITY_CREW) {
            int priorityFrom = data.getTaskPriority()[v];
            int priorityTo = data.getTaskPriority()[w];

            if (priorityTo > priorityFrom) {
                return;
            }
        }

        double t2 =
                time
                        + data.getCrewTravelTimeMatrix()[v][w]
                        + data.getServiceTimeMatrix()[w];




        double c2 =
                cost
                        + lp.getDualVisitC()[w]
                        + t2 * arrayForCrews[w]
                        ;

        StaticSharedValues.pulse_extensions_crew++;

        if (w == depotEnd) {
            if (c2 > gbl_best_cost) {
                gbl_best_cost = c2;
                gbl_p.clear();
                gbl_p = new ArrayList<>(p);
                gbl_p.add(w);
            }

            if (p.getFirst() == 0) {
                if (c2 > 1e-6) {
                    ArrayList<Integer> p2 = new ArrayList<>(p);
                    p2.add(w);

                    if (lp.r2i(p2, w2) == null && !reduced_cost_paths.contains(p2)) {
                        reduced_cost_paths.add(p2);

                        if (reduced_cost_paths.size() >= max_col) {
                            termination = true;
                        }
                    }
                }
            }

            return;
        }

        if (t2 + data.getCrewTravelTimeMatrix()[w][depotEnd] > data.getHorizon()) {
            return;
        }

        int index = (int) Math.floor(t2 / (double) Constants.DELTA_PULSE);

        ArrayList<Integer> p2 = new ArrayList<>(p);
        p2.add(w);

        long v2 = vI | (1L << w);

        /*
         * Feillet acceleration and node elimination.
         * These are safe only if their construction accounts for possible
         * positive split rewards. See the modifications in bound().
         */
        v2 = v2 | i2t2nI[w][(int) t2];
        v2 = v2 | matrixForNodeElimination[w][index];


        if (heuristic) {
            if (c2  < cost - 1e-6) {
                return;
            }
        }



        double optimisticUpperBound =
                c2 + matrix[w][index];

        if (optimisticUpperBound <= gbl_best_cost + 1e-6) {
            if (c2 > gbl_best_cost + 1e-6 || c2 > 1e-6) {
                pulse(depotEnd, c2, t2, p2, v2);
                return;
            }

            StaticSharedValues.bound_pruning_crews++;
            return;
        }

        for (int i = 1; i < data.getTasks() + 1; i++) {
            pulse(i, c2, t2, p2, v2);
        }
    }

    private void pulseRobust(
            int w,
            double cost,
            double time,
            double[] r_t,
            ArrayList<Integer> p,
            long vI
    ) {
        if (termination) {
            return;
        }

        if ((vI & (1L << w)) != 0) {
            return;
        }

        int v = p.getLast();

        if (!(lp.isFeasibleArc(v, w, true))) {
            return;
        }

        if (Utility.algo == 41 || Constants.COST_OF_PRIORITY || Constants.COST_OF_PRIORITY_CREW) {
            int priorityFrom = data.getTaskPriority()[v];
            int priorityTo = data.getTaskPriority()[w];

            if (priorityTo > priorityFrom) {
                return;
            }
        }

        StaticSharedValues.pulse_extensions_crew++;

        double t2;
        double c2;

        double t1 =
                data.getCrewTravelTimeMatrix()[v][w]
                        + data.getServiceTimeMatrix()[w];

        double[] arr_t = new double[StaticSharedValues.budget];

        arr_t[0] = r_t[0] + t1;

        if (Constants.R_TOGGLE == 0) {
            for (int k = 1; k < arr_t.length; k++) {
                arr_t[k] =
                        Math.max(
                                r_t[k] + t1,
                                r_t[k - 1]
                                        + t1
                                        + data.getServiceDeviation()[w]
                        );
            }
        } else {
            for (int k = 1; k < arr_t.length; k++) {
                arr_t[k] =
                        Math.max(
                                r_t[k] + t1,
                                r_t[k - 1]
                                        + t1
                                        + data.getCrewTravelTimeDeviations()[v][w]
                        );
            }
        }

        t2 =
                time
                        + data.getCrewTravelTimeMatrix()[v][w]
                        + data.getServiceTimeMatrix()[w];

        for (int k = 0; k < arr_t.length; k++) {
            t2 = Math.max(t2, arr_t[k]);
        }




        c2 =
                cost
                        + lp.getDualVisitC()[w]
                        + t2 * arrayForCrews[w]
                        ;

        if (w == depotEnd) {
            if (c2 > gbl_best_cost) {
                gbl_best_cost = c2;
                gbl_p.clear();
                gbl_p = new ArrayList<>(p);
                gbl_p.add(w);
            }

            if (p.getFirst() == 0) {
                if (c2 > 1e-6) {
                    ArrayList<Integer> p2 = new ArrayList<>(p);
                    p2.add(w);

                    if (lp.r2i(p2, w2) == null && !reduced_cost_paths.contains(p2)) {
                        reduced_cost_paths.add(p2);

                        if (reduced_cost_paths.size() >= max_col) {
                            termination = true;
                        }
                    }
                }
            }

            return;
        }

        if (t2 + data.getCrewTravelTimeMatrix()[w][depotEnd] > data.getHorizon()) {
            return;
        }

        int index = (int) Math.floor(t2 / (double) Constants.DELTA_PULSE);

        ArrayList<Integer> p2 = new ArrayList<>(p);
        p2.add(w);

        long v2 = vI | (1L << w);
        v2 = v2 | i2t2nI[w][(int) t2];
        v2 = v2 | matrixForNodeElimination[w][index];

        if (heuristic) {

            if (c2  < cost - 1e-6) {
                return;
            }
        }



        double optimisticUpperBound =
                c2 + matrix[w][index];

        if (optimisticUpperBound <= gbl_best_cost + 1e-6) {
            StaticSharedValues.bound_pruning_crews++;
            return;
        }

        for (int i = 1; i < data.getTasks() + 1; i++) {
            pulseRobust(i, c2, t2, arr_t, p2, v2);
        }
    }
}
