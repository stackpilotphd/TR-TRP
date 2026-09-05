package problem.BP;

import lib.StaticSharedValues;
import lib.TimerHelper;
import lib.Utility;
import problem.Constants;
import problem.graph.Data;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.PriorityQueue;


public class TowerPulse {

    private static final double EPS = 1e-6;

    private double[][] matrix;
    private int depotEnd;
    private Data data;
    private LP lp;

    private double gbl_best_cost;
    private boolean termination;
    private boolean isBounding;

    private int maxQ;
    private double serviceDuration;
    private double[] dualDifferences;

    private boolean[] tabu;
    private boolean[] visited;

    private int[] pathStack;
    private double[][] chiStack;
    private double[][] tauStack;

    private TimerHelper timerHelper;

    private PriorityQueue<ColumnCandidate> bestColumns;
    private HashSet<String> keptColumnKeys;

    private int maxReturnedColumns;
    private int maxKeptColumns;
    private int acceptedColumnCount;

    /**
     * Main entry point.
     */
    public ArrayList<Route> run(LP lp, boolean isHeuristic) {

        resetState();

        if (Utility.algo == 1) {
            timerHelper = TimerHelper.getInstance();
        }

        this.lp = lp;
        this.data = Data.getInstance();

        if (data.getTowerNumber() <= 0) {
            return new ArrayList<>();
        }

        depotEnd = data.getDepotEnd(false);

        int N = data.getNodeNumber();

        tabu = new boolean[N + 1];
        visited = new boolean[N + 1];

        if (isHeuristic) {
            boolean usefulDualFound = initializeTabuForHeuristic();
            if (!usefulDualFound) {
                return new ArrayList<>();
            }
        }

        if (Utility.algo == 1) {
            maxQ = StaticSharedValues.maximumTowerRouteLength;
        } else {
            maxQ = (int) lp.getLrangeUB() - data.getTowerNumber() + 1;
        }

        if (maxQ <= 0) {
            return new ArrayList<>();
        }

        dualDifferences = new double[N + 1];

        for (int i = 1; i < N; i++) {
            dualDifferences[i] = lp.getDualSigma()[i] - lp.getDualPhi()[i];
        }

        serviceDuration = data.getServiceM();

        maxReturnedColumns = Math.max(1, Constants.MAX_COL);


        maxKeptColumns = Math.max(maxReturnedColumns, 5 * maxReturnedColumns);

        bestColumns = new PriorityQueue<>(
                Comparator.comparingDouble(c -> c.reducedCost)
        );

        keptColumnKeys = new HashSet<>(2 * maxKeptColumns);

        pathStack = new int[maxQ + 3];


        chiStack = new double[maxQ + 3][maxQ + 1];
        tauStack = new double[maxQ + 3][maxQ + 1];

        bound2();

        if (shouldStopByTime()) {
            return buildReturnedRoutes();
        }

        gbl_best_cost = 0.0;

        double[] rootChi = new double[maxQ + 1];
        double[] rootTau = new double[maxQ + 1];

        for (int start = 1; start < N; start++) {

            if (shouldStopByTime()) {
                break;
            }

            if (tabu[start]) {
                continue;
            }

            Arrays.fill(rootChi, lp.getDualVisitR()[0]);
            Arrays.fill(rootTau, 0.0);

            Arrays.fill(visited, false);

            pathStack[0] = 0;

            pulse2(
                    start,
                    lp.getDualVisitR()[0],
                    rootChi,
                    rootTau,
                    0,
                    1,
                    maxQ
            );

            if (termination) {
                break;
            }
        }

        StaticSharedValues.pulse_iterations_tower++;

        return buildReturnedRoutes();
    }


    private void resetState() {
        matrix = null;
        data = null;
        lp = null;

        gbl_best_cost = 0.0;
        termination = false;
        isBounding = false;

        maxQ = 0;
        serviceDuration = 0.0;
        dualDifferences = null;

        tabu = null;
        visited = null;

        pathStack = null;
        chiStack = null;
        tauStack = null;

        timerHelper = null;

        bestColumns = null;
        keptColumnKeys = null;

        maxReturnedColumns = 0;
        maxKeptColumns = 0;
        acceptedColumnCount = 0;
    }


    private boolean initializeTabuForHeuristic() {

        boolean found = false;

        for (int i = 1; i < data.getNodeNumber(); i++) {

            double dualVisit = lp.getDualVisitR()[i];

            if (dualVisit <= EPS) {

                double towerReward = lp.getDualSigma()[i] - lp.getDualPhi()[i];

                if (towerReward <= EPS) {
                    tabu[i] = true;
                } else {
                    found = true;
                }

            } else {
                found = true;
            }
        }

        if (!found) {
            double depotDual = lp.getDualVisitR()[0];

            if (depotDual < EPS && depotDual > -EPS) {
                return false;
            }
        }

        return true;
    }


    private void bound2() {

        int nodeNumber = data.getNodeNumber();

        matrix = new double[nodeNumber + 1][maxQ + 1];

        for (int i = 0; i < matrix.length; i++) {
            Arrays.fill(matrix[i], Double.MAX_VALUE);
        }

        if (maxQ <= 1) {
            return;
        }

        isBounding = true;

        try {

            int lb = (int) Math.ceil(maxQ / 2.0);

            double[] rootChi = new double[maxQ + 1];
            double[] rootTau = new double[maxQ + 1];

            for (int maxRouteLength = 1; maxRouteLength < lb + 1; maxRouteLength++) {

                StaticSharedValues.bound_iterations_tower++;

                for (int start = 1; start < nodeNumber; start++) {

                    if (shouldStopByTime()) {
                        return;
                    }

                    gbl_best_cost = -Double.MAX_VALUE;

                    Arrays.fill(rootChi, 0.0);
                    Arrays.fill(rootTau, 0.0);
                    Arrays.fill(visited, false);

                    pathStack[0] = start;
                    visited[start] = true;

                    for (int next = 1; next < nodeNumber + 1; next++) {
                        pulse2(
                                next,
                                0.0,
                                rootChi,
                                rootTau,
                                0,
                                1,
                                maxRouteLength
                        );

                        if (termination || shouldStopByTime()) {
                            return;
                        }
                    }

                    visited[start] = false;
                    matrix[start][maxRouteLength] = gbl_best_cost;
                }
            }

        } finally {
            isBounding = false;
        }
    }


    private void pulse2(
            int w,
            double cost,
            double[] chi,
            double[] tau,
            int q,
            int depth,
            int maxRouteLen
    ) {

        if (termination) {
            return;
        }

        if (shouldStopByTime()) {
            return;
        }

        if (w != depotEnd) {
            if (w < 0 || w >= visited.length) {
                return;
            }

            if (visited[w]) {
                return;
            }

            if (w < tabu.length && tabu[w]) {
                return;
            }
        }

        int v = pathStack[depth - 1];

        if (!lp.isFeasibleArc(v, w, false)) {
            return;
        }

        if (violatesZonePriority(v, w)) {
            return;
        }

        StaticSharedValues.pulse_extensions_tower++;

        /*
         * Depot completion case.
         * No need to allocate/copy chi/tau arrays here.
         */
        if (w == depotEnd) {
            handleDepotCompletion(cost, chi, depth);
            return;
        }

        int q2 = q + 1;

        if (q2 > maxRouteLen || q2 > maxQ) {
            return;
        }

        /*
         * Add w to the current path.
         */
        visited[w] = true;
        pathStack[depth] = w;

        try {

            double[] nextChi = chiStack[depth + 1];
            double[] nextTau = tauStack[depth + 1];

            /*
             * Only entries 0,...,q2 are relevant.
             */
            for (int k = 0; k <= q2; k++) {
                nextChi[k] = 0.0;
                nextTau[k] = 0.0;
            }

            double deltaTime =
                    data.getPositionTimeMatrix()[w]
                            + data.getTowerTravelTimeMatrix()[v][w];

            double constantDual = lp.getDualVisitR()[w];
            double sigma = lp.getDualSigma()[w];
            double serviceReward = dualDifferences[w];

            /*
             * Case 1: zero zones are serviced.
             */
            nextTau[0] = tau[0] + deltaTime;
            nextChi[0] = chi[0] + constantDual + sigma * nextTau[0];

            /*
             * Case 2: intermediate numbers of serviced zones.
             */
            for (int k = 1; k <= q; k++) {

                nextTau[k] = tau[k] + deltaTime;

                double notServeW =
                        chi[k]
                                + constantDual
                                + sigma * nextTau[k];

                double serveW =
                        chi[k - 1]
                                + constantDual
                                + sigma * (tau[k - 1] + deltaTime)
                                + serviceDuration * serviceReward;

                nextChi[k] = Math.max(notServeW, serveW);
            }

            /*
             * Case 3: all zones are serviced.
             */
            nextTau[q2] = tau[q] + deltaTime + serviceDuration;
            nextChi[q2] =
                    chi[q]
                            + constantDual
                            + sigma * (tau[q] + deltaTime)
                            + serviceDuration * serviceReward;

            double c2 = nextChi[0];

            for (int k = 1; k <= q2; k++) {
                c2 = Math.max(c2, nextChi[k]);
            }

            int remaining = maxQ - q2;

            if (remaining < 0) {
                return;
            }

            double suffixBound = matrix[w][remaining];

            if (c2 + suffixBound <= gbl_best_cost - EPS) {

                if (!isBounding) {

                    if (c2 > EPS || c2 > gbl_best_cost) {
                        pulse2(
                                depotEnd,
                                c2,
                                nextChi,
                                nextTau,
                                q2,
                                depth + 1,
                                maxRouteLen
                        );
                    }

                } else {
                    StaticSharedValues.bound_pruning_towers++;
                }

                return;
            }

            /*
             * Continue extending the route if route length permits.
             */
            if (q2 < maxRouteLen) {

                for (int next = 1; next < data.getNodeNumber(); next++) {
                    pulse2(
                            next,
                            c2,
                            nextChi,
                            nextTau,
                            q2,
                            depth + 1,
                            maxRouteLen
                    );

                    if (termination || shouldStopByTime()) {
                        return;
                    }
                }
            }

            /*
             * Always allow route completion at the depot.
             */
            pulse2(
                    depotEnd,
                    c2,
                    nextChi,
                    nextTau,
                    q2,
                    depth + 1,
                    maxRouteLen
            );

        } finally {
            visited[w] = false;
        }
    }

    /**
     * Handles closing a path at the depot.
     */
    private void handleDepotCompletion(double cost, double[] chi, int depth) {

        double c2 = cost;

        if (c2 > gbl_best_cost) {
            gbl_best_cost = c2;
        }

        if (isBounding) {
            return;
        }

        if (c2 <= EPS) {
            return;
        }

        /*
         * Temporarily append depot to the path stack.
         */
        pathStack[depth] = depotEnd;

        int pathLength = depth + 1;
        int innerLength = pathLength - 2;

        if (innerLength <= 0) {
            return;
        }


        if (innerLength > 62) {
            throw new IllegalStateException(
                    "Tower route has more than 62 internal nodes. "
                            + "Replace long serviceMask with BitSet to support this case."
            );
        }

        long numberOfMasks = 1L << innerLength;

        for (long serviceMask = 0L; serviceMask < numberOfMasks; serviceMask++) {

            if (termination || shouldStopByTime()) {
                return;
            }

            int truthCount = Long.bitCount(serviceMask);

            if (truthCount >= chi.length) {
                continue;
            }


            if (chi[truthCount] <= EPS) {
                continue;
            }

            double reducedCost = computeReducedCostForMask(pathLength, serviceMask);

            if (reducedCost > EPS) {
                tryAddColumn(pathLength, serviceMask, reducedCost);
            }
        }
    }

    /**
     * Computes the reduced cost of the current pathStack[0..pathLength-1]
     * under a particular service mask.
     *
     * Mask convention:
     * - bit 0 corresponds to path position 1;
     * - bit 1 corresponds to path position 2;
     * - ...
     * - depot at the final position is never served.
     */
    private double computeReducedCostForMask(int pathLength, long serviceMask) {

        double reducedCost = lp.getDualVisitR()[0];
        double time = 0.0;

        int from = pathStack[0];

        for (int pos = 1; pos < pathLength; pos++) {

            int to = pathStack[pos];

            double ti2 =
                    time
                            + data.getTowerTravelTimeMatrix()[from][to]
                            + data.getPositionTimeMatrix()[to];

            double ci2 =
                    reducedCost
                            + lp.getDualVisitR()[to]
                            + lp.getDualSigma()[to] * ti2;

            boolean served = false;

            /*
             * Internal positions are 1,...,pathLength-2.
             * Final position is the depot and should not be served.
             */
            if (pos <= pathLength - 2) {
                served = ((serviceMask & (1L << (pos - 1))) != 0L);
            }

            if (served) {
                ti2 += serviceDuration;
                ci2 += serviceDuration * dualDifferences[to];
            }

            reducedCost = ci2;
            time = ti2;
            from = to;
        }

        return reducedCost;
    }

    /**
     * Adds a candidate column only if it belongs among the currently best
     * retained columns.
     */
    private void tryAddColumn(int pathLength, long serviceMask, double reducedCost) {

        if (reducedCost <= EPS) {
            return;
        }

        /*
         * If the heap is already full and this candidate is no better than
         * the worst retained candidate, skip it before creating objects.
         */
        if (bestColumns.size() >= maxKeptColumns
                && reducedCost <= bestColumns.peek().reducedCost + EPS) {
            return;
        }

        int[] savedPath = Arrays.copyOf(pathStack, pathLength);
        String key = buildColumnKey(savedPath, pathLength, serviceMask);

        if (keptColumnKeys.contains(key)) {
            return;
        }

        ArrayList<Integer> schedule = buildSchedule(savedPath, pathLength);
        ArrayList<Boolean> waitBooleans = buildWaitBooleans(pathLength, serviceMask);



        if (lp.r2i(schedule, waitBooleans) != null) {
            return;
        }

        ColumnCandidate candidate =
                new ColumnCandidate(savedPath, pathLength, serviceMask, reducedCost, key);

        if (bestColumns.size() < maxKeptColumns) {
            bestColumns.add(candidate);
            keptColumnKeys.add(key);
        } else {
            ColumnCandidate removed = bestColumns.poll();

            if (removed != null) {
                keptColumnKeys.remove(removed.key);
            }

            bestColumns.add(candidate);
            keptColumnKeys.add(key);
        }

        acceptedColumnCount++;


        int maxAcceptedBeforeStop = Math.max(5000, 10 * maxKeptColumns);

        if (acceptedColumnCount >= maxAcceptedBeforeStop) {
            termination = true;
        }
    }

    /**
     * Converts retained candidates into Route objects and returns the best MAX_COL.
     */
    private ArrayList<Route> buildReturnedRoutes() {

        ArrayList<ColumnCandidate> kept = new ArrayList<>();

        if (bestColumns != null) {
            kept.addAll(bestColumns);
        }

        kept.sort((a, b) -> -Double.compare(a.reducedCost, b.reducedCost));

        ArrayList<Route> selected = new ArrayList<>();

        int limit = Math.min(maxReturnedColumns, kept.size());

        for (int idx = 0; idx < limit; idx++) {

            ColumnCandidate candidate = kept.get(idx);

            ArrayList<Integer> schedule =
                    buildSchedule(candidate.path, candidate.pathLength);

            ArrayList<Boolean> waitBooleans =
                    buildWaitBooleans(candidate.pathLength, candidate.serviceMask);

            Route route = new Route();
            route.create(schedule, waitBooleans);
            route.setPseudo_cost(candidate.reducedCost);

            selected.add(route);
        }

        /*
         * Help GC if this object remains alive.
         */
        if (bestColumns != null) {
            bestColumns.clear();
        }

        if (keptColumnKeys != null) {
            keptColumnKeys.clear();
        }

        return selected;
    }

    private ArrayList<Integer> buildSchedule(int[] path, int pathLength) {

        ArrayList<Integer> schedule = new ArrayList<>(pathLength);

        for (int i = 0; i < pathLength; i++) {
            schedule.add(path[i]);
        }

        return schedule;
    }

    /**
     * Builds the Boolean list expected by Route.create(...).
     *
     * Convention:
     * - list size equals pathLength;
     * - index 0 is depot/start and is false;
     * - internal customer positions may be true/false based on serviceMask;
     * - final depot index is false.
     */
    private ArrayList<Boolean> buildWaitBooleans(int pathLength, long serviceMask) {

        ArrayList<Boolean> waitBooleans = new ArrayList<>(pathLength);

        for (int i = 0; i < pathLength; i++) {
            waitBooleans.add(false);
        }

        for (int pos = 1; pos <= pathLength - 2; pos++) {
            boolean served = ((serviceMask & (1L << (pos - 1))) != 0L);
            waitBooleans.set(pos, served);
        }

        return waitBooleans;
    }

    private String buildColumnKey(int[] path, int pathLength, long serviceMask) {

        StringBuilder sb = new StringBuilder(pathLength * 4 + 24);

        for (int i = 0; i < pathLength; i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append(path[i]);
        }

        sb.append('|').append(serviceMask);

        return sb.toString();
    }

    private boolean violatesZonePriority(int from, int to) {

        if (!Constants.COST_OF_PRIORITY_TOWER) {
            return false;
        }

        int[] priority = data.getZonePriority();

        if (priority == null) {
            return false;
        }

        if (from < 0 || to < 0 || from >= priority.length || to >= priority.length) {
            return false;
        }

        int priorityFrom = priority[from];
        int priorityTo = priority[to];

        return priorityTo > priorityFrom;
    }

    private boolean shouldStopByTime() {

        return Utility.algo == 1
                && timerHelper != null
                && timerHelper.hasTimedOut();
    }

    /**
     * Lightweight retained column.
     */
    private static class ColumnCandidate {

        final int[] path;
        final int pathLength;
        final long serviceMask;
        final double reducedCost;
        final String key;

        ColumnCandidate(
                int[] path,
                int pathLength,
                long serviceMask,
                double reducedCost,
                String key
        ) {
            this.path = path;
            this.pathLength = pathLength;
            this.serviceMask = serviceMask;
            this.reducedCost = reducedCost;
            this.key = key;
        }
    }
}
