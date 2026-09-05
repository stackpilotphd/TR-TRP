package problem.BP;

import ilog.concert.IloException;
import lib.StaticSharedValues;
import problem.Constants;
import problem.graph.Data;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Bidirectional labeling-based DP for tower-route pricing.
 */
public class DPT {

    private static final double EPS = 1e-6;
    private static final double INF = 1e100;
    private static final double BACKWARD_START_TIME = 1e9;

    private final Data data;
    private final int depotEnd;
    private final int N;
    private final double[] positionTime;
    private final double[][] travelTime;
    private final double serviceDuration;

    private LP lp;
    private boolean heuristic;

    private ArrayList<Route> routes;

    private double[] dualDifferences;
    private double[] dualArrival;
    private double[] dualVisit;
    private double[] dualService;
    private double[] dualRouteSize;

    private int maxTowerRouteLength;
    private int effectiveMaxCustomers;
    private int forwardMaxCustomers;
    private int backwardMaxCustomers;
    private boolean isNonElementary;

    private boolean[] tabu;
    private int[][] successors;
    private int[][] predecessors;

    private int overTheTop;
    private int thresholdForEarlyTermination;
    private boolean earlyTermination;

    private SkylineBucket[][] forwardPermanentLabels;
    private SkylineBucket[][] backwardPermanentLabels;

    private PriorityQueue<Label> forwardOpenQueue;
    private PriorityQueue<Label> backwardOpenQueue;

    /** Completed columns from forward depot closure, backward depot closure, or joining. */
    private PriorityQueue<Label> completedTopLabels;

    private int maxRoutesToReturn;
    private int maxCompletedLabelsToKeep;
    private long customerMask;


    private double maxFTime, maxBTime, maxJTime;
    private final ArrayList<Label> dominanceMarkedForRemoval = new ArrayList<>(1024);
    private long generatedForwardLabels;
    private long generatedBackwardLabels;
    private long acceptedForwardLabels;
    private long acceptedBackwardLabels;
    private long rejectedByDominance;
    private long dominanceComparisons;
    private long labelsRemovedByDominance;
    private int maxBucketSize;



    private static final ConcurrentHashMap<Integer, Integer> adaptiveForwardSplitByLength =
            new ConcurrentHashMap<>();

    private static final double SPLIT_IMBALANCE_TRIGGER = 2.0;
    private static final int SPLIT_ADJUSTMENT_STEP = 1;

    public DPT() {
        this.data = Data.getInstance();
        this.depotEnd = data.getDepotEnd(false);
        this.N = data.getNodeNumber();
        this.positionTime = data.getPositionTimeMatrix();
        this.travelTime = data.getTowerTravelTimeMatrix();
        this.serviceDuration = data.getServiceM();

        if (N + 1 >= 63) {
            throw new IllegalStateException(
                    "DPT26Bidir uses a long visited mask. The node index must be <= 62. "
                            + "For larger instances, replace the long mask with BitSet."
            );
        }
    }

    public void run(LP lp, boolean isHeuristic) throws IloException {


        this.maxFTime = lp.getMaxFTime();
        this.maxBTime = lp.getMaxBTime();
        this.maxJTime = lp.getMaxJTime();

        this.lp = lp;
        this.heuristic = isHeuristic;



        this.dualVisit = lp.getDualVisitR();
        this.dualArrival = lp.getDualSigma();
        this.dualService = lp.getDualPhi();
        this.dualDifferences = new double[N + 1];
        for (int i = 1; i < N; i++) {
            dualDifferences[i] = dualArrival[i] - dualService[i];
        }

        this.maxTowerRouteLength = StaticSharedValues.maximumTowerRouteLength;

        if (Constants.NON_ELEMENTARY_TOWER) {
            if (isHeuristic) {
                this.isNonElementary = !lp.isFirstHeuristicPass();
            } else {
                this.isNonElementary = true;
            }
        } else {
            this.isNonElementary = false;
        }

        if (isNonElementary) {
            effectiveMaxCustomers = Math.max(0, maxTowerRouteLength);
        } else {
            effectiveMaxCustomers = Math.min(Math.max(0, maxTowerRouteLength), N - 1);
        }

        resetDPStats();

        this.forwardMaxCustomers = effectiveMaxCustomers / 2;
        this.backwardMaxCustomers = effectiveMaxCustomers - forwardMaxCustomers;
//        chooseAdaptiveSplit();

        this.maxRoutesToReturn = Math.max(1, Constants.MAX_COL);
        this.maxCompletedLabelsToKeep = maxRoutesToReturn;
        this.thresholdForEarlyTermination = maxCompletedLabelsToKeep * 10;
        this.overTheTop = 0;
        this.earlyTermination = false;

        this.routes = new ArrayList<>(maxRoutesToReturn);

        if (effectiveMaxCustomers <= 0) {
            return;
        }

        initializeCustomerMask();
        initializeTabu();
        buildSuccessors();
        buildPredecessors();
        initializeLabelBuckets();

        boolean isOver = false;
        HashSet<String> selectedInThisPricingCall = new HashSet<>();

        double startF = System.nanoTime();
        /* 1. Forward half: run the forward queue to completion, then try depot closure. */
        runForwardLabelingDP();
        closeForwardLabelsToEndDepot();
        if (hasEnoughCompletedColumns()) {
            buildReturnedRoutes(selectedInThisPricingCall);
            isOver=true;
        }

        double end = (System.nanoTime()-startF)*1e-9;
        if(Constants.CONSOLE) System.out.println("Forward Time:"+end);
        this.maxFTime = Math.max(maxFTime,end);
        lp.setMaxFTime(maxFTime);
        if(isOver) {
//            updateAdaptiveSplitAfterRun();
            return;
        }

        double startB = System.nanoTime();
        /* 2. Backward half: run the backward queue to completion, then try start-depot closure. */
        runBackwardLabelingDP();
        closeBackwardLabelsToStartDepot();
        if (hasEnoughCompletedColumns()) {
            buildReturnedRoutes(selectedInThisPricingCall);
            isOver=true;
        }
        end = (System.nanoTime()-startB)*1e-9;
        if(Constants.CONSOLE) System.out.println("Backward Time:"+end);
        this.maxBTime = Math.max(maxBTime,end);
        lp.setMaxBTime(maxBTime);
        if(isOver) {
//            updateAdaptiveSplitAfterRun();
            return;
        }

        double startJ = System.nanoTime();
        /* 3. Join forward and backward labels. */
        joinForwardAndBackwardLabels();
        buildReturnedRoutes(selectedInThisPricingCall);
        end = (System.nanoTime()-startJ)*1e-9;
        if(Constants.CONSOLE) System.out.println("Join Time:"+end);
        this.maxJTime = Math.max(maxJTime,end);
        lp.setMaxJTime(maxJTime);
        updateAdaptiveSplitAfterRun();
    }

    private void chooseAdaptiveSplit() {
        if (effectiveMaxCustomers <= 0) {
            forwardMaxCustomers = 0;
            backwardMaxCustomers = 0;
            return;
        }

        if (effectiveMaxCustomers == 1) {
            forwardMaxCustomers = 1;
            backwardMaxCustomers = 0;
            return;
        }

        int defaultForwardSplit = effectiveMaxCustomers / 2;

        int storedForwardSplit =
                adaptiveForwardSplitByLength.getOrDefault(
                        effectiveMaxCustomers,
                        defaultForwardSplit
                );

        forwardMaxCustomers = clamp(
                storedForwardSplit,
                1,
                effectiveMaxCustomers - 1
        );

        backwardMaxCustomers = effectiveMaxCustomers - forwardMaxCustomers;
    }

    private void updateAdaptiveSplitAfterRun() {
        if (effectiveMaxCustomers <= 1) {
            return;
        }

        /*
         * Use generated labels, not accepted labels, because generated labels
         * better reflect the size of the search tree induced by the split.
         */
        long f = Math.max(1L, generatedForwardLabels);
        long b = Math.max(1L, generatedBackwardLabels);

        int nextForwardSplit = forwardMaxCustomers;

        if (f > SPLIT_IMBALANCE_TRIGGER * b) {
            /*
             * Forward side is too heavy.
             * Move the split toward the start depot:
             * fewer forward visits, more backward visits.
             */
            nextForwardSplit -= SPLIT_ADJUSTMENT_STEP;
        } else if (b > SPLIT_IMBALANCE_TRIGGER * f) {
            /*
             * Backward side is too heavy.
             * Move the split toward the end depot:
             * more forward visits, fewer backward visits.
             */
            nextForwardSplit += SPLIT_ADJUSTMENT_STEP;
        }

        nextForwardSplit = clamp(
                nextForwardSplit,
                1,
                effectiveMaxCustomers - 1
        );

        adaptiveForwardSplitByLength.put(
                effectiveMaxCustomers,
                nextForwardSplit
        );
    }

    private int clamp(int value, int lower, int upper) {
        return Math.max(lower, Math.min(upper, value));
    }

    public double getMaxFTime() {
        return maxFTime;
    }

    public double getMaxJTime() {
        return maxJTime;
    }

    public double getMaxBTime() {
        return maxBTime;
    }

    public ArrayList<Route> getRoutes() {
        return routes;
    }

    private void initializeCustomerMask() {
        customerMask = 0L;
        for (int i = 1; i < N; i++) {
            customerMask |= bit(i);
        }
    }

    private boolean hasEnoughCompletedColumns() {
        // || completedTopLabels.size() >= maxRoutesToReturn
        return earlyTermination;
    }

    private void initializeTabu() {
        tabu = new boolean[N + 1];

        if (!heuristic) {
            return;
        }

        boolean foundUsefulDual = false;

        for (int i = 1; i < N; i++) {
            double visitDual = dualVisit[i];

            if (visitDual <= EPS) {
                double serviceDualContribution = dualDifferences[i];

                if (serviceDualContribution <= EPS) {
                    tabu[i] = true;
                } else {
                    foundUsefulDual = true;
                }
            } else {
                foundUsefulDual = true;
            }
        }

        if (!foundUsefulDual) {
            double depotDual = dualVisit[0];

            if (Math.abs(depotDual) <= EPS) {
                for (int i = 1; i < N; i++) {
                    tabu[i] = true;
                }
            }
        }
    }

    private void buildSuccessors() {
        successors = new int[N + 1][];

        for (int i = 0; i < N; i++) {
            ArrayList<Integer> list = new ArrayList<>();

            for (int j = 1; j <= N; j++) {
                if (j == i) {
                    continue;
                }
                if (i == 0 && j == depotEnd) {
                    continue;
                }
                if (j < tabu.length && tabu[j]) {
                    continue;
                }
                if (!lp.isFeasibleArc(i, j, false)) {
                    continue;
                }
                if (violatesZonePriority(i, j)) {
                    continue;
                }
                list.add(j);
            }

            successors[i] = toIntArray(list);
        }
    }

    private void buildPredecessors() {
        predecessors = new int[N + 1][];

        for (int j = 1; j <= N; j++) {
            ArrayList<Integer> list = new ArrayList<>();

            for (int i = 0; i < N; i++) {
                if (i == j) {
                    continue;
                }
                if (i == 0 && j == depotEnd) {
                    continue;
                }
                if (i > 0 && i < tabu.length && tabu[i]) {
                    continue;
                }
                if (!lp.isFeasibleArc(i, j, false)) {
                    continue;
                }
                if (violatesZonePriority(i, j)) {
                    continue;
                }
                list.add(i);
            }

            predecessors[j] = toIntArray(list);
        }
    }

    private int[] toIntArray(ArrayList<Integer> list) {
        int[] arr = new int[list.size()];
        for (int k = 0; k < list.size(); k++) {
            arr[k] = list.get(k);
        }
        return arr;
    }

    @SuppressWarnings("unchecked")
    private void initializeLabelBuckets() {
        forwardPermanentLabels = new SkylineBucket[N + 1][forwardMaxCustomers + 1];
        backwardPermanentLabels = new SkylineBucket[N + 1][backwardMaxCustomers + 1];

        for (int i = 0; i <= N; i++) {
            for (int q = 0; q <= forwardMaxCustomers; q++) {
                forwardPermanentLabels[i][q] = new SkylineBucket(false);
            }

            for (int q = 0; q <= backwardMaxCustomers; q++) {
                backwardPermanentLabels[i][q] = new SkylineBucket(true);
            }
        }

        forwardOpenQueue = new PriorityQueue<>((a, b) -> Double.compare(b.c, a.c));
        backwardOpenQueue = new PriorityQueue<>((a, b) -> Double.compare(b.c, a.c));
        completedTopLabels = new PriorityQueue<>(Comparator.comparingDouble(a -> a.c));
    }



    private void runForwardLabelingDP() {
        Label root = new Label(Direction.FORWARD);
        root.i = 0;
        root.q = 0;
        root.c = dualVisit[0];
        root.t = 0.0;
        root.mask = bit(0);
        root.p = null;
        root.z = false;

        forwardOpenQueue.add(root);

        while (!forwardOpenQueue.isEmpty()) {
            Label parent = forwardOpenQueue.poll();

            if(earlyTermination)
                break;

            if (parent.dominated) {
                continue;
            }
            if (parent.i == depotEnd) {
                continue;
            }
            if (!acceptPermanentLabel(parent, false)) {
                continue;
            }
            if (parent.q >= forwardMaxCustomers) {
                continue;
            }

            int[] succ = successors[parent.i];
            for (int idx = 0; idx < succ.length; idx++) {
                int j = succ[idx];

//                if (j == depotEnd) {
//                    continue;
//                }
                if (!isNonElementary && contains(parent.mask, j)) {
                    continue;
                }

                extendForwardWithoutService(parent, j);

                double serviceGain = serviceDuration * dualDifferences[j];
                if (serviceGain > EPS) {
                    /*
                     * The service label has the same node, same q, same mask,
                     * larger t, and cost increased by serviceGain.
                     *
                     * If serviceGain <= 0, the the no-service label dominates it.
                     */

                    extendForwardWithService(parent, j);
                }
            }
        }
    }

    private void closeForwardLabelsToEndDepot() {
        for (int i = 0; i < N; i++) {
            if (i == 0 || i == depotEnd) {
                continue;
            }
            if (!lp.isFeasibleArc(i, depotEnd, false)) {
                continue;
            }
            if (violatesZonePriority(i, depotEnd)) {
                continue;
            }

            for (int q = 0; q <= forwardMaxCustomers; q++) {
                ArrayList<Label> bucket = forwardPermanentLabels[i][q].snapshot();
                for (int k = 0; k < bucket.size(); k++) {
                    extendForwardWithoutService(bucket.get(k), depotEnd);
                }
            }
        }
    }

    private void extendForwardWithoutService(Label parent, int j) {
        generatedForwardLabels++;

        int i = parent.i;
        double arrival = parent.t + travelTime[i][j] + positionTime[j];

        Label label = new Label(Direction.FORWARD);
        label.p = parent;
        label.i = j;
        label.t = arrival;
        label.c = parent.c + dualVisit[j] + dualArrival[j] * arrival;
        label.z = false;

        if (j == depotEnd) {
            label.q = parent.q;
            label.mask = parent.mask;
            handleCompletedLabel(label);
        } else {
            label.q = parent.q + 1;
            label.mask = parent.mask | bit(j);
            pushIfNotDominated(label, false);
        }
    }

    private void extendForwardWithService(Label parent, int j) {
        if(j==depotEnd)
            return;
        generatedForwardLabels++;
        int i = parent.i;
        double arrival = parent.t + travelTime[i][j] + positionTime[j];
        double departure = arrival + serviceDuration;

        Label label = new Label(Direction.FORWARD);
        label.p = parent;
        label.i = j;
        label.t = departure;
        label.c = parent.c
                + dualVisit[j]
                + dualArrival[j] * arrival
                + serviceDuration * dualDifferences[j];
        label.z = true;
        label.q = parent.q + 1;
        label.mask = parent.mask | bit(j);

        pushIfNotDominated(label, false);
    }

    /* ------------------------------------------------------------------
     * Backward labeling.
     * ------------------------------------------------------------------ */

    private void runBackwardLabelingDP() {
        Label root = new Label(Direction.BACKWARD);
        root.i = depotEnd;
        root.q = 0;
        root.c = 0;
        root.y = 0;
        root.mask = bit(depotEnd);
        root.p = null;
        root.z = false;

        backwardOpenQueue.add(root);

        while (!backwardOpenQueue.isEmpty()) {
            Label parent = backwardOpenQueue.poll();
            if(earlyTermination)
                break;
            if (parent.dominated) {
                continue;
            }
//            if (parent.i == 0) {
//                continue;
//            }
            if (!acceptPermanentLabel(parent, true)) {
                continue;
            }
            if (parent.q >= backwardMaxCustomers) {
                continue;
            }

            int[] pred = predecessors[parent.i];
            for (int idx = 0; idx < pred.length; idx++) {
                int i = pred[idx];

                if (i == 0 || i == depotEnd) {
                    continue;
                }
                if (!isNonElementary && contains(parent.mask, i)) {
                    continue;
                }

                double serviceGain =
                        serviceDuration * (dualDifferences[i] + parent.y);

                if (serviceGain >= -EPS) {
                    extendBackwardWithService(parent, i);
                } else {
                    extendBackwardWithoutService(parent, i);
                }
//                extendBackwardWithoutService(parent, i);
//                extendBackwardWithService(parent, i);
            }
        }
    }

    private void closeBackwardLabelsToStartDepot() {
        for (int j = 1; j <= N; j++) {
            if (j == depotEnd) {
                continue;
            }
            if (!lp.isFeasibleArc(0, j, false)) {
                continue;
            }
            if (violatesZonePriority(0, j)) {
                continue;
            }

            for (int q = 0; q <= backwardMaxCustomers; q++) {
                ArrayList<Label> bucket = backwardPermanentLabels[j][q].snapshot();
                for (int k = 0; k < bucket.size(); k++) {
                    extendBackwardWithoutService(bucket.get(k), 0);
                }
            }
        }
    }

    private void extendBackwardWithoutService(Label parent, int i) {
        generatedBackwardLabels++;
        int j = parent.i;
        Label label = new Label(Direction.BACKWARD);
        label.p = parent;
        label.i = i;
        label.c = parent.c
                + dualVisit[i]
                + dualArrival[i] * positionTime[i]
                + parent.y * (positionTime[i] + travelTime[i][j]);
        label.y = parent.y + dualArrival[i];
        label.z = false;

        if (i == 0) {
            label.q = parent.q;
            label.mask = parent.mask | bit(0);
            handleCompletedLabel(label);
        } else {
            label.q = parent.q + 1;
            label.mask = parent.mask | bit(i);
            pushIfNotDominated(label, true);
        }
    }

    private void extendBackwardWithService(Label parent, int i) {
        if(i==0)
            return;
        generatedBackwardLabels++;
        int j = parent.i;

        Label label = new Label(Direction.BACKWARD);
        label.p = parent;
        label.i = i;
        label.c = parent.c
                + dualVisit[i]
                + dualArrival[i] * positionTime[i]
                + parent.y * (positionTime[i] + travelTime[i][j])
                + serviceDuration * (dualDifferences[i] + parent.y);
        label.y = parent.y + dualArrival[i];
        label.z = true;
        label.q = parent.q + 1;
        label.mask = parent.mask | bit(i);

        pushIfNotDominated(label, true);
    }



    private void pushIfNotDominated(Label label, boolean backward) {
        int limit = backward ? backwardMaxCustomers : forwardMaxCustomers;

        if (label.q > limit) {
            return;
        }

        if (label.c <= -INF / 2) {
            return;
        }

        if (isDominatedBySkyline(label, backward)) {
            label.dominated = true;
            rejectedByDominance++;
            return;
        }

        if (backward) {
            backwardOpenQueue.add(label);
        } else {
            forwardOpenQueue.add(label);
        }
    }


    private boolean acceptPermanentLabel(Label label, boolean backward) {
        int limit = backward ? backwardMaxCustomers : forwardMaxCustomers;

        if (label.q > limit) {
            label.dominated = true;
            return false;
        }

        if (!filterAndInsertIntoSkyline(label, backward)) {
            return false;
        }

        if (backward) {
            acceptedBackwardLabels++;
        } else {
            acceptedForwardLabels++;
        }

        return true;
    }

    private boolean isDominatedBySkyline(Label label, boolean backward) {
        int node = label.i;

        if ((!backward && node == depotEnd) || (backward && node == 0)) {
            return false;
        }

        int limit = backward ? backwardMaxCustomers : forwardMaxCustomers;

        SkylineBucket[][] buckets =
                backward ? backwardPermanentLabels : forwardPermanentLabels;

        int qEnd = Constants.IMPOSE_MAXIMUM_TOWER_ROUTE_LENGTH
                ? label.q
                : limit;

        for (int q = 0; q <= qEnd; q++) {
            dominanceComparisons++;

            if (buckets[node][q].dominates(label)) {
                return true;
            }
        }

        return false;
    }

    private boolean filterAndInsertIntoSkyline(Label label, boolean backward) {
        int node = label.i;

        if ((!backward && node == depotEnd) || (backward && node == 0)) {
            return true;
        }

        int limit = backward ? backwardMaxCustomers : forwardMaxCustomers;

        SkylineBucket[][] buckets =
                backward ? backwardPermanentLabels : forwardPermanentLabels;

        int qDominatorsEnd;
        int qDominatedStart;

        if (Constants.IMPOSE_MAXIMUM_TOWER_ROUTE_LENGTH) {
            /*
             * Existing labels with q <= label.q may dominate the new label.
             * The new label may dominate existing labels with q >= label.q.
             */
            qDominatorsEnd = label.q;
            qDominatedStart = label.q;
        } else {
            /*
             * q is ignored in dominance.
             */
            qDominatorsEnd = limit;
            qDominatedStart = 0;
        }

        /*
         * Step 1: check whether the new label is dominated.
         */
        for (int q = 0; q <= qDominatorsEnd; q++) {
            dominanceComparisons++;

            if (buckets[node][q].dominates(label)) {
                label.dominated = true;
                rejectedByDominance++;
                return false;
            }
        }

        /*
         * Step 2: remove labels dominated by the new label.
         */
        for (int q = qDominatedStart; q <= limit; q++) {
            int removed = buckets[node][q].removeDominatedBy(label);

            if (removed > 0) {
                labelsRemovedByDominance += removed;
            }
        }

        /*
         * Step 3: insert the new label into its own skyline bucket.
         */
        SkylineBucket ownBucket = buckets[node][label.q];
        ownBucket.insertKnownNondominated(label);

        if (ownBucket.size() > maxBucketSize) {
            maxBucketSize = ownBucket.size();
        }

        return true;
    }

    private class SkylineBucket {
        /*
         * The key is:
         *   forward:  t
         *   backward: -y
         *
         * In both cases, smaller key is better and larger c is better.
         */
        private final TreeMap<Double, Label> skyline;
        private final boolean backward;

        SkylineBucket(boolean backward) {
            this.backward = backward;
            this.skyline = new TreeMap<>();
        }

        int size() {
            return skyline.size();
        }

        boolean isEmpty() {
            return skyline.isEmpty();
        }

        ArrayList<Label> snapshot() {
            return new ArrayList<>(skyline.values());
        }

        private double resource(Label label) {
            if (backward) {
                return -label.y;
            }

            return label.t;
        }

        boolean dominates(Label label) {
            if (skyline.isEmpty()) {
                return false;
            }

            double r = resource(label);

            /*
             * Need an existing label with:
             *   existing.resource <= label.resource
             *   existing.c >= label.c
             *
             * Because the skyline is maintained with increasing resource and
             * increasing c, the best candidate is the largest resource <= r.
             */
            Map.Entry<Double, Label> candidate =
                    skyline.floorEntry(r + EPS);

            if (candidate == null) {
                return false;
            }

            Label existing = candidate.getValue();

            return existing.c >= label.c - EPS;
        }

        int removeDominatedBy(Label label) {
            if (skyline.isEmpty()) {
                return 0;
            }

            double r = resource(label);

            /*
             * Remove existing labels with:
             *   existing.resource >= label.resource
             *   existing.c <= label.c
             *
             * Since c is increasing with resource along the skyline,
             * once c becomes greater than label.c, we can stop.
             */
            ArrayList<Double> keysToRemove = new ArrayList<>();

            Map.Entry<Double, Label> entry =
                    skyline.ceilingEntry(r - EPS);

//            int depth = 0;
            while (entry != null) {
                Label existing = entry.getValue();

//                depth++;
                if (existing.c > label.c + EPS) {
                    break;
                }

                keysToRemove.add(entry.getKey());

                entry = skyline.higherEntry(entry.getKey());
            }

            for (Double key : keysToRemove) {
                Label removed = skyline.remove(key);

                if (removed != null) {
                    removed.dominated = true;
                }
            }

            return keysToRemove.size();
        }

        void insertKnownNondominated(Label label) {
            double r = resource(label);

            /*
             * The caller already checked that the label is not dominated
             * and already removed labels dominated by it.
             */
            skyline.put(r, label);
        }
    }

    private void handleCompletedLabel(Label label) {
        if (label.c < Constants.TOWER_REDUCED_ACCEPTANCE_THRESHOLD) {
            return;
        }

        if (completedTopLabels.size() < maxCompletedLabelsToKeep) {
            completedTopLabels.add(label);
            return;
        }

        overTheTop++;
        Label worstKept = completedTopLabels.peek();
        if (worstKept != null && label.c > worstKept.c + EPS) {
            completedTopLabels.poll();
            completedTopLabels.add(label);
        }

        if (overTheTop >= thresholdForEarlyTermination) {
            earlyTermination = true;
        }
    }

    /* ------------------------------------------------------------------
     * Joining.
     * ------------------------------------------------------------------ */

    private void joinForwardAndBackwardLabels() throws IloException {
        BucketView[][] backwardViews = buildBackwardBucketViews();

        for (int i = 1; i < N && !earlyTermination; i++) {
            for (int qF = 0; qF <= forwardMaxCustomers && !earlyTermination; qF++) {
                ArrayList<Label> fBucket = forwardPermanentLabels[i][qF].snapshot();
                if (fBucket.isEmpty()) {
                    continue;
                }
                fBucket.sort((a, b) -> Double.compare(b.c, a.c));

                for (int j = 1; j < N && !earlyTermination; j++) {
                    if (j == i) {
                        continue;
                    }
                    if (!lp.isFeasibleArc(i, j, false)) {
                        continue;
                    }
                    if (violatesZonePriority(i, j)) {
                        continue;
                    }

                    double arcTravel = travelTime[i][j];

                    for (int qB = 0; qB <= backwardMaxCustomers && !earlyTermination; qB++) {
                        if (qF + qB > effectiveMaxCustomers) {
                            break;
                        }

                        BucketView bView = backwardViews[j][qB];
                        if (bView == null || bView.labels.isEmpty()) {
                            continue;
                        }

                        for (int fIdx = 0; fIdx < fBucket.size() && !earlyTermination; fIdx++) {
                            Label fL = fBucket.get(fIdx);
                            if (fL.dominated) {
                                continue;
                            }

                            double shift = arcTravel + fL.t;
                            double bucketUpperBound = fL.c + bView.maxC + bView.maxY * Math.max(0.0, shift);
                            if (bucketUpperBound < EPS) {
                                continue;
                            }

                            for (int bIdx = 0; bIdx < bView.labels.size(); bIdx++) {
                                Label bL = bView.labels.get(bIdx);
                                if (bL.dominated) {
                                    continue;
                                }

                                /* Inner-loop early break: remaining b labels have c <= current c. */
                                double remainingUpperBound = fL.c
                                        + bL.c
                                        + bView.suffixMaxY[bIdx] * Math.max(0.0, shift);
                                if (remainingUpperBound < EPS) {
                                    break;
                                }

                                if (!canJoin(fL, bL, i, j)) {
                                    continue;
                                }

                                double joinCost = fL.c + bL.c + bL.y * (travelTime[i][j] + fL.t);



                                if (joinCost < EPS) {
                                    continue;
                                }

                                Label joined = new Label(Direction.JOINED);
                                joined.i = depotEnd;
                                joined.q = fL.q + bL.q;
                                joined.c = joinCost;
                                joined.t = fL.t + travelTime[i][j] + suffixDurationFromBackwardLabel(bL);
                                joined.mask = fL.mask | bL.mask;
                                joined.fPart = fL;
                                joined.bPart = bL;
                                joined.z = false;

                                handleCompletedLabel(joined);
                            }
                        }
                    }
                }
            }
        }
    }

    private boolean isDepotNode(int node) {
        return node == 0 || node == depotEnd;
    }

    private void clearDepotServiceFlags(ArrayList<Integer> schedule,
                                        ArrayList<Boolean> waitBooleans) {
        for (int k = 0; k < schedule.size(); k++) {
            if (isDepotNode(schedule.get(k))) {
                waitBooleans.set(k, false);
            }
        }
    }



    private BucketView[][] buildBackwardBucketViews() {
        BucketView[][] views = new BucketView[N + 1][backwardMaxCustomers + 1];

        for (int j = 1; j <= N; j++) {
            for (int q = 0; q <= backwardMaxCustomers; q++) {
                ArrayList<Label> bucket = backwardPermanentLabels[j][q].snapshot();

                if (bucket.isEmpty()) {
                    continue;
                }

                bucket.sort((a, b) -> Double.compare(b.c, a.c));
                views[j][q] = new BucketView(bucket);
            }
        }

        return views;
    }
    private boolean canJoin(Label fL, Label bL, int i, int j) {
        if (fL == null || bL == null || fL.dominated || bL.dominated) {
            return false;
        }
        if (fL.q + bL.q > effectiveMaxCustomers) {
            return false;
        }
        if (!lp.isFeasibleArc(i, j, false)) {
            return false;
        }
        if (violatesZonePriority(i, j)) {
            return false;
        }
        if (!isNonElementary) {
            long overlap = (fL.mask & bL.mask) & customerMask;
            if (overlap != 0L) {
                return false;
            }
        }
        return true;
    }

    private double suffixDurationFromBackwardLabel(Label bL) {
        return 0;
    }

    /* ------------------------------------------------------------------
     * Route construction.
     * ------------------------------------------------------------------ */

    private void buildReturnedRoutes(HashSet<String> selectedInThisPricingCall) throws IloException {
        routes.clear();

        ArrayList<Label> candidates = new ArrayList<>(completedTopLabels);
        candidates.sort((a, b) -> Double.compare(b.c, a.c));




        int idx = 0;
        while (routes.size() < maxRoutesToReturn && idx < candidates.size()) {
            Label label = candidates.get(idx++);

            if (label == null || label.dominated) {
                continue;
            }
            if (label.c < Constants.TOWER_REDUCED_ACCEPTANCE_THRESHOLD) {
                break;
            }

            RouteBuild routeBuild = reconstructRoute(label);
            String signature = routeSignature(routeBuild);

            if (!selectedInThisPricingCall.add(signature)) {
                continue;
            }

            if (lp.r2i(routeBuild.schedule, routeBuild.waitBooleans) != null) {
                continue;
            }

            Route route = new Route();
            route.create(routeBuild.schedule, routeBuild.waitBooleans);
            route.setPseudo_cost(label.c);
            routes.add(route);
        }

        completedTopLabels.clear();
        forwardOpenQueue.clear();
        backwardOpenQueue.clear();
    }

    private String routeSignature(RouteBuild routeBuild) {
        StringBuilder sb = new StringBuilder(routeBuild.schedule.size() * 8);

        for (int k = 0; k < routeBuild.schedule.size(); k++) {
            sb.append(routeBuild.schedule.get(k))
                    .append(':')
                    .append(routeBuild.waitBooleans.get(k) ? '1' : '0')
                    .append('|');
        }

        return sb.toString();
    }

    private RouteBuild reconstructRoute(Label label) {
        if (label.direction == Direction.JOINED) {
            return reconstructJoinedRoute(label.fPart, label.bPart);
        }
        if (label.direction == Direction.BACKWARD) {
            return reconstructBackwardRoute(label);
        }
        return reconstructForwardRoute(label);
    }

    private RouteBuild reconstructForwardRoute(Label label) {
        int length = 0;
        Label tmp = label;

        while (tmp != null) {
            length++;
            tmp = tmp.p;
        }

        int[] scheduleArray = new int[length];
        boolean[] serviceArray = new boolean[length];

        tmp = label;
        int pos = length - 1;

        while (tmp != null) {
            scheduleArray[pos] = tmp.i;
            serviceArray[pos] = tmp.z;
            tmp = tmp.p;
            pos--;
        }

        ArrayList<Integer> schedule = new ArrayList<>(length);
        ArrayList<Boolean> waitBooleans = new ArrayList<>(length);

        for (int k = 0; k < length; k++) {
            schedule.add(scheduleArray[k]);
            waitBooleans.add(serviceArray[k]);
        }

        clearDepotServiceFlags(schedule, waitBooleans);

        return new RouteBuild(schedule, waitBooleans);
    }

    private RouteBuild reconstructBackwardRoute(Label label) {
        ArrayList<Integer> schedule = new ArrayList<>();
        ArrayList<Boolean> waitBooleans = new ArrayList<>();

        Label tmp = label;

        while (tmp != null) {
            schedule.add(tmp.i);
            waitBooleans.add(tmp.z);
            tmp = tmp.p;
        }


        clearDepotServiceFlags(schedule, waitBooleans);

        return new RouteBuild(schedule, waitBooleans);
    }

    private RouteBuild reconstructJoinedRoute(Label fL, Label bL) {
        RouteBuild forward = reconstructForwardRoute(fL);
        RouteBuild backward = reconstructBackwardRoute(bL);

        ArrayList<Integer> schedule = new ArrayList<>(forward.schedule);
        ArrayList<Boolean> waitBooleans = new ArrayList<>(forward.waitBooleans);

        schedule.addAll(backward.schedule);
        waitBooleans.addAll(backward.waitBooleans);

        clearDepotServiceFlags(schedule, waitBooleans);

        return new RouteBuild(schedule, waitBooleans);
    }

    private RouteBuild toRouteBuild(int[] scheduleArray, boolean[] serviceArray) {
        ArrayList<Integer> schedule = new ArrayList<>(scheduleArray.length);
        ArrayList<Boolean> waitBooleans = new ArrayList<>(serviceArray.length);

        for (int i = 0; i < scheduleArray.length; i++) {
            schedule.add(scheduleArray[i]);
            waitBooleans.add(serviceArray[i]);
        }
        return new RouteBuild(schedule, waitBooleans);
    }

    private boolean violatesZonePriority(int from, int to) {
        if (UtilityAlgoIsNotPriorityMode()) {
            return false;
        }

        int[] priority = data.getZonePriority();
        if (priority == null) {
            return false;
        }
        if (from < 0 || to < 0 || from >= priority.length || to >= priority.length) {
            return false;
        }
        return priority[to] > priority[from];
    }


    private boolean UtilityAlgoIsNotPriorityMode() {
        return !Constants.COST_OF_PRIORITY_TOWER;
    }

    private long bit(int node) {
        if (node < 0 || node >= 63) {
            throw new IllegalArgumentException(
                    "Node index " + node + " cannot be represented in a long mask."
            );
        }
        return 1L << node;
    }

    private boolean contains(long mask, int node) {
        return (mask & bit(node)) != 0L;
    }

    private enum Direction {
        FORWARD,
        BACKWARD,
        JOINED
    }

    private static class RouteBuild {
        final ArrayList<Integer> schedule;
        final ArrayList<Boolean> waitBooleans;

        RouteBuild(ArrayList<Integer> schedule, ArrayList<Boolean> waitBooleans) {
            this.schedule = schedule;
            this.waitBooleans = waitBooleans;
        }
    }

    private class BucketView {
        final ArrayList<Label> labels;
        final double[] suffixMaxY;
        final double maxC;
        final double maxY;

        BucketView(ArrayList<Label> labels) {
            this.labels = labels;
            this.suffixMaxY = new double[labels.size()];

            double localMaxC = -INF;
            double localMaxY = -INF;
            double runningMaxY = -INF;

            for (int i = labels.size() - 1; i >= 0; i--) {
                Label label = labels.get(i);
                runningMaxY = Math.max(runningMaxY, label.y);
                suffixMaxY[i] = runningMaxY;
                localMaxC = Math.max(localMaxC, label.c);
                localMaxY = Math.max(localMaxY, label.y);
            }

            this.maxC = localMaxC;
            this.maxY = localMaxY;
        }
    }

    private class Label {
        final Direction direction;

        int i;
        int q;

        double c;
        double t;
        double y;

        long mask;

        Label p;
        Label fPart;
        Label bPart;

        boolean dominated;
        boolean markedForRemoval;
        /** True if the head node of this label is serviced. */
        boolean z;

        Label(Direction direction) {
            this.direction = direction;
        }

        boolean dominates(Label other) {
            if (this.direction != other.direction) {
                return false;
            }

            if (Constants.IMPOSE_MAXIMUM_TOWER_ROUTE_LENGTH) {
                if (this.q > other.q) {
                    return false;
                }
            }

            if (this.c < other.c - EPS) {
                return false;
            }

            if (this.direction == Direction.BACKWARD) {
                /* Required extra backward dominance condition. */
                if (this.y < other.y - EPS) {
                    return false;
                }
            } else {
                /* Forward t is elapsed departure time; smaller is better. */
                if (this.t > other.t + EPS) {
                    return false;
                }
            }

            if (!heuristic && !isNonElementary) {
                if ((this.mask & ~other.mask) != 0L) {
                    return false;
                }
            }

            return true;
        }
    }



    private void resetDPStats() {
        generatedForwardLabels = 0L;
        generatedBackwardLabels = 0L;

        acceptedForwardLabels = 0L;
        acceptedBackwardLabels = 0L;

        rejectedByDominance = 0L;
        dominanceComparisons = 0L;
        labelsRemovedByDominance = 0L;

        maxBucketSize = 0;

        dominanceMarkedForRemoval.clear();
    }
}
