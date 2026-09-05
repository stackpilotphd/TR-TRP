package problem;

import lib.*;
import problem.analyses.StaticCounters;
import problem.graph.*;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class InstanceIO {
    public void readInstance(String filePath, int towerNumber, int crewNumber) {
        if(Constants.ROBUST && !Constants.DETERMINE_MINIMUM_NUMBER_CREWS){
            if(towerNumber <3 && towerNumber != 0)
                throw new IllegalArgumentException("tower number must be set to 3 for robust experiments");
        }
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            // Read first line (general info)
            line = br.readLine();
            String[] parts = line.split("\s++");
            int pcounter = 0;
            int crews = Integer.parseInt(parts[pcounter++]);
            int towers = Integer.parseInt(parts[pcounter++]);
            if(Constants.RUN_COMPUTATIONAL_STUDY) towers = towerNumber;
            if(Constants.DETERMINE_MINIMUM_NUMBER_CREWS) crews = crewNumber;
            int zones = Integer.parseInt(parts[pcounter++]);
            int tasks = Integer.parseInt(parts[pcounter++]);
            // Read Vertex data
            double[] repairTimes = new double[tasks + 2];
            double[] positionTimes = new double[zones + 2];
            double[] weights = new double[zones + 2];
            int[] priorities = new int[zones + 2];
            int[] numberPeople = new int[zones + 2];
            ArrayList<Integer>[] task2zones = new ArrayList[tasks+2];
            ArrayList<Integer>[] zone2tasks = new ArrayList[zones+2];
            for (int i = 1; i < zones + 1; i++) {
                pcounter = 0;
                line = br.readLine();
                parts = line.split("\s++");
                int id = Integer.parseInt(parts[pcounter++]);
                double posTime = Double.parseDouble(parts[pcounter++]);
                double w = Double.parseDouble(parts[pcounter++]);
                positionTimes[id] = posTime;
                weights[id] = w;
                priorities[id] = Integer.parseInt(parts[pcounter++]);
                numberPeople[id] = Integer.parseInt(parts[pcounter++]);
            }

            for (int i = 0; i < tasks; i++) {
                pcounter = 0;
                line = br.readLine();
                parts = line.split("\s++");
                int id = Integer.parseInt(parts[pcounter++]);
                double repTime = Double.parseDouble(parts[pcounter++]);
                repairTimes[id] = repTime;
                String[] s = parts[pcounter].split(",");
                ArrayList<Integer> hones = new ArrayList<>(s.length);
                for(String y : s)
                    hones.add(Integer.parseInt(y));
                task2zones[id] = hones;
                HashSet<Integer> hashSet = new HashSet<>(hones);
                if(hashSet.size() != hones.size())
                    throw new IllegalArgumentException("duplicate zone in : " + id + ":"+hones.toString());
                for(Integer j : hones){
                    if(zone2tasks[j] == null)
                        zone2tasks[j] = new ArrayList<>();
                    zone2tasks[j].add(id);
                }
            }
            for (int i = 1; i < zones+1; i++) {
                if(zone2tasks[i].isEmpty())
                    throw new IllegalArgumentException("zone "+i+" has no tasks.");
                Collections.sort(zone2tasks[i]);
            }

            // Read Distances
            double[][] distanceMatrix = new double[zones + 2][zones + 2];
            for (int i = 0; i < zones + 1; i++) {
                pcounter = 0;
                line = br.readLine();
                parts = line.split("\s++");
                for (int j = 0; j < zones + 1; j++) {
                    double km = Double.parseDouble(parts[pcounter++]);
                    distanceMatrix[i][j] = km;
                }
            }
            //------------Graph processing--------------
            double[][] travelTower = new double[zones + 2][zones + 2];
            for (int i = 0; i < zones + 1; i++) {
                for (int j = 0; j < zones + 1; j++) {
                    travelTower[i][j] = distanceMatrix[i][j];
                }
                int j = zones + 1;
                travelTower[i][j] = distanceMatrix[i][0];
            }

            double largeValue = 999999;
            for (int i = 0; i < travelTower.length-1; i++) {
                travelTower[i][i] = largeValue;
                travelTower[zones+1][i] = largeValue;
            }

            // Read Distances
            distanceMatrix = new double[tasks + 2][tasks + 2];
            for (int i = 0; i < tasks+1; i++) {
                pcounter = 0;
                line = br.readLine();
                parts = line.split("\s++");
                for (int j = 0; j < tasks+1; j++) {
                    double km = Double.parseDouble(parts[pcounter++]);
                    distanceMatrix[i][j] = km;
                }
            }
            //------------Graph processing--------------
            double[][] travelCrew = new double[tasks + 2][tasks + 2];
            for (int i = 0; i < travelCrew.length; i++) {
                for (int j = 0; j < travelCrew.length; j++) {
                    travelCrew[i][j] = distanceMatrix[i][j];
                }
                int j = tasks + 1;
                travelCrew[i][j] = distanceMatrix[i][0];
            }

            for (int i = 0; i < travelCrew.length-1; i++) {
                travelCrew[i][i] = largeValue;
                travelCrew[tasks+1][i] = largeValue;
            }

            Data data = Data.getInstance();
            data.setCrewNumber(crews);

            data.setTowerNumber(towers);
            data.setTaskNumber(tasks+1);
            data.setNodeNumber(zones + 1);
            data.setRepairTimeMatrix(repairTimes);
            data.setPositionTimeMatrix(positionTimes);
            data.setWeights(weights);
            data.setCrewTravelTimeMatrix(travelCrew);
            data.setTowerTravelTimeMatrix(travelTower);
            data.setArrays(task2zones,zone2tasks);
            StaticSharedValues.crewWorkTime = Constants.CREW_WORK_LENGTH;
            data.setTimeLimit(Constants.CREW_WORK_LENGTH);
            data.setPopulation(numberPeople);

            double cM = getCrewM(zones, travelCrew, repairTimes);
            double serviceM = getServiceM(zones, travelCrew, repairTimes, travelTower, positionTimes);
            if(serviceM < 1e-6)
                throw new IllegalArgumentException("empty value");
            double tM = getTowerM(zones, travelCrew, repairTimes, travelTower, positionTimes, serviceM);
            double tTravelM = getTralveTowerM(zones, travelCrew, repairTimes, travelTower, positionTimes, serviceM);

            System.out.println("Crew#:"+crews +";Tower#:"+towerNumber);
            System.out.println("Longest Service Time:" + serviceM);
            double avgServiceTime = 0;
            double cum = 0;
            for (int i = 1; i < data.getTasks(); i++) cum += data.getServiceTimeMatrix()[i];
            avgServiceTime = (cum / (1.*(data.getTasks()-1)));
            System.out.println("Average Service Time is:"+avgServiceTime);
            data.setBiGMs(cM, tM, serviceM, tM, tTravelM);
            data.setEarlisetServiceStartTimes(
                    getEarliestServiceStartTimes(zones,travelTower,positionTimes));

            SpecificFunctions.allTaskSet = new Cet(data.getTasks()+1);
            for (int i = 1; i < data.getTasks()+1; i++)
                SpecificFunctions.allTaskSet.add(i);

            SpecificFunctions.allNodeSet = new Cet(data.getNodeNumber()+1);
            for (int i = 1; i < data.getNodeNumber()+1; i++)
                SpecificFunctions.allNodeSet.add(i);

            long mI = 0x00L;
            for (int i = 1; i < data.getNodeNumber(); i++)
                mI |= 1L << i;
            data.setAllMask(mI);

            if(Constants.ROBUST) {
                double alpha = Constants.ALPHA;
                StaticSharedValues.budget = Constants.BUDGET+1;
                if(Constants.R_TOGGLE == 1) {
                    double[][] deviations = new double[data.getTasks()][data.getTasks()+1];
                    for (int i = 0; i < data.getTasks(); i++) {
                        for (int j = 0; j < data.getTasks()+1; j++) {
                            deviations[i][j] = HF.truncate(data.getCrewTravelTimeMatrix()[i][j] * 10 * alpha,10) * 0.1;
                        }
                    }
                    data.setCrewTravelTimeDeviations(deviations);
                } else {
                    double[] taskDeviations = new double[data.getTasks()+1];
                    for (int i = 1; i < data.getTasks(); i++) {
                        double v2 = data.getServiceTimeMatrix()[i];
                        taskDeviations[i] = HF.truncate(v2 * 10 * alpha,10) * 0.1;
                    }
                    data.setTaskRepairTimeDeviations(taskDeviations);
                }
            } else
                StaticSharedValues.budget = 0;
            //TIGHTER COUNT
            {
                int maxCountForCrews = 0;
                boolean[] tabu = new boolean[data.getTasks()];
                double time = 0;
                while (true) {
                    double min = Double.MAX_VALUE;
                    int selected = -1;
                    for (int i = 1; i < data.getTasks(); i++) {
                        if(!tabu[i]) {
                            for (int j = 0; j < data.getTasks(); j++) {
                                double v1 = data.getCrewTravelTimeMatrix()[j][i] + data.getServiceTimeMatrix()[i];
                                if(v1 < min) {
                                    min = v1;
                                    selected = i;
                                }
                            }
                        }
                    }
                    if(selected == -1)
                        break;
                    if(time + min > data.getHorizon())
                        break;
                    maxCountForCrews++;
                    time = time + min;
                    tabu    [selected] = true;
                }

                double depot_min = Double.MAX_VALUE;
                for (int i = 1; i < data.getTasks(); i++)
                    depot_min = Math.min(depot_min,data.getCrewTravelTimeMatrix()[i][data.getDepotEnd(true)]);
                if(time+depot_min > data.getHorizon()-1e-6)
                    maxCountForCrews--;
                maxCountForCrews = Math.min(maxCountForCrews,data.getTasks()-1);
                System.out.println("A crew route can visit at most " + maxCountForCrews + " task locations / " + (data.getTasks()-1));
                data.setMaxCrewVisits(maxCountForCrews);
            }




            //TIME PROCESSING FOR DP
            {
                double start = System.nanoTime();
                int step = 1;
                int horizon = (int) (data.getHorizon()*step);
                int depotEnd = data.getDepotEnd(true);
                int H = (int) data.getHorizon();
                long[][] i2t2nI = new long[data.getTasks()+1][horizon+1];
                for (int i = 1; i < data.getTasks(); i++) {
                    long[] longsOfI = new long[horizon+1];
                    longsOfI[0] = 0x00L | (1L << i);
                    for (int t = 1; t < horizon+1; t++) {
                        long unreachable = longsOfI[t-1];
                        for (int j = 1; j < data.getTasks(); j++) {
                            if((unreachable & (1L << j)) != 0x00L)
                                continue;
                            double real = t * step + data.getCrewTravelTimeMatrix()[i][j] + data.getServiceTimeMatrix()[j]
                                    + data.getCrewTravelTimeMatrix()[j][depotEnd];
                            if(real > H)
                                unreachable = unreachable | (1L << j);
                        }
                        int j = depotEnd;
                        double real = t * step + data.getCrewTravelTimeMatrix()[i][j] + data.getServiceTimeMatrix()[j];
                        if(real > H)
                            unreachable = unreachable | (1L << j);
                        longsOfI[t] = unreachable;
                    }
                    i2t2nI[i] = longsOfI;
                }
                double passed = (System.nanoTime() - start)*1e-9;
                data.setCrewDpLongs(i2t2nI);
            }

            {
                long[] arr  = new long[data.getNodeNumber()];
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    long vI = 0x00L;
                    for(Integer q : data.getZone2tasks()[i])
                        vI = vI | (1L << q);
                    arr[i] = vI;
                }
                data.setZone2taskLongs(arr);
            }

            {
                System.out.println("ZONE TO TASKS:");
                for (int i = 1; i < data.getNodeNumber(); i++)
                    System.out.println("\t"+i+":"+data.getZone2tasks()[i].toString());
                System.out.println("TASK TO ZONES:");
                for (int i = 1; i < data.getTasks(); i++)
                    System.out.println("\t"+i+":"+data.getTask2zones()[i].toString());
            }


            {
                //PROCESSING FOR TOWER DP
                int maximumTowerRouteLength = 0;
                {
                    PriorityQueue<Tuple> tuples = new PriorityQueue<>(new Comparator<Tuple>() {
                        @Override
                        public int compare(Tuple o1, Tuple o2) {
                            return Double.compare(o1.v,o2.v);
                        }
                    });
                    for (int i = 1; i < data.getNodeNumber(); i++) {
                        double min = Double.MAX_VALUE;
                        for (int j = 0; j < data.getNodeNumber(); j++) {
                            min = Math.min(min,data.getTowerTravelTimeMatrix()[j][i]);
                        }
                        min = min + data.getPositionTimeMatrix()[i];
                        tuples.add(new Tuple(i,min));
                    }
                    double at = 0;
                    int visitCount = 0;
                    while (at < data.getHorizon()){
                        if(tuples.isEmpty())
                            break;
                        double t = tuples.poll().v;
                        if(at + t > data.getHorizon())
                            break;
                        at = at + t;
                        visitCount++;
                    }
                    maximumTowerRouteLength = visitCount;

                    int maximumTowerRouteLength2 = computeMaximumTowerRouteLengthUpperBound();
                    maximumTowerRouteLength = Math.min(maximumTowerRouteLength,maximumTowerRouteLength2);

                }
                StaticSharedValues.maximumTowerRouteLength = maximumTowerRouteLength;
                if(Utility.algo != 29)
                    StaticSharedValues.maximumTowerRouteLength = data.getNodeNumber()-1;
                if(data.getNodeNumber() > 20){
                    if(Utility.algo == 29){
                        StaticSharedValues.maximumTowerRouteLength = maximumTowerRouteLength;
                    }
                }

                System.out.println("A Tower Route can visit at most " +StaticSharedValues.maximumTowerRouteLength+" zone locations / " +(data.getNodeNumber()-1));
            }

            for (int i = 1; i < data.getTasks(); i++) {
                double v1 = data.getCrewTravelTimeMatrix()[0][i];
                double v2 = data.getServiceTimeMatrix()[i];
                double v3 = data.getCrewTravelTimeMatrix()[i][data.getDepotEnd(true)];
                if(v1 + v2 + v3 > data.getHorizon())
                    throw new IllegalArgumentException("instance " + filePath + " is infeasible:"+i+":"+(v1+v2+v3));
            }


            if(Utility.algo==266){
                double[] taskWegith = new double[data.getTasks()];
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    double w = weights[i];
                    for(Integer q : data.getZone2tasks()[i]){
                        taskWegith[q] += w;
                    }
                }
                data.setTaskWeights(taskWegith);

                StaticCounters.hasSharedTask = false;
                boolean[] isShared = new boolean[data.getTasks()];
                for (int q = 1; q < data.getTasks(); q++) {
                    if(data.getTask2zones()[q].size() > 1) {
                        isShared[q] = true;
                        StaticCounters.hasSharedTask = true;
                    }
                }
                data.setSharedTasks(isShared);
            }

            if(Utility.algo == 41 || Utility.algo == 12 || Constants.COST_OF_PRIORITY || Utility.algo==266){
                priorities[0] = 9999;
                priorities[data.getDepotEnd(false)] = 0;
                data.setZonePriority(priorities);
                int[] taskPriority = new int[data.getTasks()+1];
                for (int i = 1; i < data.getNodeNumber(); i++) {
                    int p1 = priorities[i];
                    for(Integer q : data.getZone2tasks()[i]){
                        taskPriority[q] = Math.max(taskPriority[q],p1);
                    }
                }
                taskPriority[0] = 9999;
                taskPriority[data.getDepotEnd(true)] = 0;
                data.setTaskPriority(taskPriority);
            }


            if(Constants.ROBUST){
                for (int i = 1; i < data.getTasks(); i++) {
                    double v = data.getCrewTravelTimeMatrix()[0][i];
                    double r = data.getServiceTimeMatrix()[i] + data.getServiceDeviation()[i];
                    double v2 = data.getCrewTravelTimeMatrix()[i][data.getDepotEnd(true)];
                    if(v + r + v2 > data.getHorizon()) {
                        System.out.println("Modifying:" + (v + r + v2) + ">" + data.getHorizon() + " for task " + i);
                        double modified = data.getHorizon() - v - v2 - data.getServiceTimeMatrix()[i];
                        data.modifyServiceDeviation(modified,i);
                        i--;
                        continue;
                    }
                }
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private int computeMaximumTowerRouteLengthUpperBound() {

        Data data = Data.getInstance();

        int N = data.getNodeNumber();      // depot is 0, customers are 1,...,N-1
        int n = N - 1;

        double[][] travel = data.getTowerTravelTimeMatrix();
        double[] position = data.getPositionTimeMatrix();
        double W = data.getHorizon();

        if (n <= 0) {
            return 0;
        }

        final double INF = 1e100;
        final double EPS = 1e-9;

        /*
         * ------------------------------------------------------------
         * Arrays over customers 1,...,N-1.
         * idx = i - 1 maps customer i to array index idx.
         * ------------------------------------------------------------
         */
        double[] firstCost = new double[n];        // c_0i + p_i
        double[] continuationCost = new double[n]; // min_{j in customers, j != i} c_ji + p_i
        double[] incomingCost = new double[n];     // min_{j in {0}+customers, j != i} c_ji + p_i
        double[] positionOnly = new double[n];

        double minDepotArc = INF;

        for (int i = 1; i < N; i++) {

            int idx = i - 1;

            positionOnly[idx] = position[i];

            firstCost[idx] = travel[0][i] + position[i];

            minDepotArc = Math.min(minDepotArc, travel[0][i]);

            double minIncomingFromCustomer = INF;

            for (int j = 1; j < N; j++) {
                if (j == i) continue;

                minIncomingFromCustomer =
                        Math.min(minIncomingFromCustomer, travel[j][i]);
            }

            /*
             * If there is only one customer, then there is no possible customer-to-customer
             * continuation. This value will only be used for q >= 2.
             */
            continuationCost[idx] =
                    minIncomingFromCustomer >= INF / 2
                            ? INF
                            : minIncomingFromCustomer + position[i];

            double minIncomingFromDepotOrCustomer = travel[0][i];

            for (int j = 1; j < N; j++) {
                if (j == i) continue;

                minIncomingFromDepotOrCustomer =
                        Math.min(minIncomingFromDepotOrCustomer, travel[j][i]);
            }

            incomingCost[idx] = minIncomingFromDepotOrCustomer + position[i];
        }

        /*
         * ------------------------------------------------------------
         * Bound B ingredients:
         * q smallest position times and q-1 smallest customer-to-customer arcs.
         * ------------------------------------------------------------
         */
        Arrays.sort(positionOnly);

        double[] prefixPosition = new double[n + 1];

        for (int q = 1; q <= n; q++) {
            prefixPosition[q] = prefixPosition[q - 1] + positionOnly[q - 1];
        }

        ArrayList<Double> interCustomerArcs = new ArrayList<>();

        for (int i = 1; i < N; i++) {
            for (int j = 1; j < N; j++) {
                if (i == j) continue;

                interCustomerArcs.add(travel[i][j]);
            }
        }

        interCustomerArcs.sort(Double::compare);

        double[] prefixInterArc = new double[n];

        prefixInterArc[0] = 0.0;

        for (int k = 1; k <= n - 1; k++) {
            if (k <= interCustomerArcs.size()) {
                prefixInterArc[k] =
                        prefixInterArc[k - 1] + interCustomerArcs.get(k - 1);
            } else {
                prefixInterArc[k] = INF;
            }
        }

        /*
         * ------------------------------------------------------------
         * Bound C ingredients:
         * q smallest incomingCost values.
         * ------------------------------------------------------------
         */
        double[] sortedIncoming = incomingCost.clone();
        Arrays.sort(sortedIncoming);

        double[] prefixIncoming = new double[n + 1];

        for (int q = 1; q <= n; q++) {
            prefixIncoming[q] = prefixIncoming[q - 1] + sortedIncoming[q - 1];
        }

        /*
         * ------------------------------------------------------------
         * Bound A ingredients:
         * firstCost[i] + sum of q-1 smallest continuationCost excluding i.
         * ------------------------------------------------------------
         */
        TupleForBound[] continuationTuples = new TupleForBound[n];

        for (int idx = 0; idx < n; idx++) {
            continuationTuples[idx] = new TupleForBound(idx, continuationCost[idx]);
        }

        Arrays.sort(
                continuationTuples,
                Comparator.comparingDouble(t -> t.value)
        );

        double[] sortedContinuation = new double[n];
        int[] rankOfCustomer = new int[n];

        for (int rank = 0; rank < n; rank++) {
            sortedContinuation[rank] = continuationTuples[rank].value;
            rankOfCustomer[continuationTuples[rank].customerIndex] = rank;
        }

        double[] prefixContinuation = new double[n + 1];

        for (int q = 1; q <= n; q++) {
            prefixContinuation[q] =
                    prefixContinuation[q - 1] + sortedContinuation[q - 1];
        }

        /*
         * ------------------------------------------------------------
         * Now compute the largest q for which the combined lower bound
         * does not exceed W.
         * ------------------------------------------------------------
         */
        int upperBound = 0;

        for (int q = 1; q <= n; q++) {

            double lbA = computeFirstPlusContinuationLB(
                    q,
                    firstCost,
                    continuationCost,
                    rankOfCustomer,
                    prefixContinuation,
                    n,
                    INF
            );

            double lbB =
                    minDepotArc
                            + prefixInterArc[q - 1]
                            + prefixPosition[q];

            double lbC = prefixIncoming[q];

            double combinedLB = Math.max(lbA, Math.max(lbB, lbC));

            if (combinedLB <= W + EPS) {
                upperBound = q;
            } else {
                /*
                 * Because all three component bounds are nondecreasing in q
                 * under nonnegative travel and position times, we can stop.
                 */
                break;
            }
        }

        return upperBound;
    }

    /**
     * Computes:
     *
     * LB_A(q) = min_i { firstCost[i] + sum of q-1 smallest continuation costs excluding i }.
     */
    private double computeFirstPlusContinuationLB(
            int q,
            double[] firstCost,
            double[] continuationCost,
            int[] rankOfCustomer,
            double[] prefixContinuation,
            int n,
            double INF
    ) {

        if (q == 0) {
            return 0.0;
        }

        if (q == 1) {
            double best = INF;

            for (double v : firstCost) {
                best = Math.min(best, v);
            }

            return best;
        }

        int m = q - 1;

        if (m > n - 1) {
            return INF;
        }

        double best = INF;

        for (int i = 0; i < n; i++) {

            double continuationSumExcludingI;

            int rank = rankOfCustomer[i];

            /*
             * If customer i is not among the m smallest continuation costs,
             * the m-smallest sum already excludes it.
             */
            if (rank >= m) {
                continuationSumExcludingI = prefixContinuation[m];
            } else {
                /*
                 * If customer i is among the m smallest continuation costs,
                 * remove it and include the next cheapest one.
                 */
                if (m + 1 > n) {
                    continuationSumExcludingI = INF;
                } else {
                    continuationSumExcludingI =
                            prefixContinuation[m + 1] - continuationCost[i];
                }
            }

            best = Math.min(best, firstCost[i] + continuationSumExcludingI);
        }

        return best;
    }

    private static class TupleForBound {

        final int customerIndex;
        final double value;

        TupleForBound(int customerIndex, double value) {
            this.customerIndex = customerIndex;
            this.value = value;
        }
    }


    private double[] getEarliestServiceStartTimes(int zones, double[][] travelTower, double[] positionTimes) {
        double[] earlisetStart = new double[zones+1];
        for (int i = 1; i < zones+1; i++) {
            earlisetStart[i] = travelTower[0][i] + positionTimes[i];
        }
        return earlisetStart;
    }


    private double getTralveTowerM(int zones, double[][] travelCrew, double[] repairTimes, double[][] travelTower, double[] positionTimes, double serviceM) {
        int count = 0;
        boolean[] visited = new boolean[zones + 1];
        double m = 0;
        while (count < zones) {
            double max = 0.;
            int candidate = -1;
            for (int j = 1; j < zones + 1; j++) {
                if (!visited[j]) {
                    for (int i = 0; i < zones + 1; i++) {
                        if(i!=j){
                            double v1 = travelTower[i][j] + positionTimes[j];
                            if (v1 > max) {
                                max = v1;
                                candidate = j;
                            }
                        }

                    }
                }
            }
            m += max;
            visited[candidate] = true;
            count++;
        }

        double max = 0.;
        for (int i = 1; i < zones + 1; i++) {
            double v1 = travelTower[i][0];
            if (v1 > max) {
                max = v1;
            }
        }

        return m + max;
    }


    private double getCrewM(int zones, double[][] travelCrew, double[] repairTimes) {
        Data data = Data.getInstance();
        int tasks = data.getTasks()-1;
        double v = 0.;
        double max = 0.;
        //1. take the maximum edge value between tasks / excluding the depot
        //2. multiply by number of tasks -1
        //3. add one mroe depot edge
        //4. + return to depot
        for (int i = 1; i < tasks; i++) {
            for (int j = i+1; j < tasks+1; j++) {
                double v1 = travelCrew[i][j] + Math.max(repairTimes[i],repairTimes[j]);
                max = Math.max(max,v1);
            }
        }
        v = max * (tasks-1)  ;

        max = 0;
        for (int k = 1; k < tasks+1; k++)
            max = Math.max(max,travelCrew[0][k]+repairTimes[k]);

        v = v+max;

        max = 0;
        for (int k = 1; k < tasks+1; k++)
            max = Math.max(max,travelCrew[0][k]);
        return v+max;
    }

    private double getTowerM(int zones, double[][] travelCrew, double[] repairTimes, double[][] travelTower, double[] positionTimes, double serviceM) {
        //service each customer
        //add depot edge
        double v = 0.;
        double max = 0.;
        for (int i = 1; i < zones; i++) {
            for (int j = i+1; j < zones+1; j++) {
                max = Math.max(max,travelTower[i][j] + positionTimes[j]);
            }
        }
        max += serviceM;
        v = max*(zones-1);

        max = 0; //for end and start depots
        for (int i = 1; i < zones+1; i++) {
            max = Math.max(max,travelTower[0][i]);
        }

        v += (max*2)+serviceM;
        return v;
    }

    private double getServiceM(int zones, double[][] travelCrew, double[] repairTimes, double[][] travelTower, double[] positionTimes) {
        Data data = Data.getInstance();
        ArrayList<Double> bigMs = new ArrayList<>();

        //This implementation's service M is based on crew shift length
        double serviceM = 0;
        for (int i = 1; i < data.getNodeNumber(); i++) {

            //EARLIEST SERVICE START TIME
            double towerFromDepotToZone = data.getTowerTravelTimeMatrix()[0][i] + data.getPositionTimeMatrix()[i];

            //LATEST SERVICE END TIME
            double minCrewTimeFromTaskToDepot = Double.MAX_VALUE;
            for(Integer j : data.getZone2tasks()[i]){
                double v1 = data.getCrewTravelTimeMatrix()[j][data.getDepotEnd(true)];
                if(v1 < minCrewTimeFromTaskToDepot) minCrewTimeFromTaskToDepot = v1;
            }

            //remaining length due to crew
            double remainingServiceLength = data.getHorizon() - minCrewTimeFromTaskToDepot;
            //remaining length due to tower
            remainingServiceLength = remainingServiceLength - towerFromDepotToZone;

            bigMs.add(remainingServiceLength);

            serviceM = Math.max(serviceM,remainingServiceLength);
        }

        bigMs.sort(new Comparator<Double>() {
            @Override
            public int compare(Double o1, Double o2) {
                return -1 * Double.compare(o1,o2);
            }
        });

        double maximumValueOfTotalServiceDurationOverAllTowers = 0;
        for (int k = 0; k < data.getTowerNumber(); k++)
            maximumValueOfTotalServiceDurationOverAllTowers += bigMs.removeFirst();

        data.setMaximumZoverAllTowers(maximumValueOfTotalServiceDurationOverAllTowers);

        return serviceM;
    }
}
