package problem.BP;

import lib.*;
import problem.Constants;
import problem.SpecificFunctions;
import problem.graph.Data;

import java.util.*;
import java.util.concurrent.ThreadLocalRandom;

public class DyB {
    private int nodeNum;
    private Data data;
    private boolean isCrew;
    private ArrayList<Route> routes;
    private LP lp;
    private HashSet<ArrayList<Integer>> scheduleHash;
    private HashMap<ArrayList<Integer>,HashMap<ArrayList<Boolean>,Integer>> innerMap;
    private boolean heuristic;
    private double bigM;


    private boolean nonElemC;

    private Cet cX;




    private boolean[] tabu;
    public void run(LP lp, boolean isCrew, boolean isHeuristic) {

        if(isCrew){
            runBiDirCrew(lp,isHeuristic);
            if(Constants.GUIDED_PRICING_HEURISTIC){
                for(Route route : routes){
                    ArrayList<Integer> schedule = route.getPattern();
                    for(Integer i : schedule){
                        this.customer2numberOfColumns[i]++;
                        for (Integer j : schedule){
                            this.customer2pairInColumns[i][j]++;
                        }
                    }
                }
            }
            return;
        } else {


            throw new IllegalArgumentException("should not be called");
        }
    }








    private boolean REDUCED_HEURISTIC4DyBDP;

    public void toggleReducedHeuristic(boolean toggle){
        REDUCED_HEURISTIC4DyBDP = toggle;
    }



    private void runBiDirCrew(LP lp, boolean isHeuristic) {

        {
            this.isCrew = true;
            heuristic = isHeuristic;
            this.lp = lp;
            data = Data.getInstance();
            bigM = data.getServiceM();

            nodeNum = data.getTasks()+1;
            scheduleHash = new HashSet<>();
            innerMap = new HashMap<>();
            routes = new ArrayList<>();
            nonElemC = Constants.NON_ELEMENTARY_CREW;
            arrayForCrews = new double[nodeNum];
            {
                for (int i = 1; i < data.getNodeNumber(); i++) for(Integer q : data.getZone2tasks()[i]) arrayForCrews[q] += lp.getDual0Psi()[i][q];
            }
            tabu = new boolean[data.getTasks()+1];
            if(true){
                if(this.REDUCED_HEURISTIC4DyBDP) {
                    int maxK = data.getCrewMaxVisit();
                    int count = (data.getTasks()-1);


                    while (count > maxK) {
                        for (int k = 1; k < data.getTasks(); k++) {
                            if(!tabu[k]){
                                if(ThreadLocalRandom.current().nextBoolean()){
                                    tabu[k] =true;
                                    count--;
                                    if(count <= maxK)
                                        break;
                                }
                            }
                        }
                    }
                } else {
                    if(Constants.GUIDED_PRICING_HEURISTIC){
                        PriorityQueue<Tuple> queue;
                        if(this.heuristicIteration > (Constants.GUIDED_HEURISTIC_ITERATIONS/2.)){
                            //diversify the search by redirecting the queue, and selecting the least visited customer
                            queue = new PriorityQueue<>(new Comparator<Tuple>() {
                                @Override
                                public int compare(Tuple o1, Tuple o2) {
                                    return Double.compare(o1.v,o2.v);
                                }
                            });
                        } else {
                            queue = new PriorityQueue<>(new Comparator<Tuple>() {
                                @Override
                                public int compare(Tuple o1, Tuple o2) {
                                    return -1*Double.compare(o1.v,o2.v);
                                }
                            });
                        }
                        for (int i = 1; i < data.getTasks(); i++) queue.add(new Tuple(i,this.customer2numberOfColumns[i]));

                        Tuple first = queue.poll();

                        PriorityQueue<Tuple> queue2;
                        if(this.heuristicRedirection){
                            //diversify the search by redirecting the queue, and selecting the least visited customer
                            queue2 = new PriorityQueue<>(new Comparator<Tuple>() {
                                @Override
                                public int compare(Tuple o1, Tuple o2) {
                                    return Double.compare(o1.v,o2.v);
                                }
                            });
                        } else {
                            queue2 = new PriorityQueue<>(new Comparator<Tuple>() {
                                @Override
                                public int compare(Tuple o1, Tuple o2) {
                                    return -1*Double.compare(o1.v,o2.v);
                                }
                            });
                        }
                        for (int i = 1; i < data.getTasks(); i++) if(i!=first.id) queue2.add(new Tuple(i,this.customer2pairInColumns[first.id][i]));

                        ArrayList<Tuple> set = new ArrayList<>();
                        set.add(first);
                        int maxK = data.getCrewMaxVisit();
                        int count = 1;

                        while (count < maxK){
                            if(queue2.isEmpty()) break;
                            set.add(queue2.poll());
                            count++;
                        }



                        {
                            ArrayList<Integer> integers = new ArrayList<>();
                            for(Tuple t : set) integers.add(t.id);
                            Collections.sort(integers);
                            if(this.heuristicSet.contains(integers)){
                                Random random = new Random(250823);
                                //diversify
                                Collections.shuffle(set);
                                int numberToRemove = random.nextInt(maxK);
                                count = 0;
                                for(Tuple t : set) tabu[t.id] = true;
                                while (count < numberToRemove){
                                    set.removeFirst();
                                    count++;
                                    if(set.isEmpty()) break;
                                }
                                ArrayList<Tuple> candidates = new ArrayList<>();
                                for (int i = 1; i < data.getTasks(); i++) if(!tabu[i]) candidates.add(new Tuple(i,0));
                                Collections.shuffle(candidates);
                                count = set.size();
                                while (count < maxK){
                                    if(candidates.isEmpty()) break;
                                    set.add(candidates.removeFirst());
                                    count++;
                                }
                            }
                        }
                        for (int i = 1; i < data.getTasks(); i++) tabu[i] = true;
                        ArrayList<Integer> integers = new ArrayList<>();
                        for(Tuple t : set) {
                            tabu[t.id] = false;
                            integers.add(t.id);
                        }
                        Collections.sort(integers);

                        this.heuristicSet.add(integers);
                    }
                }
            }
        }

        Comparator<Label> nonIncreasing = (o1, o2) -> Double.compare(o2.c, o1.c);
        int halfWayPoint = (int) Math.ceil(StaticSharedValues.crewWorkTime / 2.);
        double bHp = (int) Math.ceil(StaticSharedValues.crewWorkTime - halfWayPoint);
        double fHp = bHp;
        if(fHp + bHp < StaticSharedValues.crewWorkTime - 1e-6)
            throw new IllegalArgumentException("halfway points do not match");

        if(true) {
            fHp = StaticSharedValues.crewWorkTime;
            bHp = StaticSharedValues.crewWorkTime;
        }

        if(heuristic)
            cX = SpecificFunctions.allTaskSet;
        else
            cX = lp.getBNode().getCx().copy();

        while (true) {
            Label best = new Label(); best.c = -1.*Double.MAX_VALUE;
            ArrayList<Label>[] fT = new ArrayList[nodeNum];
            for (int i = 0; i < fT.length; i++) fT[i] = new ArrayList<>();
            ArrayList<Label>[] bT = new ArrayList[nodeNum];
            for (int i = 0; i < bT.length; i++) bT[i] = new ArrayList<>();
            //-----------------------------------------------
            //-----------------------------------------------
            //FORWARD LABELING
            double start = System.nanoTime();
            if(true){

                PriorityQueue<Label> fQ = new PriorityQueue<>(nonIncreasing);
                {
                    Label label = new Label();
                    label.i = 0;
                    label.c = lp.getDualVisitC()[0];
                    label.t = 0;
                    label.crewRouteSize = 0;
                    label.n = new Cet(nodeNum);
                    label.m = new Cet(nodeNum);

                    if(Constants.ROBUST)
                        label.r_t = new double[StaticSharedValues.budget];
                    fQ.offer(label);
                }

                int iter = 0;
                while (true) {
                    iter++;
                    Label parent = fQ.poll();
                    if (parent == null)
                        break;
                    if (parent.dominated)
                        continue;
                    for (Label theirs : fT[parent.i]) {
                        if (theirs.dominatesCrewForward(parent)) {
                            parent.dominated = true;
                            break;
                        }
                    }
                    if(parent.dominated)
                        continue;
                    fT[parent.i].add(parent);
                    if(parent.i == data.getDepotEnd(isCrew))
                        continue;
                    if(parent.t >= fHp)
                        continue;
                    int i = parent.i;
                    for (int j = 1; j < nodeNum; j++) {
                        if ((j == i || i == 0 && j == data.getDepotEnd(isCrew))
                                || !lp.isFeasibleArc(i, j, isCrew)
                                || violatesPriorityExtension(i, j)) {
                            continue;
                        }
                        if (parent.n.contains(j) && cX.contains(j))
                            continue;
                        if(tabu[j])
                            continue;

                        if(j==data.getDepotEnd(isCrew)){
                            if(parent.t + data.getCrewTravelTimeMatrix()[i][j]
                                    > StaticSharedValues.crewWorkTime)
                                continue;
                        } else {
                            if(parent.t
                                    + data.getServiceTimeMatrix()[j] + data.getCrewTravelTimeMatrix()[i][j]
                                    + data.getCrewTravelTimeMatrix()[j][data.getDepotEnd(isCrew)] > StaticSharedValues.crewWorkTime)
                                continue;
                        }


                        Label label = new Label();
                        label.p = parent;
                        label.i = j;
                        label.crewRouteSize = parent.crewRouteSize
                                + (j == data.getDepotEnd(isCrew) ? 0 : 1);


                        label.t = parent.t + data.getServiceTimeMatrix()[j] + data.getCrewTravelTimeMatrix()[i][j];
                        label.m = parent.m.copy();
                        if(parent.n.contains(j))
                            label.m.add(j);
                        label.n = parent.n.copy();
                        label.n.add(j);

                        label.c = parent.c + lp.getDualVisitC()[j];
                        double c1 = label.c;
                        label.c = label.c + label.t * arrayForCrews[j];



                        if(Constants.ROBUST){
                            double t1 = data.getServiceTimeMatrix()[j] + data.getCrewTravelTimeMatrix()[i][j];
                            label.r_t = new double[StaticSharedValues.budget];
                            label.r_t[0] = parent.r_t[0] + t1;
                            if(Constants.R_TOGGLE == 0) {
                                for (int k = 1; k < label.r_t.length; k++) {
                                    label.r_t[k] = Math.max(parent.r_t[k] + t1
                                            , parent.r_t[k-1]+t1+data.getServiceDeviation()[j]);
                                }
                            } else {
                                for (int k = 1; k < label.r_t.length; k++) {
                                    label.r_t[k] = Math.max(parent.r_t[k] + t1
                                            , parent.r_t[k-1]+t1+data.getCrewTravelTimeDeviations()[i][j]);
                                }
                            }
                            for (int k = 0; k < label.r_t.length; k++) label.t = Math.max(label.t,label.r_t[k]);
                            label.c = c1 + label.t * arrayForCrews[j];
                        }




                        if(j == data.getDepotEnd(isCrew)){
                            fT[label.i].add(label);
                        } else {
                            dominanceCheckForward(fT, label, true);
                            if (!label.dominated) fQ.add(label);
                        }
                    }
                }


                {
                    PriorityQueue<Label> labels = new PriorityQueue<>(nonIncreasing);
                    labels.addAll(fT[data.getDepotEnd(true)]);

                    while (routes.size() <= 100 && !labels.isEmpty()){
                        Label label = labels.poll();
                        if(label == null)
                            throw new IllegalArgumentException("null");
                        if(label.dominated)
                            continue;
                        if(label.c > best.c){
                            best = label;
                        }
                        if(!label.m.isEmpty())
                            continue;
                        if(label.c > 1e-6){
                            Route r = getRoute(label,true);
                            if (r != null)
                                routes.add(r);
                        } else {
                            break;
                        }
                    }
                }
            }




            if(!routes.isEmpty())
                return;
            //-----------------------------------------------
            //-----------------------------------------------
            //-----------------------------------------------


            //-----------------------------------------------
            //-----------------------------------------------
            //BACKWARD LABELING
            start = System.nanoTime();
            if(true) {

                PriorityQueue<Label> fQ = new PriorityQueue<>(nonIncreasing);
                {
                    Label label = new Label();
                    label.i = data.getDepotEnd(isCrew);
                    label.c = 0;
                    label.t = Constants.CREW_WORK_LENGTH;
                    label.crewRouteSize = 0;
                    label.n = new Cet(nodeNum);
                    label.m = new Cet(nodeNum);

                    if(Constants.ROBUST) {
                        label.r_t = new double[StaticSharedValues.budget];
                        Arrays.fill(label.r_t, label.t);
                    }

                    fQ.offer(label);
                }

                int iter = 0;
                while (true) {
                    iter++;
                    Label parent = fQ.poll();
                    if (parent == null)
                        break;
                    if (parent.dominated)
                        continue;
                    for (Label theirs : bT[parent.i]) {
                        if (theirs.dominatesCrewBackward(parent)) {
                            parent.dominated = true;
                            break;
                        }
                    }
                    if(parent.dominated)
                        continue;
                    bT[parent.i].add(parent);

                    if(parent.i == 0)
                        continue;
                    if(parent.t <= bHp)
                        continue;

                    for (int i = 0; i < nodeNum-1; i++) {
                        int j = parent.i;
                        if ((i == j || i == 0 && j == data.getDepotEnd(isCrew))
                                || !lp.isFeasibleArc(i, j, isCrew)
                                || violatesPriorityExtension(i, j)) {
                            continue;
                        }
                        if (parent.n.contains(i) && cX.contains(i))
                            continue;
                        if(i==0){
                            if(parent.t - data.getCrewTravelTimeMatrix()[i][j]
                                    < 0.)
                                continue;
                        } else {
                            if(parent.t
                                    - data.getServiceTimeMatrix()[i] - data.getCrewTravelTimeMatrix()[i][j]
                                    - data.getCrewTravelTimeMatrix()[0][i] < 0.)
                                continue;
                        }


                        Label label = new Label();
                        label.p = parent;
                        label.i = i;
                        label.crewRouteSize = parent.crewRouteSize + (i == 0 ? 0 : 1);



                        label.t = parent.t - data.getServiceTimeMatrix()[i] - data.getCrewTravelTimeMatrix()[i][j];



                        label.m = parent.m.copy();
                        if(parent.n.contains(i))
                            label.m.add(i);
                        label.n = parent.n.copy();
                        label.n.add(i);



                        label.c = parent.c + lp.getDualVisitC()[i]
                                + parent.y * (data.getServiceTimeMatrix()[i] + data.getCrewTravelTimeMatrix()[i][j])
                                + data.getServiceTimeMatrix()[i] * arrayForCrews[i];
                        label.y = parent.y + arrayForCrews[i];




                        if(Constants.ROBUST){
                            label.c = parent.c + lp.getDualVisitC()[i];

                            double t1 = data.getServiceTimeMatrix()[i] + data.getCrewTravelTimeMatrix()[i][j];
                            label.r_t = new double[StaticSharedValues.budget];
                            label.r_t[0] = parent.r_t[0] - t1;
                            boolean worst_trigger = false;
                            double worst_val = label.r_t[0];
                            if(Constants.R_TOGGLE == 0) {
                                for (int k = 1; k < label.r_t.length; k++) {
                                    double v1 = parent.r_t[k] - t1;
                                    double v2 = parent.r_t[k-1]-t1-data.getServiceDeviation()[i];
                                    if(v1 < v2 - 1e-6){
                                        label.r_t[k] = v1;
                                        if(v1 < worst_val - 1e-6){
                                            worst_val = v1;
                                            worst_trigger = false;
                                        }
                                    } else {
                                        label.r_t[k] = v2;
                                        if(v2 < worst_val - 1e-6){
                                            worst_val = v2;
                                            worst_trigger = true;
                                        }
                                    }
                                }
                                if(worst_trigger) {
                                    label.c = label.c
                                            + parent.y * (data.getServiceTimeMatrix()[i] + data.getCrewTravelTimeMatrix()[i][j] + data.getServiceDeviation()[i])
                                            + (data.getServiceTimeMatrix()[i] + data.getServiceDeviation()[i]) * arrayForCrews[i];
                                } else {
                                    label.c = label.c
                                            + parent.y * (data.getServiceTimeMatrix()[i] + data.getCrewTravelTimeMatrix()[i][j])
                                            + data.getServiceTimeMatrix()[i] * arrayForCrews[i];
                                }
                            } else {
                                Msg.incompleteMethod();
                                for (int k = 1; k < label.r_t.length; k++) {
                                    label.r_t[k] = Math.min(parent.r_t[k] - t1
                                            , parent.r_t[k-1]-t1-data.getCrewTravelTimeDeviations()[i][j]);
                                }
                            }

                            for (int k = 0; k < label.r_t.length; k++) label.t = Math.min(label.t,label.r_t[k]);
                        }



                        if(i == 0){
                            bT[label.i].add(label);
                        } else {
                            dominanceCheckForward(bT, label, false);
                            if (!label.dominated) fQ.add(label);
                        }
                    }
                }


                {
                    PriorityQueue<Label> labels = new PriorityQueue<>(nonIncreasing);
                    labels.addAll(bT[0]);

                    while (routes.size() <= 100 && !labels.isEmpty()){
                        Label label = labels.poll();
                        if(label == null)
                            throw new IllegalArgumentException("null");
                        if(label.dominated)
                            continue;
                        if(label.c > best.c){
                            best.m = label.m;
                            best.c = label.c;
                        }
                        if(!label.m.isEmpty())
                            continue;
                        if(label.c > 1e-6){
                            Route r = getRoute(label,false);
                            if (r != null)
                                routes.add(r);
                        } else {
                            break;
                        }
                    }
                }
            }

            if(!routes.isEmpty())
                return;
            //-----------------------------------------------
            //-----------------------------------------------
            //-----------------------------------------------


            //-----------------------------------------------
            //-----------------------------------------------
            //JOINING
            start = System.nanoTime();
            if(true){
                PriorityQueue<Label> labels = new PriorityQueue<>(nonIncreasing);
                for (int i = 0; i < nodeNum; i++) {
                    fT[i].removeIf(k -> k.dominated);
                    fT[i].sort(new Comparator<Label>() {
                        @Override
                        public int compare(Label o1, Label o2) {
                            return Double.compare(o1.t,o2.t);
                        }
                    });
                }
                for (int i = 1; i < nodeNum-1; i++) {
                    int j = data.getDepotEnd(isCrew);
                    double dij = data.getCrewTravelTimeMatrix()[i][j];
                    for(Label fL : fT[i]) {
                        if(fL.t + dij > StaticSharedValues.crewWorkTime)
                            break;
                        if (!lp.isFeasibleArc(i, j, isCrew)
                                || violatesPriorityExtension(i, j)) {
                            continue;
                        }
                        for(Label bL : bT[j]){
                            Label label = new Label();
                            label.i = -1;
                            label.m = fL.m.copy();
                            label.n = fL.n.copy();
                            label.crewRouteSize = fL.crewRouteSize + bL.crewRouteSize;
                            label.c = fL.c + bL.c;
                            label.p = fL;
                            label.p2 = bL;
                            labels.add(label);
                        }
                    }
                }

                while (routes.size() <= 100 && !labels.isEmpty()){
                    Label label = labels.poll();
                    if(label == null)
                        throw new IllegalArgumentException("null");
                    if(label.dominated)
                        continue;
                    if(label.c > best.c){
                        best.m = label.m;
                        best.c = label.c;
                    }
                    if(!label.m.isEmpty())
                        continue;
                    if(label.c > 1e-6){
                        Route r = getRouteJoined(label);
                        if (r != null)
                            routes.add(r);
                    } else
                        break;
                }
                if(!routes.isEmpty())
                    break;

                //---------------------------------------------------------
                //---------------------------------------------------------
                //---------------------------------------------------------
                labels.clear();
                for (int i = 0; i < nodeNum; i++) {
                    bT[i].removeIf(k -> k.dominated);
                    bT[i].sort(new Comparator<Label>() {
                        @Override
                        public int compare(Label o1, Label o2) {
                            return Double.compare(o2.t,o1.t);
                        }
                    });
                }
                for (int j = 1; j < nodeNum-1; j++) {
                    int i = 0;
                    double dij = data.getCrewTravelTimeMatrix()[i][j];
                    for(Label bL : bT[j]){
                        if(bL.t - dij < 0)
                            break;
                        if (!lp.isFeasibleArc(i, j, isCrew)
                                || violatesPriorityExtension(i, j)) {
                            continue;
                        }
                        for(Label fL : fT[i]) {
                            Label label = new Label();
                            label.i = -1;
                            label.m = bL.m.copy();
                            label.n = bL.n.copy();

                            label.crewRouteSize = fL.crewRouteSize + bL.crewRouteSize;

                            label.c = fL.c + bL.c
                                    + bL.y * (fL.t + dij);
                            label.p = fL;
                            label.p2 = bL;
                            labels.add(label);
                        }
                    }
                }

                while (routes.size() <= 100 && !labels.isEmpty()){
                    Label label = labels.poll();
                    if(label == null)
                        throw new IllegalArgumentException("null");
                    if(label.dominated)
                        continue;
                    if(label.c > best.c){
                        best.m = label.m;
                        best.c = label.c;
                    }
                    if(!label.m.isEmpty())
                        continue;
                    if(label.c > 1e-6){
                        Route r = getRouteJoined(label);
                        if (r != null)
                            routes.add(r);
                    } else
                        break;
                }
                if(!routes.isEmpty())
                    break;

                //---------------------------------------------------------
                //---------------------------------------------------------
                //---------------------------------------------------------
                labels.clear();
                for (int i = 1; i < nodeNum-1; i++) {
                    for (int j = 1; j < nodeNum-1; j++){
                        if (j == i
                                || !lp.isFeasibleArc(i, j, isCrew)
                                || violatesPriorityExtension(i, j)) {
                            continue;
                        }
                        double dij = data.getCrewTravelTimeMatrix()[i][j];
                        for(Label fL : fT[i]){
                            if (fL.n.contains(j) && cX.contains(j))
                                continue;
                            if(fL.t + dij > StaticSharedValues.crewWorkTime)
                                break;
                            for(Label bL : bT[j]) {
                                if(bL.t + 1e-6 < fL.t + dij)
                                    break;
                                long[] v1 = fL.n.intersectCet(cX).V;
                                long[] v2 = bL.n.intersectCet(cX).V;
                                boolean feasible = true;
                                for (int k = 0; k < v1.length; k++) {
                                    if((v1[k] & v2[k]) != 0x00L){
                                        feasible = false;
                                        break;
                                    }
                                }
                                if(feasible) {

                                    if(Constants.ROBUST){
                                        for (int k = 0; k < StaticSharedValues.budget; k++) {
                                            if(bL.r_t[k] + 1e-6 < fL.r_t[k] + dij) {
                                                feasible = false;
                                                break;
                                            }
                                        }
                                        if(!feasible)
                                            continue;
                                    }

                                    Label label = new Label();
                                    label.i = -1;
                                    label.m = fL.m.copy();
                                    label.m.union(bL.m);
                                    label.m.union(
                                            fL.n.intersectCet(bL.n));
                                    label.n = fL.n.copy();
                                    label.n.union(bL.n);


                                    label.crewRouteSize = fL.crewRouteSize + bL.crewRouteSize;

                                    label.c = fL.c + bL.c
                                            + bL.y * (fL.t + dij);


                                    label.p = fL;
                                    label.p2 = bL;

                                    // Do not mutate label.c after insertion into a PriorityQueue.
                                    labels.add(label);
                                }
                            }
                        }
                    }
                }

                while (routes.size() <= 100 && !labels.isEmpty()){
                    Label label = labels.poll();
                    if(label == null)
                        throw new IllegalArgumentException("null");
                    if(label.dominated)
                        continue;
                    if(label.c > best.c){
                        best.m = label.m;
                        best.c = label.c;
                    }
                    if(!label.m.isEmpty())
                        continue;
                    if(label.c > 1e-6){
                        Route r = getRouteJoined(label);
                        if (r != null)
                            routes.add(r);
                    } else
                        break;
                }
                if(!routes.isEmpty())
                    break;
            }

            if(!routes.isEmpty())
                return;
            //-----------------------------------------------
            //-----------------------------------------------
            //-----------------------------------------------
            //-----------------------------------------------


            //-----------------------------------------------
            //-----------------------------------------------
            //DSSS TERMINATION CONDITIONS
            if(heuristic || best.c <= 1e-6 || best.m.isEmpty())
                return;
            cX.union(best.m);
            lp.getBNode().updateCx(cX);
//            if(Constants.CONSOLE) System.out.println("C-DSSS:"+cX.getIndexList().toString());
        }
    }






    private double[] arrayForCrews;





    private Route getRoute(Label label, boolean isForward) {
        ArrayList<Integer> schedule = new ArrayList<>();
        ArrayList<Boolean> waitBooleans = new ArrayList<>();
        Label temp = label;
        while (temp != null) {
            if(isForward)
                schedule.addFirst(temp.i);
            else
                schedule.add(temp.i);
            if(!isCrew){
                if(temp.z) {
                    if(isForward)
                        waitBooleans.addFirst(true);
                    else
                        waitBooleans.add(true);
                }
                else {
                    if(isForward)
                        waitBooleans.addFirst(false);
                    else
                        waitBooleans.add(false);
                }
            }
            temp = temp.p;
        }


//        HashSet<Integer> debugHash = new HashSet<>(schedule);
//        if(debugHash.size()!=schedule.size()) {
//            System.out.println(label.n.getIndexList().toString());
//            System.out.println(label.m.getIndexList().toString());
//            System.out.println(cX.getIndexList().toString());
//            Msg.getValueMismatchException(1, 2);
//        }
        boolean ter = scheduleHash.contains(schedule);
        if(ter){
            ter = innerMap.get(schedule).containsKey(waitBooleans);
        }
        if(!ter){
            if(lp.r2i(schedule,waitBooleans) == null){
                scheduleHash.add(schedule);
                innerMap.computeIfAbsent(schedule,k->new HashMap<>()).put(waitBooleans,1);

                Route route = new Route();
                route.create(schedule,waitBooleans);
                return route;
            }
        }
        return null;
    }

    private Route getRouteJoined(Label label) {
        ArrayList<Integer> schedule = new ArrayList<>();
        ArrayList<Boolean> waitBooleans = new ArrayList<>();
        Label temp = label.p2;
        while (temp != null) {
            schedule.add(temp.i);
            if(!isCrew)
                waitBooleans.add(temp.z);
            temp = temp.p;
        }

        temp = label.p;
        while (temp != null) {
            schedule.addFirst(temp.i);
            if(!isCrew)
                waitBooleans.addFirst(temp.z);
            temp = temp.p;
        }


        HashSet<Integer> debugHash = new HashSet<>(schedule);
        if(debugHash.size()!=schedule.size()) {
            System.out.println(label.n.getIndexList().toString());
            System.out.println(label.m.getIndexList().toString());
            System.out.println(cX.getIndexList().toString());
            Msg.getValueMismatchException(1, 2);
        }
        boolean ter = scheduleHash.contains(schedule);
        if(ter)
            ter = innerMap.get(schedule).containsKey(waitBooleans);

        if(!ter){
            if(lp.r2i(schedule,waitBooleans) == null){
                scheduleHash.add(schedule);
                innerMap.computeIfAbsent(schedule,k->new HashMap<>()).put(waitBooleans,1);

                Route route = new Route();
                route.create(schedule,waitBooleans);
                return route;
            }
        }
        return null;
    }

    private void dominanceCheckForward(ArrayList<Label>[] fT, Label mine, boolean isForward) {
        Iterator<Label> it = fT[mine.i].iterator();
        if(isForward){
            while (it.hasNext()) {
                Label theirs = it.next();

                if (theirs.dominatesCrewForward(mine)) {
                    mine.dominated = true;
                    break;
                }

                if (mine.dominatesCrewForward(theirs)) {
                    theirs.dominated = true;
                    it.remove();
                }
            }
        } else {
            while (it.hasNext()) {
                Label theirs = it.next();

                if (theirs.dominatesCrewBackward(mine)) {
                    mine.dominated = true;
                    break;
                }

                if (mine.dominatesCrewBackward(theirs)) {
                    theirs.dominated = true;
                    it.remove();
                }
            }
        }
    }




    public ArrayList<Route> getRoutes() {
        return routes;
    }



    private boolean loopToggle;

    private int heuristicIteration;
    private boolean heuristicRedirection;
    private int[] customer2numberOfColumns;
    private int[][] customer2pairInColumns;
    private HashSet<ArrayList<Integer>> heuristicSet;
    public void setHeuristicParameters(int heuristicIteration, boolean heuristicRedirection, int[] customer2numberOfColumns, int[][] customer2pairInColumns, HashSet<ArrayList<Integer>> heuristicSet) {
        this.heuristicIteration = heuristicIteration;
        this.heuristicRedirection = heuristicRedirection;
        this.customer2numberOfColumns = customer2numberOfColumns;
        this.customer2pairInColumns = customer2pairInColumns;
        this.heuristicSet = heuristicSet;
    }

    private class Label {
        int i, q;
        int crewRouteSize;
        double c, t, y, cumT;
        Cet n, w1,w2, m;
        Label p, p2;
        boolean dominated, z;

        double[] r_t;



        public boolean dominates(Label theirs) {

            if(isCrew){
                if(c < theirs.c - 1e-6)
                    return false;
                if(y < theirs.y - 1e-6)
                    return false;

                if(cumT > theirs.cumT + 1e-6)
                    return false;


                if(nonElemC) {
                    if(!heuristic){
                        if(Constants.TWO_CYCLE_ELEMINATION){
                            if(p.i != theirs.p.i)
                                return false;
                        }
                    }
                    return true;
                } else {
                    if(!heuristic){
                        if(!n.isSubset(theirs.n))
                            return false;
                    }
                    return true;
                }
            } else {
                if(c < theirs.c - 1e-6)
                    return false;
                if(y < theirs.y - 1e-6)
                    return false;



                if(!heuristic){

                    if(q > theirs.q)
                        return false;

                    long[] one = n.intersect(cX);
                    long[] two = theirs.n.intersect(cX);
                    for (int l = 0; l < one.length; l++) {
                        if((one[l] & two[l]) != one[l])
                            return false;
                    }
                }
                return true;
            }
        }



        public boolean dominatesCrewForward(Label theirs) {
            {
                if(c < theirs.c - 1e-6)
                    return false;
                if(t > theirs.t + 1e-6)
                    return false;

                if(Constants.ROBUST){
                    for (int k = 0; k < r_t.length; k++)
                        if(r_t[k] > theirs.r_t[k] + 1e-6)
                            return false;
                }

                if(!heuristic) {
                    long[] one = n.intersect(cX);
                    long[] two = theirs.n.intersect(cX);
                    for (int l = 0; l < one.length; l++) {
                        if((one[l] & two[l]) != one[l])
                            return false;
                    }
                }
            }
            return true;
        }

        public boolean dominatesCrewBackward(Label theirs) {
            {
                if(c < theirs.c - 1e-6)
                    return false;
                if(y < theirs.y - 1e-6)
                    return false;
                if(t < theirs.t - 1e-6)
                    return false;

                if(Constants.ROBUST){
                    for (int k = 0; k < r_t.length; k++) {
                        if(r_t[k] < theirs.r_t[k] - 1e-6)
                            return false;
                    }
                }



                if(!heuristic) {
                    long[] one = n.intersect(cX);
                    long[] two = theirs.n.intersect(cX);
                    for (int l = 0; l < one.length; l++) {
                        if((one[l] & two[l]) != one[l])
                            return false;
                    }
                }
            }
            return true;
        }

    }

    /**
     * Returns true when the arc {@code from -> to} violates the active
     * priority-discrimination rule for the current resource type.
     */
    private boolean violatesPriorityExtension(int from, int to) {
        return isCrew
                ? violatesTaskPriority(from, to)
                : violatesZonePriority(from, to);
    }

    /**
     * Crew routes may only move to a task with priority less than or equal to
     * the priority of the current task when crew priority mode is active.
     */
    private boolean violatesTaskPriority(int from, int to) {
        if (!isCrewPriorityMode()) {
            return false;
        }

        int[] priority = data.getTaskPriority();
        if (!hasValidPriorityIndices(priority, from, to)) {
            return false;
        }

        return priority[to] > priority[from];
    }

    /**
     * Tower routes may only move to a zone with priority less than or equal to
     * the priority of the current zone when tower priority mode is active.
     */
    private boolean violatesZonePriority(int from, int to) {
        if (!isTowerPriorityMode()) {
            return false;
        }

        int[] priority = data.getZonePriority();
        if (!hasValidPriorityIndices(priority, from, to)) {
            return false;
        }

        return priority[to] > priority[from];
    }

    private boolean isCrewPriorityMode() {
        return Utility.algo == 41
                || Constants.COST_OF_PRIORITY
                || Constants.COST_OF_PRIORITY_CREW;
    }

    private boolean isTowerPriorityMode() {
        return Utility.algo == 41
                || Constants.COST_OF_PRIORITY
                || Constants.COST_OF_PRIORITY_TOWER;
    }

    private boolean hasValidPriorityIndices(int[] priority, int from, int to) {
        return priority != null
                && from >= 0
                && to >= 0
                && from < priority.length
                && to < priority.length;
    }

}
