package problem.BP;

import lib.*;
import problem.Constants;
import problem.graph.Arc;
import problem.graph.Data;

import java.util.ArrayList;

public class BNode implements Comparable<BNode>{

    public enum BranchDirection { UP, DOWN }

    private int fleet_lb;
    private int fleet_ub;
    private int crewLB;
    private int crewUB;
    private double cost;
    private double pseudoCost;
    private double costWhenPruned;
    private int depth;
    private boolean hasFeasibleSol;
    private int[][][] feasibleSchedule_arc; //-1/+1 for relief vehicles, -2/+2 for crews

    private ArrayList<Route> relief;
    private ArrayList<Route> crew;
    private String branch;
    /** Canonical root-to-node branch path used only as a deterministic queue tie-breaker. */
    private String deterministicPath = "";
    private double parentLpBound;
    private int branchedVariableId = -1;
    private BranchDirection branchDirection;
    private double branchDelta;
    private boolean pseudocostUpdateDone;
    private boolean positive;

    private int numberBanned;
    private ArrayList<String> solutions;

    private double[] packingLB;
    private double[] packingUB;
    private double[][] aBounds;
    private boolean[][] violatedArc;
    private BNode father;




    public BNode() {

        fixedCrewRoutes = new ArrayList<>();


    }

    public BNode copy() {
        BNode dst = new BNode();
        dst.iceaID = iceaID;

        dst.fleet_lb = fleet_lb;
        dst.fleet_ub = fleet_ub;
        dst.crewLB = crewLB;
        dst.crewUB = crewUB;
        dst.cost = cost;
        dst.deterministicPath = deterministicPath;
        dst.depth = depth + 1;
        dst.ftr = ftr; //shallow copy
        dst.non_improvement_counter = non_improvement_counter;


        if(BendersCuts.toggle){
            dst.fixedCrewRoutes = new ArrayList<>();
            for(Route r1 : fixedCrewRoutes){
                Route r2 = new Route();
                r2.create(r1.getPattern(),new ArrayList<>());
                dst.fixedCrewRoutes.add(r2);
            }
        }



        if(totsBounds != null){
            dst.totsBounds = new int[2];
            for (int i = 0; i < 2; i++) {
                dst.totsBounds[i] = totsBounds[i];
            }
        }
        if(solutions != null)
            dst.solutions = new ArrayList<>(solutions);





        

        dst.packingLB = new double[packingLB.length];
        dst.packingUB = new double[packingUB.length];

        for (int i = 0; i < packingLB.length; i++) {
            dst.packingLB[i] = packingLB[i];
            dst.packingUB[i] = packingUB[i];
            

        }


        dst.aBounds = new double[aBounds.length][2];
        for (int i = 0; i < aBounds.length; i++) {
            for (int j = 0; j < 2; j++) {
                dst.aBounds[i][j] = aBounds[i][j];
            }
        }
        dst.violatedArc = new boolean[packingLB.length][packingLB.length];
        for (int i = 0; i < violatedArc.length; i++) {
            for (int j = 0; j < violatedArc.length; j++) {
                dst.violatedArc[i][j] = violatedArc[i][j];
            }
        }
        Data data = Data.getInstance();
        dst.feasibleSchedule_arc = new int[2][][];
        for (int k = 0; k < 2; k++) {
            if(k==0){
                dst.feasibleSchedule_arc[k] = new int[data.getNodeNumber()][data.getNodeNumber()+1];
                for (int i = 0; i < data.getNodeNumber(); i++) {
                    for (int j = 0; j < data.getNodeNumber()+1; j++) {
                        dst.feasibleSchedule_arc[k][i][j] = feasibleSchedule_arc[k][i][j];
                    }
                }
            } else {
                dst.feasibleSchedule_arc[k] = new int[data.getTasks()][data.getTasks()+1];
                for (int i = 0; i < data.getTasks(); i++) {
                    for (int j = 0; j < data.getTasks()+1; j++) {
                        dst.feasibleSchedule_arc[k][i][j] = feasibleSchedule_arc[k][i][j];
                    }
                }
            }
        }










        dst.numberBanned = numberBanned;

        dst.father = this;

        if(cx != null)
            dst.cx = cx.copy();




        return dst;
    }
    @Override
    public int compareTo(BNode other){
        if(Constants.DEPTH_FIRST_SEARCH){
            //experiments
            if(this.depth == other.depth){
                int costComparison = Double.compare(this.cost,other.cost);
                if (costComparison != 0) return costComparison;
                return deterministicPath.compareTo(other.deterministicPath);
            }
            return Integer.compare(other.depth,this.depth);
        }
        if(Double.compare(this.cost,other.cost) == 0){
            if (positive && !other.positive) return -1;   // this before other
            if (!positive && other.positive) return 1;    // this after other
            return deterministicPath.compareTo(other.deterministicPath);
        }
        return Double.compare(this.cost,other.cost);
    }

    public double getCost() {
        return cost;
    }

    public void setPseudocostBranchMetadata(
            double parentLpBound,
            int branchedVariableId,
            BranchDirection branchDirection,
            double branchDelta
    ) {
        this.parentLpBound = parentLpBound;
        this.branchedVariableId = branchedVariableId;
        this.branchDirection = branchDirection;
        this.branchDelta = branchDelta;
        this.pseudocostUpdateDone = false;
    }

    public boolean hasPseudocostBranchMetadata() {
        return branchedVariableId >= 0 && branchDirection != null && branchDelta > 0.0;
    }

    public double getParentLpBound() { return parentLpBound; }
    public int getBranchedVariableId() { return branchedVariableId; }
    public BranchDirection getBranchDirection() { return branchDirection; }
    public double getBranchDelta() { return branchDelta; }
    public boolean isPseudocostUpdateDone() { return pseudocostUpdateDone; }
    public void markPseudocostUpdateDone() { pseudocostUpdateDone = true; }

    public void setCost(double cost) {
        this.cost = cost;
    }

    public int getDepth() {
        return depth;
    }

    public void setDepth(int depth) {
        this.depth = depth;
    }


    public int getFleetLB() {
        return fleet_lb;
    }

    public void setFleetLB(int fleet_lb) {
        this.fleet_lb = fleet_lb;
    }

    public int getFleetUB() {
        return fleet_ub;
    }

    public void setFleetUB(int fleet_ub) {
        this.fleet_ub = fleet_ub;
    }

    public int getCrewLB() {
        return crewLB;
    }

    public void setCrewLB(int crewLB) {
        this.crewLB = crewLB;
    }

    public int getCrewUB() {
        return crewUB;
    }

    public void setCrewUB(int crewUB) {
        this.crewUB = crewUB;
    }

    public boolean hasInitialSolution() {
        return hasFeasibleSol;
    }

    public int[][][] getFeasibleSchedule_arc() {
        return feasibleSchedule_arc;
    }

    public void setFeasibleSchedule_arc(int[][][] feasibleSchedule_arc) {
        this.feasibleSchedule_arc = feasibleSchedule_arc;
    }

    public void setSetBounds(int[][] setBounds) {
        this.setBounds = setBounds;
    }

    public void setTotsBounds(int[] totsBounds) {
        this.totsBounds = totsBounds;
    }


    public void setInitialSolution(ArrayList<Route> relief,  ArrayList<Route> crew) {
        if(!relief.isEmpty() && !crew.isEmpty())
            hasFeasibleSol = true;

        this.relief = relief;
        this.crew = crew;

        for(Route route : relief){
            ArrayList<Integer> shcedule = route.getPattern();
            int from =shcedule.getFirst();
            for(Integer to : shcedule){
                if(from != to){
                    if(feasibleSchedule_arc[0][from][to] == -1)
                        Msg.infeasibleArc();
                    if(aBounds[to][1] == 0)
                        throw new IllegalArgumentException("infeasible customer visit");
                    from = to;
                }
            }
        }

        for(Route route : crew){
            ArrayList<Integer> shcedule = route.getPattern();
            int from =shcedule.getFirst();
            for(Integer to : shcedule){
                if(from != to){
                    if(feasibleSchedule_arc[1][from][to] == -1)
                        Msg.infeasibleArc();
                }
            }
        }
    }

    public void setInitialSolution(LP model,ArrayList<Route> relief,  ArrayList<Route> crew) {
        if(!relief.isEmpty() && !crew.isEmpty())
            hasFeasibleSol = true;

        this.relief = relief;
        this.crew = crew;

        for(Route route : relief){
            ArrayList<Integer> shcedule = route.getPattern();
            int from =shcedule.getFirst();
            for(Integer to : shcedule){
                if(from != to){
                    if(feasibleSchedule_arc[0][from][to] == -1)
                        Msg.infeasibleArc(); //|| superBranchingArc[from][to] == -1
                    if(aBounds[to][1] == 0)
                        throw new IllegalArgumentException("infeasible customer visit");
                    from = to;
                }
            }
        }

        for(Route route : crew){
            ArrayList<Integer> shcedule = route.getPattern();
            int from =shcedule.getFirst();
            for(Integer to : shcedule){
                if(from != to){
                    if(feasibleSchedule_arc[1][from][to] == -1)
                        Msg.infeasibleArc();
                    from = to;
                }
            }
        }
    }




    public ArrayList<Route> getCrewRoutes() {
        return crew;
    }

    public ArrayList<Route> getReliefRoutes() {
        return relief;
    }

    public int getOuterArcValue(int i, int j, boolean isCrew) {
        if(isCrew)
            return feasibleSchedule_arc[1][i][j];
        return feasibleSchedule_arc[0][i][j];
    }



    public void setSetBound(int i, int lb, int ub){
        setBounds[i][0] = lb;
        setBounds[i][1] = ub;
    }

    public void setTotBound(int lb, int ub){
        totsBounds[0] = lb;
        totsBounds[1] = ub;
    }

    public void setFeasibleArcValue(Arc arc, int lb, int ub, boolean isCrew) {
        Data data = Data.getInstance();
        if(isCrew){
            if(ub < 1e-6)
                feasibleSchedule_arc[1][arc.from()][arc.to()] = -2;
            else if(lb > 1e-6){
                feasibleSchedule_arc[1][arc.from()][arc.to()] = 2;
                if(arc.to() != data.getDepotEnd(true)){
                    for (int i = 0; i < data.getTasks(); i++) {
                        if(i != arc.from()){
                            feasibleSchedule_arc[1][i][arc.to()] = -2;
                        }
                    }
                }

                if(arc.from() != 0){
                    for (int j = 0; j < data.getTasks()+1; j++) {
                        if(j != arc.to()){
                            feasibleSchedule_arc[1][arc.from()][j] = -2;
                        }
                    }
                }
                feasibleSchedule_arc[1][arc.from()][arc.to()] = 2;
            }
        } else {
            if(ub < 1e-6)
                feasibleSchedule_arc[0][arc.from()][arc.to()] = -1;
            else if(lb > 1e-6){
                feasibleSchedule_arc[0][arc.from()][arc.to()] = 1;
                if(arc.to() != data.getDepotEnd(false)) {
                    for (int i = 0; i < data.getNodeNumber(); i++) {
                        if(i != arc.from()){
                            feasibleSchedule_arc[0][i][arc.to()] = -1;
                        }
                    }
                }

                if(arc.from() != 0) {
                    for (int j = 0; j < data.getNodeNumber()+1; j++) {
                        if(j != arc.to()){
                            feasibleSchedule_arc[0][arc.from()][j] = -1;
                        }
                    }
                }
            }
        }
    }

    public void setFeasibleConsecutiveArcValue(int a1, int a2, int a3, int case1) {
        Data data = Data.getInstance();
        switch (case1){
            case 1 -> {
                feasibleSchedule_arc[1][a1][a2] = 2;
                feasibleSchedule_arc[1][a2][a3] = 2;
                {
                    int from = a1, to = a2;
                    if (from != 0) {
                        for (int j = 0; j < data.getTasks()+1; j++) {
                            if (j != to) {
                                feasibleSchedule_arc[1][from][j] = -2;
                            }
                        }
                    }
                    if (a3 != data.getDepotEnd(true)) {
                        for (int i = 0; i < data.getTasks(); i++) {
                            if (i != from) {
                                feasibleSchedule_arc[1][i][to] = -2;
                            }
                        }
                    }
                }
                {
                    int from = a2, to = a3;
                    if (from != 0) {
                        for (int j = 0; j < data.getTasks()+1; j++) {
                            if (j != to) {
                                feasibleSchedule_arc[1][from][j] = -2;
                            }
                        }
                    }
                    if (a3 != data.getDepotEnd(true)) {
                        for (int i = 0; i < data.getTasks(); i++) {
                            if (i != from) {
                                feasibleSchedule_arc[1][i][to] = -2;
                            }
                        }
                    }
                }
            }
            case 2 -> {
                feasibleSchedule_arc[1][a1][a2] = 2;
                feasibleSchedule_arc[1][a2][a3] = - 2;
                {
                    int from = a1, to = a2;
                    if (from != 0) {
                        for (int j = 0; j < data.getTasks()+1; j++) {
                            if (j != to) {
                                feasibleSchedule_arc[1][from][j] = -2;
                            }
                        }
                    }
                    if (a3 != data.getDepotEnd(true)) {
                        for (int i = 0; i < data.getTasks(); i++) {
                            if (i != from) {
                                feasibleSchedule_arc[1][i][to] = -2;
                            }
                        }
                    }
                }
                {
                    int from = a2, to = a3;
                    feasibleSchedule_arc[1][from][to] = -2;
                }
            }
            case 3 -> {
                feasibleSchedule_arc[1][a1][a2] = - 2;
                feasibleSchedule_arc[1][a2][a3] = 2;
                {
                    int from = a1, to = a2;
                    feasibleSchedule_arc[1][from][to] = -2;
                }
                {
                    int from = a2, to = a3;
                    if (from != 0) {
                        for (int j = 0; j < data.getTasks()+1; j++) {
                            if (j != to) {
                                feasibleSchedule_arc[1][from][j] = -2;
                            }
                        }
                    }
                    if (a3 != data.getDepotEnd(true)) {
                        for (int i = 0; i < data.getTasks(); i++) {
                            if (i != from) {
                                feasibleSchedule_arc[1][i][to] = -2;
                            }
                        }
                    }
                }
            }
            case 4 -> {
                feasibleSchedule_arc[1][a1][a2] = - 2;
                feasibleSchedule_arc[1][a2][a3] = - 2;
                {
                    int from = a1, to = a2;
                    feasibleSchedule_arc[1][from][to] = -2;
                }
                {
                    int from = a2, to = a3;
                    feasibleSchedule_arc[1][from][to] = -2;
                }
            }
        }
    }
    public void print() {
        String s = "Node:"+depth+","+cost   +";"+branch;
        System.out.println(s);
    }

    public void print(int tots) {
        String s = "Node:"+depth + "/"+tots  +","+cost   +";"+branch;
        System.out.println(s);
    }
    public void print(int tots, double best_cost) {
        String s = "Node:"+depth + "/"+tots  +","+HF.truncate(cost,1000)   +";"+branch+".UB:"+best_cost;
        System.out.println(s);
    }

    public void print(int tots, double best_cost, int uniqueID, double bestUB, int bestID) {
        String s = "Node(T"+uniqueID +"):"+depth + "/"+tots  +","+HF.truncate(cost,1000)   +";"+branch
                +".This UB:"+best_cost+". GBL UB(T"+bestID+"):"+bestUB;
        System.out.println(s);
    }
    public void print(int tots, double best_cost, int uniqueID, int workerID, double bestUB, int bestID) {
        String s = "Node(T"+uniqueID +",W"+workerID +"):"+depth + "/"+tots  +","+HF.truncate(cost,1000)   +";"+branch
                +".This UB:"+best_cost
                +". GBL UB(T"+bestID+"):"+bestUB;
        System.out.println(s);
    }


    public void setBranchStrig(String string) {
        branch = "Br("+ depth+"):" + string;
        deterministicPath = deterministicPath + "/" + branch;
    }
    public String getBranchStrig() {
        return branch;
    }

    public void setPositive(boolean positive) {
        this.positive = positive;
    }



    public void updateSolutions(String s){
        if(s == null)
            throw new IllegalArgumentException("empty solution");
        if(solutions == null)
            solutions = new ArrayList<>();
        solutions.add(s);
    }



    public double getPackingLB(int i) {
        return packingLB[i];
    }

    public double getPackingUB(int i){
        return packingUB[i];
    }




    
    public void initializePackingBounds(int len){
        packingLB = new double[len];
        packingUB = new double[len];
        aBounds = new double[len][2];
        for (int i = 1; i < len; i++) {
            packingUB[i] = 1;
            aBounds[i][0] = 0;
            aBounds[i][1] = 1;
        }
        packingUB[0] = Data.getInstance().getTowerNumber();
        packingUB[Data.getInstance().getDepotEnd(false)] = packingUB[0];
        violatedArc = new boolean[len][len];



        totsBounds = new int[]{0, Data.getInstance().getNodeNumber()};
    }

    public void setPackingBounds(int i, double lb, double ub) {
        packingLB[i] = lb;
        packingUB[i] = ub;
    }



    public BNode getFather() {
        return father;
    }

    public void setABounds(int i, int lb, int ub) {
        aBounds[i][0] = lb;
        aBounds[i][1] = ub;
    }





    public double[] getABounds(int i) {
        return aBounds[i];
    }

    public void printBranchStrings() {
        BNode father = this;
        while (father != null){
            System.out.println(father.getBranchStrig());
            father = father.getFather();
        }
    }

    public void printBranchStringsWithCosts() {
        BNode father = this;
        while (father != null){
            System.out.println(father.getBranchStrig()+";("+father.getCost()+")");
            father = father.getFather();
        }
    }





    public double getCostWhenPruned() {
        return costWhenPruned;
    }

    public void setCostWhenPruned(double costWhenPruned) {
        this.costWhenPruned = costWhenPruned;
    }

    private int[][] setBounds;
    public int[] getSetBounds(int i) {
        return setBounds[i];
    }

    private int[] totsBounds;
    public int[] getTotsBounds() {
        return totsBounds;
    }

    public ArrayList<String> getSolutionStrings() {
        return solutions;
    }

    public void addViolatedArc(int i, int j) {
        violatedArc[i][j] = true;
    }

    public boolean isViolatedArc(int i, int j) {
        return violatedArc[i][j];
    }





    public void addOneMoreBannedVisit() {
        numberBanned++;
    }

    public int getNumberBanned() {
        return numberBanned;
    }



    private Cet cx;
    public Cet getCx() {
        return cx;
    }
    public void createCx(){
        cx = new Cet(Data.getInstance().getNodeNumber());
    }

    public void updateCx(Cet cX) {
        this.cx.union(cX);
    }


    private ArrayList<ArrayList<Integer>> ftr;


    public void setInitialCrewSolution(ArrayList<Route> initialCrew) {
        this.crew = initialCrew;
    }








    public boolean isPositive() {
        return positive;
    }




    private int non_improvement_counter;
    public void increaseConsecutiveNonImprovement() {
        this.non_improvement_counter = non_improvement_counter + 1;
    }

    public void restartConsecutiveNonImprovement() {
        non_improvement_counter = 0;
    }

    public int getConsecutiveNonImprovement() {
        return non_improvement_counter;
    }



    private ArrayList<Route> fixedCrewRoutes;
    public void setFixedCrewRoute(Route e) {
        fixedCrewRoutes.add(e);
    }

    public ArrayList<Route> getFixedCrewRoutes() {
        return fixedCrewRoutes;
    }




    private int iceaID;
    public void setICEAid(int id) {
        iceaID = id;
    }

    public int getIceaID() {
        return iceaID;
    }


}
