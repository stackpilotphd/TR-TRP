package problem.BP;

import ilog.concert.IloException;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

public class Constraint {

    private double lb;
    private double ub;
    private String name;
    private HashMap<Integer, IloRange> ranges;
    private HashMap<Integer, Integer> nodeID2constraint;
    private HashMap<Integer, HashMap<Integer,Integer>> ij2constraint;
    private HashMap<Integer,ArrayList<Integer>> nodeID2wholeRouteConstraintIDList;
    private HashMap<ArrayList<Integer>, ArrayList<IloRange>> rep2constraint;
    private HashMap<Integer,ArrayList<IloRange>> nodeID2ranges;
    private HashMap<Integer,HashMap<Integer,Integer>> demand2repair2constraint;

    public Constraint(double lb, double ub, String name) {
        this.lb = lb;
        this.ub = ub;
        this.name = name;
        ranges = new HashMap<>();
        nodeID2constraint = new HashMap<>();
        ij2constraint = new HashMap<>();
        nodeID2wholeRouteConstraintIDList = new HashMap<>();
        rep2constraint = new HashMap<>();
        nodeID2ranges = new HashMap<>();
        demand2repair2constraint = new HashMap<>();
    }


    public int initialize(IloCplex cplex) throws IloException {
        int ret = ranges.size();
        ranges.put(ret,cplex.addRange(lb,ub,name+ret));
        return  ret;
    }

    public int initialize(IloCplex cplex, Integer i, Integer j) {
        int ret = ranges.size();
        try {
            ranges.put(ret,cplex.addRange(lb,ub,name+i+"_"+j));
            ij2constraint.computeIfAbsent(i,k->new HashMap<>()).put(j,ret);
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
        return  ret;
    }

    public int initialize(IloCplex cplex, Integer nodeID) {
        int ret = ranges.size();
        try {
            ranges.put(ret,cplex.addRange(lb,ub,name+nodeID));
            nodeID2constraint.put(nodeID,ret);
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
        return  ret;
    }

    public int initialize(IloCplex cplex, Integer nodeID, ArrayList<Integer> rep) {
        int ret = ranges.size();
        try {
            ranges.put(ret,cplex.addRange(lb,ub,name+nodeID+","+rep));
            nodeID2wholeRouteConstraintIDList.computeIfAbsent(nodeID, k -> new ArrayList<>());
            nodeID2wholeRouteConstraintIDList.get(nodeID).add(ret);
            nodeID2ranges.computeIfAbsent(nodeID,k->new ArrayList<>());
            nodeID2ranges.get(nodeID).add(ranges.get(ret));
            nodeID2constraint.put(nodeID,ret);
            rep2constraint.computeIfAbsent(rep, k -> new ArrayList<>());
            rep2constraint.get(rep).add(ranges.get(ret));
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
        return  ret;
    }

    public Collection<IloRange> getRanges() {
        return ranges.values();
    }

    public IloRange getRangeFromRID(Integer ID){
        return ranges.get(ID);
    }
    public IloRange getRange(Integer nodeID){
        return ranges.get(
                nodeID2constraint.get(nodeID));
    }

    public Collection<IloRange> getRanges(Integer nodeID) {
        return nodeID2ranges.get(nodeID);
    }



    public Collection<IloRange> getRanges(ArrayList<Integer> rep) {
        return rep2constraint.get(rep);
    }

    public IloRange getRange(Integer i, Integer j){
        if(ij2constraint.get(i) == null)
            return null;
        if(ij2constraint.get(i).get(j) == null)
            return null;
        return ranges.get(ij2constraint.get(i).get(j));
    }
}
