package problem.BP;

import lib.StaticSharedValues;
import problem.Constants;
import problem.graph.Data;

import java.util.*;

public class Route {
    private double pseudo_cost;
    private ArrayList<Integer> schedule;
    private ArrayList<Integer> arcIndices;
    private double cost;
    private int waitNode;
    private ArrayList<Boolean> waitBooleans;
    private ArrayList<Double> arrivals;
    private long vI;

    public Route() {}


    public Route(ArrayList<Integer> schedule, ArrayList<Boolean> waitBooleans) {
        create(schedule,waitBooleans);
    }

    public Route(ArrayList<Integer> schedule, ArrayList<Boolean> waitBooleans, ArrayList<Double> serviceLenghts) {
        Data data = Data.getInstance();
        this.schedule = schedule;
        this.waitBooleans = waitBooleans;
        this.arrivals = new ArrayList<>();
        int from = schedule.getFirst();
        double at = 0.;
        int count = 0;
        for (int l = 0; l < schedule.size(); l++) {
            Integer to = schedule.get(l);
            if(to != from){
                at += data.getTowerTravelTimeMatrix()[from][to] + data.getPositionTimeMatrix()[to];
                if(waitBooleans.get(count)){
                    at += serviceLenghts.get(l);
                }
                from = to;
            }
            this.arrivals.add(at);
            count++;
        }
        return;
    }




    public ArrayList<Boolean> getWaitBooleans() {
        return waitBooleans;
    }
    public double getCost() {
        return cost;
    }



    public void setCost(double cost) {
        this.cost = cost;
    }


    public ArrayList<Integer> getPattern() {
        return schedule;
    }

    public void setPattern(ArrayList<Integer> pattern) {
        this.schedule = pattern;
    }




    public void setRepresentation(ArrayList<Integer> arcIndices) {
        this.arcIndices = arcIndices;
    }

    public ArrayList<Integer> getArcIndices() {
        return arcIndices;
    }


    public ArrayList<Double> getArrivals() {
        return arrivals;
    }


    public void create(ArrayList<Integer> schedule, ArrayList<Boolean> waitBooleans) {
        Data data = Data.getInstance();
        double bigM = data.getServiceM();
        if(waitBooleans.isEmpty()){
            this.schedule = schedule;
            double c = 0;
            this.arrivals = new ArrayList<>();
            int from = schedule.getFirst();

            long mI = 0x00L;

            if(Constants.ROBUST){
                double[][] gammas = new double[data.getTasks()+1][StaticSharedValues.budget];
                arrivals.add(0.);
                for (int i = 1; i < schedule.size(); i++) {
                    int to = schedule.get(i);
                    double t1 = data.getCrewTravelTimeMatrix()[from][to] + data.getServiceTimeMatrix()[to];
                    gammas[to][0] = gammas[from][0] + t1;
                    double max = gammas[to][0];
                    for (int k = 1; k < StaticSharedValues.budget; k++) {
                        if(Constants.R_TOGGLE == 0)
                            gammas[to][k] = Math.max(gammas[from][k] + t1,
                                    gammas[from][k-1] + t1 + data.getServiceDeviation()[to]);
                        else
                            gammas[to][k] = Math.max(gammas[from][k] + t1,
                                    gammas[from][k-1] + t1 + data.getCrewTravelTimeDeviations()[from][to]);
                        max = Math.max(max,gammas[to][k]);
                    }
                    arrivals.add(max);
                    mI = mI | (1L << to);
                    from = to;
                }
            } else {
                double at = 0.;
                for(Integer to : schedule){
                    if(to != from){
                        at += data.getCrewTravelTimeMatrix()[from][to] + data.getServiceTimeMatrix()[to];
                        from = to;
                    }
                    mI = mI | (1L << to);
                    arrivals.add(at);
                }
            }


            int depotEnd = data.getDepotEnd(true);
            mI = mI & ~(1L << 0);
            mI = mI & ~(1L << depotEnd);
            this.vI = mI;
            return;
        } else {
            this.schedule = schedule;
            this.waitBooleans = waitBooleans;
            this.arrivals = new ArrayList<>();
            double ciost = 0.;
            int from = schedule.getFirst();
            double at = 0.;
            int count = 0;
            for(Integer to : schedule){
                if(to != from){
                    at += data.getTowerTravelTimeMatrix()[from][to] + data.getPositionTimeMatrix()[to];
                    if(waitBooleans.get(count)){
                        at += bigM;
                        ciost += data.getWeights()[to]*bigM;
                    } else {
                    }
                    from = to;
                }
                this.arrivals.add(at);
                count++;
            }
            this.cost = ciost;
            return;
        }
    }

    public HashMap<Integer,Double> computeCostTower(ArrayList<Integer> schedule, double v) {
        HashMap<Integer,Double> map = new HashMap<>();
        Data data = Data.getInstance();
        double bigM = data.getServiceM();
        int from = schedule.getFirst();
        double at = 0.;
        int count = 1;

        for(Integer to : schedule){
            if(to != from){
                at += data.getTowerTravelTimeMatrix()[from][to] + data.getPositionTimeMatrix()[to];
                if(waitBooleans.get(count++)){
                    at += bigM;
                    map.put(to,bigM * v);
                }
                from = to;
            }
        }

        return map;
    }


    public void setPseudo_cost(double pseudo_cost) {
        this.pseudo_cost = pseudo_cost;
    }

    public double getPseudo_cost() {
        return pseudo_cost;
    }



    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Route)) return false;
        Route other = (Route) obj;
        return Objects.equals(this.schedule, other.schedule);
    }

    @Override
    public int hashCode() {
        return Objects.hash(schedule);
    }


    public long getLong() {
        return vI;
    }


    @Override
    public String toString() {
        return "Route{" +
                schedule +
                "," + waitBooleans +
                '}';
    }
}
