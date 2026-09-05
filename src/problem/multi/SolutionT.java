package problem.multi;

import problem.BP.Route;
import problem.graph.Data;

import java.util.ArrayList;
import java.util.HashSet;

public class SolutionT {

    private final ArrayList<Route> best_crew;
    private final ArrayList<Route> best_towerRoutes;
    private final ArrayList<Double> route_values;
    private final ArrayList<ArrayList<Boolean>> route_waits;
    private final double cost;
    private final HashSet<ArrayList<Integer>> towerRoutesHashSet;

    public SolutionT(double v1, ArrayList<Route> best_crew, ArrayList<Route> best_towerRoutes, ArrayList<Double> route_values, ArrayList<ArrayList<Boolean>> route_waits) {
        this.best_crew = best_crew;
        this.best_towerRoutes = best_towerRoutes;
        this.route_values = route_values;
        this.route_waits = route_waits;
        this.cost = v1;

        HashSet<ArrayList<Integer>> towerRoutesHashSet = new HashSet<>();
        if(best_towerRoutes!=null){
            for (int i = 0; i < best_towerRoutes.size(); i++) {
                Route route = best_towerRoutes.get(i);
                towerRoutesHashSet.add(route.getPattern());
                if (!route_waits.get(i).equals(route.getWaitBooleans()))
                    throw new IllegalArgumentException("wait patternes do not match");
            }
        }

        this.towerRoutesHashSet = towerRoutesHashSet;
    }

    public int getNumZonesAssignedToTowers() {
        HashSet<Integer> unique = new HashSet<>();
        for(Route r : best_towerRoutes){
            ArrayList<Integer> route = r.getPattern();
            for (int k = 1; k < route.size()-1; k++) {
                unique.add(route.get(k));
            }
        }
        return unique.size();
    }

    public HashSet<ArrayList<Integer>> getTowerRoutesHashSet() {
        return towerRoutesHashSet;
    }

    public ArrayList<Route> getBest_crew() {
        return best_crew;
    }

    public ArrayList<Route> getBest_towerRoutes() {
        return best_towerRoutes;
    }

    public ArrayList<Double> getRoute_values() {
        return route_values;
    }

    public ArrayList<ArrayList<Boolean>> getRoute_waits() {
        return route_waits;
    }

    public void print() {
        Data data = Data.getInstance();
        HashSet<ArrayList<Integer>> towerRoutesHashSet = new HashSet<>();
        for (int i = 0; i < best_towerRoutes.size(); i++) {
            Route route = best_towerRoutes.get(i);
            double v = route_values.get(i);
            System.out.print("\t" + v + ":");
            System.out.print(route.getPattern().toString());
            System.out.println(route_waits.get(i));

            towerRoutesHashSet.add(route.getPattern());
            if (!route_waits.get(i).equals(route.getWaitBooleans()))
                throw new IllegalArgumentException("wait patternes do not match");
        }

        System.out.println("Optimal Tower Routes:");
        for(ArrayList<Integer> list : towerRoutesHashSet)
            System.out.println("\t"+list.toString());


        System.out.println("Optimal Repair Crews:");
//        for(Route route : best_crew)
//            System.out.println("\t"+route.getPattern().toString() + " | " + " Total travel time: " + route.getArrivals().getLast() + "/" + data.getHorizon());
        for(Route route : best_crew)
            System.out.println(route.getPattern().toString());

        System.out.print("Optimal Repair Schedule:[");
        double[] restoration = new double[data.getNodeNumber()];
        double[] taksCompletion = new double[data.getTasks()];
        for(Route route : best_crew){
            for (int j = 1; j < route.getPattern().size() - 1; j++) {
                int key = route.getPattern().get(j);
                taksCompletion[key] += route.getArrivals().get(j);
            }
        }
        for (int i = 1; i < data.getNodeNumber(); i++) {
            double max = 0;
            for(Integer k : data.getZone2tasks()[i])
                max = Math.max(taksCompletion[k],max);
            restoration[i] = max;
        }
        boolean[] tabu = new boolean[data.getNodeNumber()];
        for (int k = 1; k < data.getNodeNumber(); k++) {
            if(k!=1)
                System.out.print(", ");
            double earliest = Double.MAX_VALUE;
            int id = -1;
            for (int i = 1; i < data.getNodeNumber(); i++) {
                if(tabu[i])
                    continue;
                if(restoration[i] < earliest){
                    earliest = restoration[i];
                    id = i;
                }
            }
            tabu[id] = true;
            System.out.print(id);
        }
        System.out.println("]");
        System.out.println("Final Cost:"+cost);
    }

    public double getCost() {
        return cost;
    }

    public boolean hasSolution() {
        return best_crew != null
                && best_towerRoutes != null
                && route_values != null
                && route_waits != null
                && (!best_crew.isEmpty() || !best_towerRoutes.isEmpty());
    }
}
