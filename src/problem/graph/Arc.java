package problem.graph;

import lib.Msg;

import java.util.ArrayList;
import java.util.Objects;

public class Arc  {

    private int id;
    private int from, to;
    private double distance, time;
//    private double vehicle_red_cost, crew_red_cost;
    private boolean absoluteDominance;
    private ArrayList<Integer> path;
    private boolean isDamaged;

    public Arc(int i, int j, double distance, double time, ArrayList<Integer> path){
        this.from = i;
        this.to = j;
        this.distance = distance;
        this.time = time;
//        this.path = new ArrayList<>(path);
        this.path = path; //TODO
    }

    public Arc(int i, int j){
        this.from = i;
        this.to = j;
    }


    public int from() {
        return from;
    }

    public int to() {
        return to;
    }

    public double distance() {
        return distance;
    }

    public double time() {
        return time;
    }


    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    @Override
    public String toString() {
        return "("+from+","+to+")";
    }

    public String toStringTime(){
        return from + "\t"+to+"\t"+time;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Arc arc = (Arc) o;
        return from == arc.from && to == arc.to && Double.compare(distance, arc.distance) == 0 && Double.compare(time, arc.time) == 0 && path.equals(arc.path);
    }

    @Override
    public int hashCode() {
        return Objects.hash(from, to, distance, time);
    }

//    public void setDual(double v, boolean isCrew){
//        if(isCrew)
//            crew_red_cost = v;//+distance
//        else
//            vehicle_red_cost = v;//+distance
//    }
//
//    public double getDualV(){
//        return vehicle_red_cost;
//    }
//
//    public double getDualC(){
//        return crew_red_cost;
//    }

    public ArrayList<Integer> getPath() {
        if(path == null){
            Msg.stop("null path");
        }
        return path;
    }

    public void setPath(ArrayList<Integer> path) {
        this.path = path;
    }


    public void timeAndDistancePathCoincide() {
        absoluteDominance = true;
    }

    public boolean isAbsoluteDominance() {
        return absoluteDominance;
    }

    public void setDamaged(boolean damaged) {
        isDamaged = damaged;
    }

    public boolean isDamaged() {
        return isDamaged;
    }

    public void setArc(int i, int j) {
        from = i;
        to = j;
    }

    public void setTime(double score) {
        time = score;
    }

    private String scoreString;
    public void setScoreString(String scoreString) {
        this.scoreString = scoreString;
    }


    public String getScoreString() {
        return scoreString;
    }

    private  double minChange;
    public void setMinimumChange(double min) {
        minChange = min;
    }

    public double getMinimumChange() {
        return minChange;
    }
}
