package problem.BP;

import ilog.concert.IloNumVar;

public class VarR {
    private IloNumVar theta;
    private Route route;
    private double objective_coefficient;
    private double value;

    public double getCost() {
        return objective_coefficient;
    }

    public void setCost(double objective_coefficient) {
        this.objective_coefficient = objective_coefficient;
    }

    public Route getRoute() {
        return route;
    }

    public void setRoute(Route route) {
        this.route = route;
    }

    private Route crew;
    public void setRoutes(Route tower, Route crew){
        this.route = tower;
        this.crew = crew;
    }

    public Route getCrew() {
        return crew;
    }

    public IloNumVar getNumVar() {
        return theta;
    }

    public void setTheta(IloNumVar theta) {
        this.theta = theta;
    }

    public double getValue() {
        return value;
    }

    public void setValue(double value) {
        this.value = value;
    }
}
