package problem.multi;

public class ICEAconstraint {
    public final double lb, ub;
    public final int id;


    public ICEAconstraint(double lb, double ub, int _id) {
        this.lb = lb;
        this.ub = ub;
        id = _id;
    }




    @Override
    public String toString() {
        return "ICEAconstraint{" +
                "id=" + id +
                ", lb=" + lb +
                ", ub=" + ub +
                '}';
    }
}
