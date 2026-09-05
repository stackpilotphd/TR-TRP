package lib;

public class Tuple {
    public int id, k;
    public double v;

    public Tuple(int id, double val) {
        this.id = id;
        this.v = val;
    }

    public Tuple(int id, int k, double val) {
        this.id = id;
        this.k = k;
        this.v = val;
    }



    @Override
    public String toString() {
        return "Tuple{" +
                 + id +
                "," + k +"," + v +
                '}';
    }
}
