package problem.multi;

public class BestBound {
    public final double value;
    public final int threadId;

    public BestBound(double value,int threadId) {
        this.threadId = threadId;
        this.value = value;
    }
}
