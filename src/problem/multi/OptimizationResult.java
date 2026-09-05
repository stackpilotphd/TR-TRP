package problem.multi;

public class OptimizationResult {
    private final double bestValue;
    private final SolutionT bestSolution;
    private final int nodesExplored;
    private final double timeMillis;
    private final int id;
    private final double lbTerm;
    private final boolean hasTimeOut;
    private final double rootLB;


    public OptimizationResult(double rootLB, double bestValue, SolutionT bestSolution,
                              int nodesExplored, double timeMillis, int i1, double lbTermination, boolean isTimeOut) {
        this.rootLB = rootLB;
        this.bestValue = bestValue;
        this.bestSolution = bestSolution;
        this.nodesExplored = nodesExplored;
        this.timeMillis = timeMillis;
        this.id = i1;
        this.lbTerm = lbTermination;
        this.hasTimeOut = isTimeOut;
    }

    public double getBestValue() { return bestValue; }
    public SolutionT getBestSolution() { return bestSolution; }
    public int getNodesExplored() { return nodesExplored; }
    public double getTime() { return timeMillis; }
    public boolean isTimeOut() {
        return hasTimeOut;
    }

    @Override
    public String toString() {
        String header = "T";
        return header+id+ ": Value=" + bestValue +
                ", LB(termination)=" + lbTerm +
                ", Nodes=" + nodesExplored +
                ", Time=" + timeMillis + "s";
    }

    public int getID() {
        return id;
    }

    public double getLBtermination() {
        return lbTerm;
    }

    public double getRootLB() {
        return rootLB;
    }
}
