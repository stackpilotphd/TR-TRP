package problem.BP;

/** Tracks monotone best-open-node bounds over processed branch-and-bound nodes. */
public final class LowerBoundProgressTracker {
    private static final int HISTORY_CAPACITY = 101;

    private final double[] lowerBoundHistory = new double[HISTORY_CAPACITY];
    private final int stallWindow;
    private final double stallDelta;
    private long iterationCount;
    private int historySize;
    private int nextRecord;
    private double lastRecordedGlobalLowerBound = Double.NEGATIVE_INFINITY;

    public LowerBoundProgressTracker(int stallWindow, double stallDelta) {
        if (stallWindow != 100) {
            throw new IllegalArgumentException("The 101-record buffer requires stallWindow=100");
        }
        this.stallWindow = stallWindow;
        this.stallDelta = stallDelta;
    }

    /** Records a bound and returns true when the configured window has stalled. */
    public boolean record(double bestOpenNodeBound) {
        double monotoneBound = Math.max(bestOpenNodeBound, lastRecordedGlobalLowerBound);
        double boundFromWindowAgo = Double.NaN;
        if (historySize >= stallWindow) {
            int oldest = historySize < HISTORY_CAPACITY ? 0 : nextRecord;
            int target = (oldest + historySize - stallWindow) % HISTORY_CAPACITY;
            boundFromWindowAgo = lowerBoundHistory[target];
        }

        lowerBoundHistory[nextRecord] = monotoneBound;
        nextRecord = (nextRecord + 1) % HISTORY_CAPACITY;
        if (historySize < HISTORY_CAPACITY) historySize++;

        iterationCount++;
        lastRecordedGlobalLowerBound = monotoneBound;

        return historySize == HISTORY_CAPACITY
                && monotoneBound - boundFromWindowAgo < stallDelta;
    }

    public long getIterationCount() { return iterationCount; }
    public int getStallWindow() { return stallWindow; }
    public double getStallDelta() { return stallDelta; }
    public double getLastRecordedGlobalLowerBound() { return lastRecordedGlobalLowerBound; }
}
