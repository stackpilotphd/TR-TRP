package problem.BP;

import java.util.HashMap;
import java.util.Map;

/** Solver-local pseudocost observations indexed by integer-variable id. */
public final class PseudocostTable {
    public static final class Entry {
        private double upSum;
        private double downSum;
        private long upCount;
        private long downCount;

        public double upAverage() { return upCount == 0 ? 1.0 : upSum / upCount; }
        public double downAverage() { return downCount == 0 ? 1.0 : downSum / downCount; }

        private void observe(BNode.BranchDirection direction, double value) {
            if (direction == BNode.BranchDirection.UP) {
                upSum += value;
                upCount++;
            } else {
                downSum += value;
                downCount++;
            }
        }
    }

    private final Map<Integer, Entry> entries = new HashMap<>();

    public Entry get(int variableId) {
        return entries.computeIfAbsent(variableId, ignored -> new Entry());
    }

    public void observe(int variableId, BNode.BranchDirection direction, double value) {
        get(variableId).observe(direction, value);
    }
}
