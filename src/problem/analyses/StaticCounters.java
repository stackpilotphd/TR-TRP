package problem.analyses;

public class StaticCounters {
    public static int[] numberOfTasksAtPosition;

    public static int[] priorityCounterAtPosition;
    public static double[] totalWeightAtPosition;
    public static double[] totalServiceAtPosition;
    public static boolean isInitialized;

    public static boolean hasSharedTask;
    public static int[] numberOfTasksAtPositionInSharedInstances;
    public static int[] numberSharedTasksAtPosition;
}
