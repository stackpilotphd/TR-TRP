package problem.BP;

import java.util.ArrayList;

public class EarlyTerminationICEA {
    public static double UB;
    public static boolean isFirstIteration;
    public static boolean hasTerminatedEarly;
    public static ArrayList<BNode> suspendedNodes;
    public static int limLB, limUB;
    public static boolean isRerun;
    public static final double gap = 9.001;
    public static double bestBound;
    public static boolean stopCheckling;
    public static ArrayList<Route> best_crew;
    public static ArrayList<Double> route_values;
    public static ArrayList<ArrayList<Boolean>> route_waits;
    public static ArrayList<Route> best_route;

    public static void initalize(){
        isFirstIteration = false;
        hasTerminatedEarly = false;
        suspendedNodes = new ArrayList<>();
        limLB = 0;
        limUB = 0;
        isRerun = false;
        bestBound = 999;
        stopCheckling = false;
        best_crew = new ArrayList<>();
        route_values = new ArrayList<>();
        route_waits = new ArrayList<>();
        best_route = new ArrayList<>();
    }
}
