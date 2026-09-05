package problem.milp.genericSolverTools;

import java.util.ArrayList;
import java.util.HashSet;

public class Operators {
    private Operators() {
        // prevent instantiation
    }

    public static boolean equals(double a, double b) {
        return Math.abs(a - b) <= Constants.EPS;
    }

    public static boolean greaterThan(double a, double b) {
        return a > b + Constants.EPS;
    }

    public static boolean lessThan(double a, double b) {
        return a < b - Constants.EPS;
    }

    public static boolean greaterThanEqual(double a, double b) {
        return a > b - Constants.EPS;
        // or: return greaterThan(a, b) || equals(a, b);
    }

    public static boolean lessThanEqual(double a, double b) {
        return a < b + Constants.EPS;
        // or: return lessThan(a, b) || equals(a, b);
    }

    public static boolean isSubset(long v1, long v2) {
        return (v1 & v2) == v1;
    }

    public static long add(long v, int i) {
        return v | (1L << i);
    }
    public static long remove(long v, int i) {
        return v & ~(1L << i);
    }
    public static boolean contains(long v, int i) {
        return ((v & (1L << i)) != 0);
    }

    public static long setDifference(long a, long b) {
        return a & ~b;
    }

    public static long setUnion(long a, long b) {
        return a | b;
    }

    public static long setIntersection(long a, long b) {
        return a & b;
    }

    public static ArrayList<Integer> toList(long v1) {
        ArrayList<Integer> items = new ArrayList<>();
        while (v1 != 0) {
            int i = Long.numberOfTrailingZeros(v1);
            items.add(i);
            v1 &= v1 - 1;
        }
        return items;
    }

    public static double elapsedSince(double start) {
        return (System.nanoTime() - start) * 1e-9;
    }

    public static double getTimePassed(double since) {
        return System.nanoTime() - since;
    }

    public static double getTimePassedSeconds(double since){
        return getTimePassed(since) * 1e-9;
    }


    public static boolean isFractional(double v) {
        double f = Math.floor(v);
        return v > f + Constants.EPS && v < f + 1. - Constants.EPS;
    }

    public static double distToHalf(double v) {
        double k = Math.floor(v);
        return Math.abs(v - (k + 0.5));
    }

    public static double truncate(double v, int precision) {
        double scale = Math.pow(10, precision);
        return greaterThanEqual(v,0.) ? Math.floor(v * scale) / scale : Math.ceil(v * scale) / scale;
    }
    public static long getLongFromHashSet(HashSet<Integer> set) {
        long v1 = 0x00L;
        for(Integer i : set)
            v1 = add(v1,i);
        return v1;
    }
    public static long getLongFromList(ArrayList<Integer> set) {
        long v1 = 0x00L;
        for(Integer i : set)
            v1 = add(v1,i);
        return v1;
    }
}
