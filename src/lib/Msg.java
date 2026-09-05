package lib;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class Msg {
    public static final int triangleIneqCode = 101;
    public static final String triangleIneq = "The data does not fulfill the triangle inequality.";

    public static final int infeasibilityCode = 201;
    public static final String infeasibility = "The model is infeasible.";

    public static final String genCapacity = "The generator capacity is exceeded.";
    public static final String algoType = "This algorithm type is not recognised";

    public static final String pTreePerClu = "The number of trees in a cluster is incorrect.";
    public static final String vertexNum = "Unsupported vertex number.";
    public static final String customerNum = "Unsupported customer number.";
    public static final String noRoot = "No root has been identified.";
    public static final String noElement = "The tree does not contain such an element.";
    public static final String brErr = "Branching error.";
    public static final String candidateErr = "The algorithm failed to find a suitable candidate.";
    public static final String illegalArc = "Infeasible arc is traversed!";;
    public static final String emptyRouteException = "Empty route!";
    public static final String emptySetException = "The set is empty.";
    public static final String requireNonNull = "The argument must be NonNull!";
    public static final String vehicleBranchErr = "Upper/lower bound is incorrect during branching.";
    public static final String sizeMismatchException = "The array sizes do not match!";
    public static final String valueMismatchException = "The values do not match!";
    public static final String vehicleCap = "Vehicle capacity is violated.";
    public static final String setToolException = "Set Tool Exception";
    public static final String unknownException = "Why? This should not happen";
    public static final String nonElementary = "The solution is not elementary.";
    public static final String infeasSolution = "The solution is infeasible." ;
    public static final String incompleteMethod = "Incomplete method.";
    public static final String infeasibleMethod = "Method infeasibility. Failed to achieve the desired result.";
    public static final String negativeValue = "Negative value.";;
    public static final String largeValue = "Large value.";
    public static final String stop = "terminating...";
    public static final String gotcha = "gotcha!";

    public static String getValueMismatchException(double v1, double v2){
        throw new IllegalStateException(Msg.valueMismatchException + ": " + v1 + " vs " + v2 + ".") ;
    }
    public static void stop(){
        throw new IllegalStateException(stop);
    }

    public static String getCurrentDateTime() {
        // Get current date and time
        LocalDateTime currentDateTime = LocalDateTime.now();
        // Define the desired date and time format
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy.MM.dd.HH.mm");
        // Format the current date and time
        return currentDateTime.format(formatter);
    }
    public static void writeCurrentDateTimeToFile() {
        String fileName = "start.txt";
        LocalDateTime currentDateTime = LocalDateTime.now();
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");
        String formattedDateTime = currentDateTime.format(formatter);
        System.out.println(formattedDateTime);
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(fileName))) {
            writer.write(formattedDateTime);
        } catch (IOException e) {
            System.err.println("An error occurred while writing to the file: " + e.getMessage());
        }
    }

    public static void columnRepetition() {
        throw new IllegalArgumentException("column repitition");
    }

    public static void incompleteMethod() {
        throw new IllegalStateException(Msg.incompleteMethod);
    }

    public static void emptyRouteException() {
        throw new IllegalStateException(Msg.emptyRouteException);
    }

    public static void gotcha() {
        System.out.println("gotcha");
    }

    public static void stop(String s) {
        throw new IllegalStateException(s);
    }

    public static void infeasibleArc() {
        throw new IllegalStateException("infeasible arc traversal");
    }

    public static void todo(){
        //use this to mark any important debug
    }
    public static void todo(String date){
        //use this to mark any important debug
    }
    public static void check(String date){
        //use this to mark any important debug
    }

    public static void gotcha(String s) {
        System.out.println(s);
    }

    public static String shouldNotHappen() {
        throw new IllegalArgumentException("this should not happen");
    }

    public static void infeasibility() {
        throw new IllegalStateException(Msg.infeasibility);
    }

//    public static void todo(String s) {
//        System.out.println("TODO:"+s);
//    }
}
