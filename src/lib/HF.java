package lib;

import problem.Constants;

import java.util.ArrayList;
import java.util.List;

public class HF {

    public static double computeEuclidianDistance(double x, double y, double x1, double y1) {
        return Math.sqrt(Math.pow(x - x1, 2) + Math.pow(y - y1, 2));
    }

    public static double truncate(double v) {
        if (v >= 1e6)
            return v;
//        if(v <= 1e-6)
//            return 0;
//        if(v >= 0.0049 && v <= 0.0051)
//            return 0.005;
        double n = 100.;
//        double a = (double) Math.round(v * 10.) / 10.; //one decimal place
        double a = Math.round(v * n) / n; //two decimal spaces
        while(a < 1e-6) {
            n *= 10;
            a = Math.round(v * n) / n;
            if(n >= 1e6)
                break;
        }
        return a;
    }


  public static String getVarName(List<Integer> scripts){
        StringBuilder stringBuilder = new StringBuilder();
      for (int i = 0; i < scripts.size()-1; i++) {
          stringBuilder.append(scripts.get(i));
          stringBuilder.append("_");
      }
      stringBuilder.append(scripts.getLast());
      return stringBuilder.toString();
  }


  public static double truncateToTwoDecimals(double v){
      return Math.floor(v * 100) / 100.0;
  }

    public static double truncate(double v, int precision) {
        if (v >= 1e6 || v <= 1e-6)
            return v;
        double n = precision * 1.;
        double a = Math.round(v * n) / n;
        //TODO
//        while(a < 1e-6) {
//            n *= 10;
//            a = Math.round(v * n) / n;
//            if(n >= 1e6)
//                break;
//        }
//        if(a < 0.1){
//            String gotcha = "";
//        }
//        if(precision <= 10 + 1e-6){
//            if(a < 0.1){
//                double surrogate = a*10;
//                a = surrogate;
//            }
//        }
        return a;
    }


    public static int ceiling(double v){
        int tv=(int)v;
        if(tv == v)
            return tv;
        else
            return tv+1;
    }

    public static int floor(double v){
        return (int)v;
    }

    public static int d2i(double v){
        int iv = (int) v;
        if(iv > v - 1e-6)
            return iv;
        else
            return iv + 1;
    }

    public static  int double2integer(double value) {
        // Extract the fractional part of the double
        double fractionalPart = value - (int) value;

        // Apply the specified rules
        if (fractionalPart >= 0.0 && fractionalPart < 0.5) {
            // For range 0.0 ... 0.49, round down to the nearest integer
            return (int) value;
        } else {
            // For range 0.5 ... 0.999, round up to the nearest integer
            return (int) value + 1;
        }
    }

    public static boolean is_fractional(double v){
        if( v > (int) v + 1e-6 && v < (int) v + 1 - 1e-6)
            return true;
        else
            return false;
    }

    public static double round(double value, int decimals) {
        double factor = Math.pow(10, decimals);
        return Math.round(value * factor) / factor;
    }

    public static double roundConstant(double value) {
        double factor = Math.pow(10, Constants.DECIMALS);
        return Math.round(value * factor) / factor;
    }


    /**
     * High precision
     * @param v
     * @return
     */
    public static boolean is_fractionalHP(double v){
        //lower precision
//        if( v > (int) v + 0.0001 && v < (int) v + 1 - 0.0001)
//            return true;
//        else
//            return false;
        //higher precision
        if( v > (int) v + 1e-6 && v < (int) v + 1 - 1e-6)
            return true;
        else
            return false;
    }

    public static boolean is_fractional1(double v){
        if( v > (int) v + 0.1 && v < (int) v + 1 - 0.1)
            return true;
        else
            return false;
    }

    public static double frac_cost(double v){
        double line = floor(v) + 0.5;
        if(v >= line)
            return v - line;
        else
            return line - v;
    }

    public static boolean is_one_zero(double v){
        if(v < 1e-6 || v > 1- 1e-6 && v < 1 + 1e-6)
            return true;
        else return false;
    }

    public static double one_zero_cost(double v){
        if(v < 0.5)
            return 0.5 - v;
        else
            return v - 0.5;
    }

    public static ArrayList<ArrayList<Integer>> parseNestedList(String line) {
        // Remove the outer brackets
        line = line.substring(1, line.length() - 1);

        // Split the string into individual sublists
        String[] sublistStrings = line.split("\\], \\[");

        // Initialize the main list
        ArrayList<ArrayList<Integer>> nestedList = new ArrayList<>();

        // Process each sublist
        for (String sublistString : sublistStrings) {
            // Remove any remaining brackets
            sublistString = sublistString.replace("[", "").replace("]", "");

            // Split the sublist into individual integers
            String[] integerStrings = sublistString.split(", ");

            // Convert the strings to integers and add to a new ArrayList
            ArrayList<Integer> sublist = new ArrayList<>();
            for (String integerString : integerStrings) {
                sublist.add(Integer.parseInt(integerString));
            }

            // Add the sublist to the main list
            nestedList.add(sublist);
        }

        return nestedList;
    }


    public static void triangulateMatrix(double[][] matrix, boolean shouldTruncate, int precision){
        boolean violated = true;
//        out.println("Running the Floyd-Warshall algorithm");
        int iter = 1;
        while (violated){
            while (violated) {
//                out.println("\titer"+iter++);
                violated = triangulateInnerHelper(matrix);
            }
            if(shouldTruncate){
//                out.println("truncating the values");
                for (int i = 0; i < matrix.length; i++){
                    for (int j = 0; j < matrix.length; j++) {
                        matrix[i][j] = HF.truncate(matrix[i][j],precision);
                    }
                }
                violated = triangulateInnerHelper(matrix);
            }
        }
//        out.println("over");
    }

    private static boolean triangulateInnerHelper(double[][] dM) {
        boolean res = false;
        int length = dM.length; //length -1 excludes the end depot
        double[][] shortest = new double[length][length];
        for(int i = 0; i < length-1; i++){
            //---------------------------------------------------------------
            //---calculate the shortest path from i to the other points------
            //---------------------------------------------------------------
            double[] shi = new double[length];
            for(int j = 0; j < length; j++) {
                shi[j] = dM[i][j];
            }

            for(int j = 0; j < length; j++){
                boolean updated = false;
                for(int k = 0; k < length; k++){
                    if(shi[k] < 1e-6)
                        continue;
                    for(int h = 0; h < length-1; h++){
                        if(shi[h] < 1e-6)
                            continue;
                        if(h != k && (shi[k] - 1e-6 > shi[h] + dM[h][k]) && shi[k] < 1e6) {
//                            if(dM[h][k] < 1e-6) {
//                                System.out.println("Zero:("+i+","+k+") > ("+i+","+h+")+("+h+","+k+"):"+shi[k]+">"+shi[h]+"+"+dM[h][k]);
//                                continue;
//                            }
//                            System.out.println("Updating:("+i+","+k+") > ("+i+","+h+")+("+h+","+k+"):"+shi[k]+">"+shi[h]+"+"+dM[h][k]);
                            shi[k] = shi[h] + dM[h][k];
//                            if(shi[k] <= 1e-6 || shi[h] <= 1e-6 || dM[h][k] <= 1e-6)
//                                throw new IllegalArgumentException("bug");
                            updated = true;
                            res = true;
                        }
                    }
                }
                if(!updated)
                    break;
            }

            //---------------------------------------------------------------
            //---copy the matrix---------------------------------------------
            //---------------------------------------------------------------
            for(int j = 0; j < length; j++) {
                shortest[i][j] = shi[j];
            }
        }
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                dM[i][j] = shortest[i][j];
            }
        }
        return res;
    }



}
