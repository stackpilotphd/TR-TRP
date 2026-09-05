package problem.milp.genericSolverTools;

import com.gurobi.gurobi.*;
import ilog.concert.*;
import ilog.cplex.IloCplex;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Locale;

public class SolverTools {
    private final boolean CPLEX, GUROBI;
    private final String constraintPrefix = "con";
    private IloCplex cplex;
    private GRBModel model;
    private GRBEnv env;
    private Expression expression;

    private HashMap<Integer,Variable> variableHashMap;
    private HashMap<Integer,Constraint> constraintHashMap;

    public SolverTools(SolverType type) {
        switch (type){
            case CPLEX -> {
                this.CPLEX = true;
                this.GUROBI = false;
                System.out.println("USING ILOG CPLEX SOLVER");
            }
            case GUROBI -> {
                this.GUROBI = true;
                this.CPLEX = false;
                System.out.println("USING GUROBI SOLVER");
            }
            default -> {
                this.GUROBI = false;
                this.CPLEX = false;
                throwUnrecognizedSolverException();
            }
        }
    }
    public void initializeModel(int threads, double timeLimit, String logFileName, boolean disableLogging, boolean disableConsoleOutput) throws IloException, GRBException {
        if(CPLEX){
            cplex = new IloCplex();
            cplex.setParam(IloCplex.Param.TimeLimit, timeLimit);
            if(threads > 0) cplex.setParam(IloCplex.Param.Threads, threads);

            if(!disableLogging){
                try {
                    FileOutputStream logStream = new FileOutputStream(logFileName,true);
                    DateTimeFormatter formatter = DateTimeFormatter.ofPattern("EEE MMM d HH:mm:ss yyyy", Locale.ENGLISH);
                    String now = LocalDateTime.now().format(formatter);
                    PrintWriter headerWriter = new PrintWriter(logStream);
                    headerWriter.println("\nCPLEX logging started " + now);
                    headerWriter.flush();
                    cplex.setOut(logStream);
                } catch (FileNotFoundException e) {
                    throw new RuntimeException(e);
                }
            }
            if(disableConsoleOutput)
                cplex.setOut(null);
        } else if(GUROBI){
            // Create empty environment, set options, and start
            this.env = new GRBEnv(true);
            if(threads > 0) env.set(GRB.IntParam.Threads, threads);
            if(!disableLogging)
                env.set("logFile", logFileName);
            if(disableConsoleOutput)
                env.set(GRB.IntParam.OutputFlag, 0);
            env.start();
            // Create empty model
            this.model = new GRBModel(env);
            model.set(GRB.DoubleParam.TimeLimit, timeLimit);
        } else
            throwUnrecognizedSolverException();
        //--------------------------------------
        variableHashMap = new HashMap<>();
        constraintHashMap = new HashMap<>();
    }
    public int createVariable(VariableType type, double lowerBound, double upperBound, String name) throws IloException, GRBException {
        Variable var = new Variable();
        int variableID = variableHashMap.size()+1;
        if(name.isEmpty())
            name = "var"+variableID;
        if(CPLEX){
            switch (type) {
                case BINARY -> {
                    var.cplex_x = cplex.numVar(lowerBound, upperBound, IloNumVarType.Bool, name);
                }
                case INTEGER -> {
                    var.cplex_x = cplex.numVar(lowerBound, upperBound, IloNumVarType.Int, name);
                }
                case CONTINUOUS -> {
                    var.cplex_x = cplex.numVar(lowerBound, upperBound, IloNumVarType.Float, name);
                }
                default -> throwUnknownVariableTypeException();
            }
        } else if(GUROBI){
            switch (type) {
                case BINARY -> {
                    var.grb_x = model.addVar(lowerBound, upperBound, 0., GRB.BINARY, name);
                }
                case INTEGER -> {
                    var.grb_x = model.addVar(lowerBound, upperBound, 0., GRB.INTEGER, name);
                }
                case CONTINUOUS -> {
                    var.grb_x = model.addVar(lowerBound, upperBound, 0., GRB.CONTINUOUS, name);
                }
                default -> throwUnknownVariableTypeException();
            }
        }
        //---------------------------------------------
        variableHashMap.put(variableID,var);
        return variableID;
    }
    public void initializeExpression() throws IloException {
        resetExpression();
    }
    public void addTerm(double coeff, Integer varKey) throws IloException {
        if(expression.isNull())
            throwExpressionNotInitializedException();
        expression.addTerm(coeff,variableHashMap.get(varKey));
    }
    public void addConstant(double v) throws IloException {
        if(expression.isNull())
            throwExpressionNotInitializedException();
        expression.addConstant(v);
    }

    public double[][][][][] getVariableValues(int[][][][][] variableIDs) throws IloException, GRBException {
        int len = variableIDs.length;
        double[][][][][] v = new double[len][][][][];
        for (int i = 0; i < len; i++) {
            v[i] = getVariableValues(variableIDs[i]);
        }
        return v;
    }

    public double[][][][] getVariableValues(int[][][][] variableIDs) throws IloException, GRBException {
        int len = variableIDs.length;
        double[][][][] v = new double[len][][][];
        for (int i = 0; i < len; i++) {
            v[i] = getVariableValues(variableIDs[i]);
        }
        return v;
    }

    public double[][][] getVariableValues(int[][][] variableIDs) throws IloException, GRBException {
        int len = variableIDs.length;
        double[][][] v = new double[len][][];
        for (int i = 0; i < len; i++) {
            v[i] = getVariableValues(variableIDs[i]);
        }
        return v;
    }

    public double[][] getVariableValues(int[][] variableIDs) throws IloException, GRBException {
        int len = variableIDs.length;
        double[][] v = new double[len][];
        for (int i = 0; i < len; i++) {
            v[i] = getVariableValues(variableIDs[i]);
        }
        return v;
    }

    public double[] getVariableValues(int[] variableIDs) throws IloException, GRBException {
        int len = variableIDs.length;
        double[] v = new double[len];
        for (int i = 0; i < len; i++) {
            int id = variableIDs[i];
            if(id>0){
                v[i] = getVariableValue(id);
            }
        }
        return v;
    }
    public String getNonZeroValueStringFromVariable(int[][][][][] variableIDs) throws GRBException, IloException {
        StringBuilder stringBuilder = new StringBuilder();
        double[][][][][] values = getVariableValues(variableIDs);
        int len = values.length;
        for (int i = 0; i < len; i++) {
            stringBuilder.append(getNonZeroValueStringFromVariable(variableIDs[i]));
        }
        return stringBuilder.toString();
    }

    public String getNonZeroValueStringFromVariable(int[][][][] variableIDs) throws GRBException, IloException {
        StringBuilder stringBuilder = new StringBuilder();
        double[][][][] values = getVariableValues(variableIDs);
        int len = values.length;
        for (int i = 0; i < len; i++) {
            stringBuilder.append(getNonZeroValueStringFromVariable(variableIDs[i]));
        }
        return stringBuilder.toString();
    }

    public String getNonZeroValueStringFromVariable(int[][][] variableIDs) throws GRBException, IloException {
        StringBuilder stringBuilder = new StringBuilder();
        double[][][] values = getVariableValues(variableIDs);
        int len = values.length;
        for (int i = 0; i < len; i++) {
            stringBuilder.append(getNonZeroValueStringFromVariable(variableIDs[i]));
        }
        return stringBuilder.toString();
    }

    public String getNonZeroValueStringFromVariable(int[][] variableIDs) throws GRBException, IloException {
        StringBuilder stringBuilder = new StringBuilder();
        double[][] values = getVariableValues(variableIDs);
        int len = values.length;
        for (int i = 0; i < len; i++) {
            stringBuilder.append(getNonZeroValueStringFromVariable(variableIDs[i]));
        }
        return stringBuilder.toString();
    }

    public String getNonZeroValueStringFromVariable(int[] variableIDs) throws GRBException, IloException {
        StringBuilder stringBuilder = new StringBuilder();
        double[] values = getVariableValues(variableIDs);
        int len = values.length;
        for (int i = 0; i < len; i++) {
            double v = values[i];
            if(Operators.greaterThan(v,0.)){
                stringBuilder.append(getVariableName(variableIDs[i])).append(":")
                        .append(v)
                        .append(System.lineSeparator());
            }
        }
        return stringBuilder.toString();
    }


    public String getManualSolution() throws GRBException, IloException {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append("OBJECTIVE VALUE:")
                .append(getBestIntegerFeasibleSolutionValue())
                .append(System.lineSeparator());
        for(Integer variableID : variableHashMap.keySet()){
            double v = getVariableValue(variableID);
            if(Operators.greaterThan(v,0.)) {
                stringBuilder.append(getVariableName(variableID))
                        .append(":")
                        .append(v)
                        .append(System.lineSeparator());
            }
//            stringBuilder.append(System.lineSeparator());
        }
        return stringBuilder.toString();
    }

    public String concatenateString(int conID){
        return constraintPrefix+conID;
    }
    public String concatenateString(int conID, int i){
        return constraintPrefix+conID+","+i;
    }
    public String concatenateString(int conID, int i, int j){
        return concatenateString(conID,i)+","+j;
    }
    public String concatenateString(int conID, int i, int j, int k){
        return concatenateString(conID,i,j)+","+k;
    }
    public String concatenateString(int conID, int i, int j, int k, int l){
        return concatenateString(conID,i,j,k)+","+l;
    }
    public String concatenateString(int conID, int i, int j, int k, int l, int m){
        return concatenateString(conID,i,j,k,l)+","+m;
    }
    public String concatenateString(int conID, int i, int j, int k, int l, int m, int n){
        return concatenateString(conID,i,j,k,l,m)+","+n;
    }

    public int addConstraint(ConstraintSense sense, double RHS, int conID, int i, int j, int k, int l, int m, int n) throws IloException, GRBException {
        return addConstraint(sense,RHS,concatenateString(conID,i,j,k,l,m,n));
    }

    public int addConstraint(ConstraintSense sense, double RHS, int conID, int i, int j, int k, int l, int m) throws IloException, GRBException {
        return addConstraint(sense,RHS,concatenateString(conID,i,j,k,l,m));
    }

    public int addConstraint(ConstraintSense sense, double RHS, int conID, int i, int j, int k, int l) throws IloException, GRBException {
        return addConstraint(sense,RHS,concatenateString(conID,i,j,k,l));
    }

    public int addConstraint(ConstraintSense sense, double RHS, int conID, int i, int j, int k) throws IloException, GRBException {
        return addConstraint(sense,RHS,concatenateString(conID,i,j,k));
    }

    public int addConstraint(ConstraintSense sense, double RHS, int conID, int i, int j) throws IloException, GRBException {
        return addConstraint(sense,RHS,concatenateString(conID,i,j));
    }

    public int addConstraint(ConstraintSense sense, double RHS, int conID, int i) throws IloException, GRBException {
        return addConstraint(sense,RHS,concatenateString(conID,i));
    }

    public int addConstraint(ConstraintSense sense, double RHS, int conID) throws IloException, GRBException {
        return addConstraint(sense,RHS,concatenateString(conID));
    }

    public int addConstraint(ConstraintSense sense, double RHS, String name) throws IloException, GRBException {
        if(expression.isNull())
            throwExpressionNotInitializedException();
        int constraintID = constraintHashMap.size();
        if(name.isEmpty())
            name = "con"+constraintID;
        Constraint constraint = new Constraint();
        if(CPLEX){
            switch (sense) {
                case LE -> {
                    constraint.range = cplex.addLe(expression.cplex_e, RHS, name);
                }
                case EQ -> {
                    constraint.range = cplex.addEq(expression.cplex_e, RHS, name);
                }
                case GE -> {
                    constraint.range = cplex.addGe(expression.cplex_e, RHS, name);
                }
                default -> throwSenseUnknownException();
            }
        } else if(GUROBI){
            switch (sense) {
                case LE -> {
                    constraint.grbConstr = model.addConstr(expression.grb_e,GRB.LESS_EQUAL,RHS,name);
                }
                case EQ -> {
                    constraint.grbConstr = model.addConstr(expression.grb_e,GRB.EQUAL,RHS,name);
                }
                case GE -> {
                    constraint.grbConstr = model.addConstr(expression.grb_e,GRB.GREATER_EQUAL,RHS,name);
                }
                default -> throwSenseUnknownException();
            }
        }
        resetExpression(); //Important to make sure that other terms will not be added to the previous expression
        //------------------------------------------------------
        constraintHashMap.put(constraintID,constraint);
        return constraintID;
    }
    public void setObjective(ObjectiveSense sense) throws GRBException, IloException {
        if(expression.isNull())
            throwExpressionNotInitializedException();
        if(CPLEX){
            switch (sense) {
                case MIN -> {
                    cplex.addMinimize(expression.cplex_e);
                }
                case MAX -> {
                    cplex.addMaximize(expression.cplex_e);
                }
                default -> throwSenseUnknownException();
            }
        } else if(GUROBI){
            switch (sense) {
                case MIN -> {
                    model.setObjective(expression.grb_e,GRB.MINIMIZE);
                }
                case MAX -> {
                    model.setObjective(expression.grb_e,GRB.MAXIMIZE);
                }
                default -> throwSenseUnknownException();
            }
        }
        resetExpression();
    }
    public void setVariableBounds(int variableID, double lowerBound, double upperBound) throws IloException, GRBException {
        Variable var = variableHashMap.get(variableID);
        if(CPLEX){
            var.cplex_x.setLB(lowerBound);
            var.cplex_x.setUB(upperBound);
        } else if(GUROBI){
            var.grb_x.set(GRB.DoubleAttr.LB,lowerBound);
            var.grb_x.set(GRB.DoubleAttr.UB,upperBound);
        }
    }
    public void setConstraintBounds(int constraintID, double lowerBound, double upperBound) throws IloException, GRBException {
        Constraint constraint = constraintHashMap.get(constraintID);
        if(CPLEX){
            constraint.range.setBounds(lowerBound,upperBound);
        } else if(GUROBI){
            throw new IllegalArgumentException("This Method Does not work with Gurobi; Create two separate constraints instead.");
        }
    }
    public boolean solveRootNode() throws IloException, GRBException {
        if(CPLEX){
            cplex.setParam(IloCplex.IntParam.NodeLim,1); // Stop after root node.
            //disable the cuts to obtain the unlifted root bound
//            cplex.setParam(IloCplex.Param.MIP.Cuts.Gomory, -1);
//            cplex.setParam(IloCplex.Param.MIP.Cuts.MIRCut, -1);
//            cplex.setParam(IloCplex.Param.MIP.Cuts.Cliques, -1);
//            cplex.setParam(IloCplex.Param.MIP.Cuts.Covers, -1);
//            cplex.setParam(IloCplex.Param.MIP.Cuts.FlowCovers, -1);
//            cplex.setParam(IloCplex.Param.MIP.Cuts.PathCut, -1);
//            cplex.setParam(IloCplex.Param.MIP.Cuts.LiftProj, -1);
            cplex.solve();
            cplex.setParam(IloCplex.IntParam.NodeLim,Integer.MAX_VALUE);
            return isFeasibleModel();
        } else if(GUROBI){
            model.set(GRB.DoubleParam.NodeLimit,0); // Stop after root node.
            model.optimize();
//            int status = model.get(GRB.IntAttr.Status);
            //restore the parameters
            model.set(GRB.DoubleParam.NodeLimit,Double.MAX_VALUE);
            return isFeasibleModel();
        }
        return false;
    }
    public boolean solveModel() throws IloException, GRBException {
        if(CPLEX){
            cplex.solve();
            return isFeasibleModel();
        } else if(GUROBI){
            model.optimize();
            return isFeasibleModel();
        }
        return false;
    }
    public boolean isFeasibleModel() throws IloException, GRBException {
        if(CPLEX){
            IloCplex.Status status = cplex.getStatus();
            if (status == IloCplex.Status.InfeasibleOrUnbounded
                    || status == IloCplex.Status.Infeasible || status == IloCplex.Status.Unbounded)
                return false;
            return true;
        } else if(GUROBI){
            int status = model.get(GRB.IntAttr.Status);
            if(status == GRB.Status.INF_OR_UNBD || status == GRB.Status.INFEASIBLE || status == GRB.Status.UNBOUNDED)
                return false;
            return true;
        }
        return false;
    }
    public double getBestIntegerFeasibleSolutionValue() throws IloException, GRBException {
        if(!hasFeasibleSolution())
            return Double.MAX_VALUE;
        if(CPLEX){
            return cplex.getObjValue();
        } else if(GUROBI){
            return model.get(GRB.DoubleAttr.ObjVal);
        }
        return Double.MAX_VALUE;
    }
    public double getBestBound() throws IloException, GRBException {
        if(CPLEX){
            return cplex.getBestObjValue();
        } else if(GUROBI){
            return model.get(GRB.DoubleAttr.ObjBound);
        }
        throwUnrecognizedSolverException();
        return -1;
    }
    public double getMIPGapPercentage() throws GRBException, IloException {
        if(CPLEX){
            return cplex.getMIPRelativeGap() * 100;
        } else if(GUROBI){
            return model.get(GRB.DoubleAttr.MIPGap) * 100;
        }
        throwUnrecognizedSolverException();
        return -1;
    }


    public String getVariableName(Integer varID) throws GRBException {
        Variable var = variableHashMap.get(varID);
        if(CPLEX){
            return var.cplex_x.getName();
        } else if(GUROBI){
            return var.grb_x.get(GRB.StringAttr.VarName);
        }
        throwUnrecognizedSolverException();
        return "";
    }


    public double getVariableValue(Integer varID) throws GRBException, IloException {
        Variable var = variableHashMap.get(varID);
        if(CPLEX){
            return cplex.getValue(var.cplex_x);
        } else if(GUROBI){
            return var.grb_x.get(GRB.DoubleAttr.X);
        }
        throwUnrecognizedSolverException();
        return 0.;
    }
    public void clearModel() throws IloException, GRBException {
        if(CPLEX){
            cplex.clearModel();
            cplex.endModel();
            cplex.end();
        } else if(GUROBI){
            // Dispose of model and environment
            model.dispose();
            env.dispose();
        }
        variableHashMap.clear();
        constraintHashMap.clear();
        expression = null;
    }
    private void resetExpression() throws IloException {
        expression = new Expression();
    }
    public boolean hasFeasibleSolution() throws IloException, GRBException {
        if(CPLEX){
            return cplex.getSolnPoolNsolns() > 0;
        } else if(GUROBI){
            return model.get(GRB.IntAttr.SolCount) > 0;
        }
        return false;
    }

    public void exportModel(String nameWithoutExtension) throws IloException, GRBException {
        if(CPLEX){
            cplex.exportModel(nameWithoutExtension+".lp");
        } else if(GUROBI){
            model.write(nameWithoutExtension+".lp");
        }
    }

    public void writeSolution(String nameWithoutExtension) throws IloException, GRBException {
        if(CPLEX){
            cplex.writeSolution(nameWithoutExtension+".sol");
        } else if(GUROBI){
            model.write(nameWithoutExtension+".sol");
        }
    }

    //--------------------CLASSES---------------------
    private class Variable{
        IloNumVar cplex_x;
        GRBVar grb_x;
    }
    private class Constraint{
        IloRange range;
        GRBConstr grbConstr;
    }
    private class Expression{
        IloNumExpr cplex_e;
        GRBLinExpr grb_e;

        private Expression() throws IloException {
            if(CPLEX){
                cplex_e = cplex.numExpr();
            } else if(GUROBI){
                grb_e = new GRBLinExpr();
            }
        }

        private boolean isNull(){
            if(CPLEX){
                if(cplex_e==null)
                    return true;
            } else if(GUROBI){
                if(grb_e==null)
                    return true;
            }
            return false;
        }

        private void addTerm(double coeff, Variable variable) throws IloException {
            if(CPLEX){
                cplex_e = cplex.sum(cplex_e,cplex.prod(coeff, variable.cplex_x));
            } else if(GUROBI){
                grb_e.addTerm(coeff, variable.grb_x);
            }
        }

        private void addConstant(double v) throws IloException {
            if(CPLEX){
                cplex_e = cplex.sum(cplex_e,v);
            } else if(GUROBI){
                grb_e.addConstant(v);
            }
        }
    }




    //-------------------------------------------------

    
    //--------------------EXCEPTIONS---------------------
    private void throwExpressionNotInitializedException() {
        throw new IllegalArgumentException("Expression has not been initialized");
    }

    private void throwUnrecognizedSolverException() {
        throw new IllegalArgumentException("This solver type is unrecognized");
    }
    
    private void throwSenseUnknownException() {
        throw new IllegalArgumentException("Unknown sense");
    }


    private void throwUnknownVariableTypeException() {
        throw new IllegalArgumentException("Unknown variable type");
    }
    //-------------------------------------------------
}
