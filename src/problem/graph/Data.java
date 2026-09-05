package problem.graph;

import lib.Cet;

import java.util.ArrayList;

public class Data {

    private static Data instance;

    public static Data getInstance() {
        if (instance == null)
            instance = new Data();
        return instance;
    }


    private int nodeNumber;
    private int customerNumber;
    private double[][] distanceMatrix;
    private int crewNumber;
    private double[] repairTimeMatrix;
    private int towerNumber;
    private double[] positionTimeMatrix;
    private double[][] crewTravelTimeMatrix;
    private double[][] towerTravelTimeMatrix;
    private double[] weights;
    private int end_depot;
    private double cM;
    private double tM;
    private double serviceM;
    private double pricingTM;
    private double tTravelM;
    private double[] taskWeight;

    public int getCrewNumber() {
        return crewNumber;
    }

    public void setCrewNumber(int crewNumber) {
        this.crewNumber = crewNumber;
    }

    public double[][] getCrewTravelTimeMatrix() {
        return crewTravelTimeMatrix;
    }

    public void setCrewTravelTimeMatrix(double[][] crewTravelTimeMatrix) {
        this.crewTravelTimeMatrix = crewTravelTimeMatrix;
    }

    public double[][] getDistanceMatrix() {
        return distanceMatrix;
    }

    public void setDistanceMatrix(double[][] distanceMatrix) {
        this.distanceMatrix = distanceMatrix;
    }

    public static void setInstance(Data instance) {
        Data.instance = instance;
    }

    public int getNodeNumber() {
        return nodeNumber;
    }

    public void setNodeNumber(int nodeNumber) {
        this.nodeNumber = nodeNumber;
        end_depot = nodeNumber;
        this.customerNumber = nodeNumber-1;
    }

    public double[] getPositionTimeMatrix() {
        return positionTimeMatrix;
    }

    public void setPositionTimeMatrix(double[] positionTimeMatrix) {
        this.positionTimeMatrix = positionTimeMatrix;
    }

    public double[] getServiceTimeMatrix() {
        return repairTimeMatrix;
    }

    public void setRepairTimeMatrix(double[] repairTimeMatrix) {
        this.repairTimeMatrix = repairTimeMatrix;
    }

    public int getTowerNumber() {
        return towerNumber;
    }

    public void setTowerNumber(int towerNumber) {
        this.towerNumber = towerNumber;
    }

    public double[][] getTowerTravelTimeMatrix() {
        return towerTravelTimeMatrix;
    }

    public void setTowerTravelTimeMatrix(double[][] towerTravelTimeMatrix) {
        this.towerTravelTimeMatrix = towerTravelTimeMatrix;
    }

    public double[] getWeights() {
        return weights;
    }

    public void setWeights(double[] weights) {
        this.weights = weights;
    }



    public int getCustomerNumber() {
        return customerNumber;
    }

    public void setServiceM(double v) {
        serviceM = v;
    }

    public double getServiceM() {
        return serviceM;
    }

    public double getCrewM() {
        return cM;
    }

    public double getBigTM() {
        return tM;
    }

    public double getPricingTM() {
        return pricingTM;
    }

    public void setBiGMs(double cM, double tM, double serviceM, double pricingTM, double tTravelM) {
        this.serviceM = serviceM;
        this.cM = cM;
        this.tM = tM;
        this.pricingTM = pricingTM;
        this.tTravelM = tTravelM;
    }

    public double getTowerMTravel() {
        return tTravelM;
    }

    private double[] minimalLambda;
    private double[][] minIJlambda;
    public void setLambdaLBs(double[] minimalLambda, double[][] minIJlambda) {
        this.minimalLambda = minimalLambda;
        this.minIJlambda = minIJlambda;
    }

    public double[][] getMinIJlambda() {
        return minIJlambda;
    }

    public double[] getMinimalLambda() {
        return minimalLambda;
    }


    public void setPricingTowerM(double v) {
        serviceM = v;
    }

    private double[] earlisetServiceStartTimes;
    public void setEarlisetServiceStartTimes(double[] earliestServiceStartTimes) {
        this.earlisetServiceStartTimes = earliestServiceStartTimes;
    }

    public double[] getEarlisetServiceStartTimes() {
        return earlisetServiceStartTimes;
    }

    private ArrayList<Integer>[] task2zones; private ArrayList<Integer>[] zone2tasks;

    public ArrayList<Integer>[] getZone2tasks() {
        return zone2tasks;
    }

    public void setZone2tasks(ArrayList<Integer>[] zone2tasks) {
        this.zone2tasks = zone2tasks;
    }

    public ArrayList<Integer>[] getTask2zones() {
        return task2zones;
    }

    public void setTask2zones(ArrayList<Integer>[] task2zones) {
        this.task2zones = task2zones;
    }

    public void setArrays(ArrayList<Integer>[] task2zones, ArrayList<Integer>[] zone2tasks) {
        setZone2tasks(zone2tasks);
        setTask2zones(task2zones);
    }

    private int taskNumber;
    public int getTasks() {
        return taskNumber;
    }

    public void setTaskNumber(int taskNumber) {
        this.taskNumber = taskNumber;
        this.crewEndDepot = taskNumber;
    }

    private int crewEndDepot;
    public int getDepotEnd(boolean isCrew) {
        if(isCrew)
            return crewEndDepot;
        return end_depot;
    }

        public int getDepotEnd() {
        return end_depot;
    }

    private double[] omegaLBs;
    public void setOmegaLBs(double[] times) {
        omegaLBs = times;
    }

    public double[] getOmegaLBs() {
        return omegaLBs;
    }

    private double[] zoneLBs;
    public void setZoneLBs(double[] zoneLBs) {
        this.zoneLBs = zoneLBs;
    }

    public double[] getZoneLBs() {
        return zoneLBs;
    }

    public void setDepotEnd(int depotEnd) {
        end_depot = depotEnd;
    }

    private double horizon;
    public void setTimeLimit(double maxT) {
        horizon = maxT;
    }

    public double getHorizon() {
        return horizon;
    }

    private ArrayList<ArrayList<Cet>> allSets;
    public ArrayList<ArrayList<Cet>> getAllUnreachableSets() {
       return allSets;
    }


    public void setUnreachableSets(ArrayList<ArrayList<Cet>> allSets) {
        this.allSets= allSets;
    }

    private long mI;
    public long getAllMask() {
        return mI;
    }

    public void setAllMask(long mI) {
        this.mI = mI;
    }

    private double maxZ;
    public void setMaximumZoverAllTowers(double maximumValueOfTotalServiceDurationOverAllTowers) {
        maxZ = maximumValueOfTotalServiceDurationOverAllTowers;
    }

    public double getMaxZ() {
        return maxZ;
    }

    private    double[][] deviations;
    private double[] taskDeviations;
    public void setCrewTravelTimeDeviations(double[][] deviations) {
        this.deviations = deviations;
    }

    public double[][] getCrewTravelTimeDeviations() {
        return deviations;
    }

    public void setTaskRepairTimeDeviations(double[] taskDeviations) {
        this.taskDeviations = taskDeviations;
    }

    public double[] getServiceDeviation() {
        return taskDeviations;
    }

    private int maxCountForCrews;
    public void setMaxCrewVisits(int maxCountForCrews) {
        this.maxCountForCrews = maxCountForCrews;
    }

    public int getCrewMaxVisit() {
        return maxCountForCrews;
    }

    private long[][] crews_i2t2nI;
    public void setCrewDpLongs(long[][] i2t2nI) {
        crews_i2t2nI = i2t2nI;
    }

    public long[][] getCrewUnreachableI2T2set() {
        return crews_i2t2nI;
    }

    private long[] zone2taskLongs;
    public void setZone2taskLongs(long[] arr) {
        zone2taskLongs = arr;
    }

    public long[] getZone2taskLongs() {
        return zone2taskLongs;
    }




    private int[] population;
    public void setPopulation(int[] numberPeople) {
        population = numberPeople;
    }

    public int[] getPopulation(){
        return population;
    }

    private  int[] priority;
    public int[] getZonePriority() {
        return priority;
    }

    public void setZonePriority(int[] priorities) {
        this.priority = priorities;
    }

    private  int[] taskPriority;
    public int[] getTaskPriority() {
        return taskPriority;
    }

    public void setTaskPriority(int[] priorities) {
        this.taskPriority = priorities;
    }

    public void modifyServiceDeviation(double modified, int i) {
        taskDeviations[i] = modified;
    }

    public void setTaskWeights(double[] taskWegith) {
        this.taskWeight = taskWegith;
    }

    public double[] getTaskWeight(){
        return taskWeight;
    }

    private boolean[] isShared;
    public void setSharedTasks(boolean[] isShared) {
        this.isShared = isShared;
    }

    public boolean[] getIsShared(){
        return isShared;
    }
}
