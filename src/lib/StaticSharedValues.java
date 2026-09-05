package lib;

import problem.BP.BNode;
import problem.BP.Route;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class StaticSharedValues {

    public static boolean isFirstIteration;
    public static double lowerBound;
    public static double limLB, limUB;
    public static double crewWorkTime;
    public static int budget;
    public static double timerTower;
    public static double timerCrew;
    public static double heuristicCG;
    public static double exactCG;
    public static int pulse_extensions_tower;
    public static int pulse_extensions_crew;
    public static int bound_pruning_towers;
    public static int bound_pruning_crews;
    public static int bound_iterations_crew;
    public static int bound_iterations_tower;
    public static int pulse_iterations_crew;
    public static int pulse_iterations_tower;
    public static double lp_columnCPU;
    public static double lp_solvingCPU;
    public static double lp_columnTowerCPU;
    public static double lp_columnCrewCPU;
    public static int towerColumnCount;
    public static int crewColumnCount;
    public static int exactIterations;
    public static double timerRandomizedCrewSearch;
    public static int columnsRandomizedCrewSearch;
    public static int columnsExactCrewSearch;
    public static double timerExactTowerSearch;
    public static int columnsExactTowerSearch;
    public static int iceaIteration;
    public static StringBuilder iceaBuilder;
    public static double timerExactCrewSearch;
    public static int maximumTowerRouteLength;
    public static boolean feasibleFleetExperiment;
    public static double mipHeuristicTimer;
    public static double strongBranchingCPU;
    public static int strongBranchingSubproblemsSolved;


    public static void restart() {
        strongBranchingSubproblemsSolved = 0;
        strongBranchingCPU = 0;
        isFirstIteration = true;
        lowerBound = 0.0;
        limLB = 0.0;
        limUB = Double.MAX_VALUE;
        crewWorkTime = 0.0;
        budget = 0;
        timerTower = 0.0;
        timerCrew = 0.0;
        heuristicCG = 0.0;
        exactCG = 0.0;
        pulse_extensions_tower = 0;
        pulse_extensions_crew = 0;
        bound_pruning_towers = 0;
        bound_pruning_crews = 0;
        bound_iterations_crew = 0;
        bound_iterations_tower = 0;
        pulse_iterations_crew = 0;
        pulse_iterations_tower = 0;
        lp_columnCPU = 0.0;
        lp_solvingCPU = 0.0;
        lp_columnTowerCPU = 0.0;
        lp_columnCrewCPU = 0.0;
        towerColumnCount = 0;
        crewColumnCount = 0;
        exactIterations = 0;
        timerRandomizedCrewSearch = 0.0;
        columnsRandomizedCrewSearch = 0;
        columnsExactCrewSearch = 0;
        timerExactTowerSearch = 0.0;
        columnsExactTowerSearch = 0;
        iceaIteration = 0;
        iceaBuilder = new StringBuilder();
        timerExactCrewSearch = 0.0;
        maximumTowerRouteLength = 0;
        feasibleFleetExperiment = false;
        mipHeuristicTimer = 0.0;
    }
}
