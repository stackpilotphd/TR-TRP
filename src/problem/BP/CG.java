package problem.BP;

import ilog.concert.IloException;
import lib.*;
import problem.Constants;
import problem.graph.Data;

import java.util.ArrayList;
import java.util.HashSet;

public class CG {

    private static final double EPS = 1e-6;

    private int iterH;
    private int iterE;


    private int[] customer2numberOfColumns;
    private int[][] customer2pairInColumns;
    private HashSet<ArrayList<Integer>> heuristicSet;

    private int hCounter;



    public CG() {

    }

    public void initializeHeuristicStructures() {
        Data data = Data.getInstance();
        this.customer2numberOfColumns = new int[data.getTasks() + 1];
        this.customer2pairInColumns = new int[data.getTasks() + 1][data.getTasks() + 1];
        this.heuristicSet = new HashSet<>();
    }

    public void run(LP lp) throws IloException {

        lp.setExactGeneration(true);
        lp.setFirstHeuristicPass(true);


        TimerHelper timerHelper = TimerHelper.getInstance();

        double startTime = System.nanoTime();
        double cgStart = timerHelper.getTime();

        boolean stoppedByTimeout = false;

        try {

            if (lp.getBNode().getDepth() <= 0 && Utility.algo != 22) {
                initializeHeuristicStructures();
            }

            double costAtParentNode =
                    lp.getBNode().getDepth() <= 0
                            ? -Double.MAX_VALUE
                            : lp.getBNode().getFather().getCost();

            iterH = 0;
            iterE = 0;
            hCounter = 0;

            /*
             * ------------------------------------------------------------
             * Integrated heuristic-exact CG loop.
             *
             * Logic:
             * 1. Run heuristics until heuristic pricing finds no column.
             * 2. Run one exact pricing pass.
             * 3. If exact pricing finds a column, LP has already been solved
             *    inside e(lp), so restart heuristics from the beginning.
             * 4. Stop only when both heuristic and exact pricing find nothing.
             * ------------------------------------------------------------
             */
            while (!timerHelper.hasTimedOut()) {

                /*
                 * --------------------------------------------------------
                 * 1. Heuristic phase: run to saturation.
                 * --------------------------------------------------------
                 */
                if (Constants.GUIDED_PRICING_HEURISTIC) {

                    double startH = System.nanoTime();

                    while (!timerHelper.hasTimedOut()) {

                        boolean heuristicFound = h(lp);

                        if (!heuristicFound) {
                            break;
                        }

                        /*
                         * h(lp) is assumed to:
                         * - add columns if found,
                         * - solve the LP,
                         * - return true.
                         *
                         * Therefore, after every successful heuristic iteration,
                         * the next heuristic call sees fresh duals.
                         */
                    }

                    StaticSharedValues.heuristicCG += (System.nanoTime() - startH);
                    lp.setFirstHeuristicPass(false);
                }

                if (timerHelper.hasTimedOut()) {
                    stoppedByTimeout = true;
                    break;
                }

                /*
                 * --------------------------------------------------------
                 * 2. Exact phase: run only one exact pricing pass.
                 *
                 * If exact pricing finds a column, e(lp) should:
                 * - add columns,
                 * - solve the LP,
                 * - return true.
                 *
                 * Then we continue the outer loop, which restarts heuristics.
                 * --------------------------------------------------------
                 */
                double startE = System.nanoTime();

                boolean exactFound = e(lp);

                StaticSharedValues.exactCG += (System.nanoTime() - startE);

                if (timerHelper.hasTimedOut()) {
                    stoppedByTimeout = true;
                    break;
                }

                if (exactFound) {
                    /*
                     * This is the key change:
                     * exact DP found at least one column, so restart heuristic loop.
                     */
                    continue;
                }

                /*
                 * No exact column found after heuristic saturation.
                 * Therefore, CG has converged.
                 */
                break;
            }

            if (timerHelper.hasTimedOut()) {
                stoppedByTimeout = true;
            }



            if (Constants.CONSOLE) {
                System.out.println(
                        "CG:H#" + iterH
                                + ",E#" + iterE
                                + ",CPU:" + timerHelper.getTimePassedSeconds(cgStart)
                );
                lp.printSolution();
            }

            double endTime = (System.nanoTime() - startTime);

            if (lp.getBNode().getDepth() <= 0) {
                StaticSharedValues.iceaBuilder.append(lp.getCost());
                StaticSharedValues.iceaBuilder.append(",");
                StaticSharedValues.iceaBuilder.append(endTime * 1e-9);
                StaticSharedValues.iceaBuilder.append(",");
            }

            /*
             * If active slacks remain, the original master may still be infeasible
             * with respect to the generated columns. Do not validate final pricing.
             */
            if (lp.hasActiveSlaks()) {
                return;
            }



            /*
             * Child LP lower bound should not be lower than parent LP lower bound.
             */
            if (!BendersCuts.toggle && lp.getCost() < costAtParentNode) {
                if (Math.abs(lp.getCost() - costAtParentNode) > 1e-4) {

                    lp.printSolution();

                    lp.getBNode().printBranchStrings();

                    System.out.println("BUG IS IN THREAD " + lp.getBNode().getIceaID() + "!>");
                    System.out.println(
                            "Child bound lower than parent bound: "
                                    + lp.getCost()
                                    + " < "
                                    + costAtParentNode
                    );

                    System.exit(104321);
                }
            }

            if (lp.getBNode().getDepth() > 0) {
                lp.checkBranchingConstraints();
            }

            if (Constants.DETERMINE_MINIMUM_NUMBER_CREWS) {
                StaticSharedValues.feasibleFleetExperiment = true;
                return;
            }

            if (Utility.algo == 41 || Constants.COST_OF_PRIORITY) {
                lp.checkPriorities();
            }

            if(Constants.BT_CONSOLE) {
                System.out.print("Lp cost:" + lp.getCost());
                System.out.println();
            }
            if (Constants.CONSOLE) {
//                lp.printSolution();


                System.out.println("CG Time:" + endTime * 1e-9);

            }

        } finally {

            /*
             * Important:
             * Make sure this flag is reset even if the method returns early
             * or throws an exception.
             */
            lp.setExactGeneration(false);

        }
    }



    public boolean h(LP lp) throws IloException {
        iterH++;
        return heuristicPass(lp);
    }

    public boolean e(LP lp) throws IloException {
        if (BendersCuts.toggle) {
            return false;
        }
        if(BendersCuts.toggle && lp.getCost() < 900 && Data.getInstance().getTasks() > 42)
            return false; //ease the heuristic's performance

        iterE++;
        return exactPass(lp);
    }

    /**
     * Heuristic pricing phase.
     *
     * This method is intentionally restart-immediate:
     * as soon as one pricing heuristic finds columns, columns are added,
     * the LP is solved, and the caller restarts pricing with fresh duals.
     */
    private boolean heuristicPass(LP lp) throws IloException {

        if (Constants.CONSOLE) {
            hCounter++;
            System.out.println("Heuristic iteration:" + hCounter);
        }

        /*
         * 1. Tower heuristic pricing.
         */
        {

            if(Constants.CONSOLE) System.out.println("Tower DP-H Start...");
            ArrayList<Route> towerColumns;
            double startTower = System.nanoTime();

            if(lp.isFifthThread() || Constants.BRANCH_AND_PRICE) {
                DPT towerDP = new DPT();
                towerDP.run(lp, true);
                towerColumns = towerDP.getRoutes();
            } else {
                TowerPulse pulse = new TowerPulse();
                towerColumns = pulse.run(lp,true);
            }

            double elapsedTower = System.nanoTime() - startTower;


            StaticSharedValues.timerTower += elapsedTower;
            if(Constants.CONSOLE) System.out.println("Tower DP-H Over...");
            if (!towerColumns.isEmpty()) {
                lp.addColumns(towerColumns, new ArrayList<>());
                lp.solve();
                return true;
            }
        }

        /*
         * 2. Crew heuristic pricing.
         */
        if (lp.getBNode().getDepth() <= 0 && Utility.algo != 22) {

            if(Constants.CONSOLE) System.out.println("Crew DP-H Start...");
            ArrayList<Route> crewColumns;

            if (Constants.GUIDED_PRICING_HEURISTIC && !BendersCuts.toggle) {
                crewColumns = runGuidedCrewHeuristic(lp);
            } else {
                crewColumns = runReducedCrewHeuristic(lp);
            }
            if(Constants.CONSOLE) System.out.println("Crew DP-H Over...");
            if (!crewColumns.isEmpty()) {
                lp.addColumns(new ArrayList<>(), crewColumns);
                lp.solve();
                return true;
            }
        }

        return false;
    }

    /**
     * Exact pricing phase.
     *
     * This is the critical part:
     * - Crew exact pricing is run first.
     * - If it finds a column, add it, solve, and restart.
     * - Tower exact pricing is run only if crew exact pricing found nothing.
     * - Tower DP is called with false, not true.
     */
    private boolean exactPass(LP lp) throws IloException {

        /*
         * 1. Exact crew pricing.
         */
        if (Utility.algo != 22) {

            if(Constants.CONSOLE) System.out.println("Exact iteration Crew Start...");
            double startCrew = System.nanoTime();

            ArrayList<Route> crewColumns;
            Pulse pulse = new Pulse();
            crewColumns = pulse.run(lp, false);


            double elapsedCrew = System.nanoTime() - startCrew;
            if(Constants.CONSOLE) System.out.println("Exact iteration Crew Over...");
            StaticSharedValues.exactIterations++;
            StaticSharedValues.columnsExactCrewSearch += crewColumns.size();
            StaticSharedValues.timerCrew += elapsedCrew;
            StaticSharedValues.timerExactCrewSearch += elapsedCrew;

            if (!crewColumns.isEmpty()) {
                lp.addColumns(new ArrayList<>(), crewColumns);
                lp.solve();
                return true;
            }
        }

        /*
         * 2. Exact tower pricing.
         */
        {
            if(Constants.CONSOLE) System.out.println("Exact iteration Tower Start...");

            ArrayList<Route> towerColumns;
            double startTower = System.nanoTime();
            if(lp.isFifthThread() || Constants.BRANCH_AND_PRICE) {
                DPT towerDP = new DPT();
                towerDP.run(lp, false);
                towerColumns = towerDP.getRoutes();
            } else {
                TowerPulse pulse = new TowerPulse();
                towerColumns = pulse.run(lp,false);
            }
            if(Constants.CONSOLE) System.out.println("Exact iteration Tower Over...");
            double elapsedTower = System.nanoTime() - startTower;
            StaticSharedValues.timerTower += elapsedTower;
            StaticSharedValues.timerExactTowerSearch += elapsedTower;
            StaticSharedValues.columnsExactTowerSearch += towerColumns.size();

            if (!towerColumns.isEmpty()) {
                lp.addColumns(towerColumns, new ArrayList<>());
                lp.solve();
                return true;
            }
        }

        /*
         * No exact crew column and no exact tower column were found.
         * This is the only normal exact CG convergence condition.
         */
        return false;
    }

    private ArrayList<Route> runGuidedCrewHeuristic(LP lp) throws IloException {

        DyB dp = new DyB();

        double startCrew = System.nanoTime();

        ArrayList<Route> crewColumns = new ArrayList<>();

        for (int k = 0; k < Constants.GUIDED_HEURISTIC_ITERATIONS; k++) {

            int heuristicIteration = k + 1;
            boolean heuristicRedirection = (k % 2 == 1);

            dp.setHeuristicParameters(
                    heuristicIteration,
                    heuristicRedirection,
                    customer2numberOfColumns,
                    customer2pairInColumns,
                    heuristicSet
            );

            dp.run(lp, true, false);
            crewColumns = dp.getRoutes();

            if (!crewColumns.isEmpty()) {
                break;
            }
        }

        double elapsedCrew = System.nanoTime() - startCrew;

        StaticSharedValues.timerRandomizedCrewSearch += elapsedCrew;
        StaticSharedValues.columnsRandomizedCrewSearch += crewColumns.size();
        StaticSharedValues.timerCrew += elapsedCrew;

        return crewColumns;
    }

    private ArrayList<Route> runReducedCrewHeuristic(LP lp) throws IloException {

        DyB dp = new DyB();
        dp.toggleReducedHeuristic(true);

        double startCrew = System.nanoTime();

        ArrayList<Route> crewColumns = new ArrayList<>();

        for (int k = 0; k < 10; k++) {

            dp.setHeuristicParameters(
                    k,
                    false,
                    customer2numberOfColumns,
                    customer2pairInColumns,
                    heuristicSet
            );

            dp.run(lp, true, true);
            crewColumns = dp.getRoutes();

            if (!crewColumns.isEmpty()) {
                break;
            }
        }

        double elapsedCrew = System.nanoTime() - startCrew;

        StaticSharedValues.timerRandomizedCrewSearch += elapsedCrew;
        StaticSharedValues.columnsRandomizedCrewSearch += crewColumns.size();
        StaticSharedValues.timerCrew += elapsedCrew;

        return crewColumns;
    }


}