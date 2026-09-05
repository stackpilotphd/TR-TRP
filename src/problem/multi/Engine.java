package problem.multi;

import ilog.concert.IloException;
import lib.*;
import problem.BP.BTree;
import problem.Constants;
import problem.graph.Data;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

public class Engine {

    private final int numThreads = Constants.THREADS;
    private final LocWriterT localWriter;

    public Engine(LocWriterT localWriter) {
        this.localWriter = localWriter;
    }

    public void execute(double start_time) throws ExecutionException, InterruptedException, IloException, IOException {
        Data data = Data.getInstance();
        int myConsNumber;

        int customerNumber = data.getNodeNumber()-1;
        int lb = data.getTowerNumber();
        int count = 0;
        int ubThreads = Constants.MT_EXTRA_THREAD ? 4 : numThreads;

        while (true){
            if(count >= ubThreads)
                break;
            count++;
            lb++;
            if(lb > customerNumber)
                break;
        }
        myConsNumber = count;

        if(Constants.NO_REPOSITIONING || Constants.BRANCH_AND_PRICE || Constants.SOLVE_FOR_CREWS)
            myConsNumber = 1;

        ParallelOptimizer parallelOptimizer = new ParallelOptimizer(numThreads);
        //-----------------------------------------------------------------------
        double initialUB = 999;
        SolutionT heuristicSolution = null;
        SharedBounds sharedBounds = new SharedBounds(initialUB);
        System.out.println("Initial UB:"+sharedBounds.getUB());
        //-----------------------------------------------------------------------
        //-----------------------------------------------------------------------
        int coverage = data.getTowerNumber();
        int id = 1;
        List<ICEAconstraint> constraints = new ArrayList<>();
        if(Constants.BRANCH_AND_PRICE || Constants.SOLVE_FOR_CREWS){
            constraints.add(new ICEAconstraint(0,customerNumber,1));
        }
        else {
            for (int t = 0; t <myConsNumber; t++) constraints.add(new ICEAconstraint(coverage,coverage++,id++));

            if(!Constants.NO_REPOSITIONING){
                while (coverage <= customerNumber){
                    myConsNumber++;
                    if(myConsNumber>=numThreads) {
                        constraints.add(new ICEAconstraint(coverage,customerNumber,id++));
                        break;
                    } else
                        Msg.incompleteMethod();
                    constraints.add(new ICEAconstraint(coverage,coverage,id++));
                    coverage++;
                }
            }
        }



        for(ICEAconstraint iceAconstraints : constraints)
            System.out.println(iceAconstraints);
        if(constraints.size() != myConsNumber)
            throw new IllegalArgumentException("constraints size and threads numbers do not match");
        //-----------------------------------------------------------------------


        //-----------------------------------------------------------------------
        //RUN THE ICEA ALGORITHM
        List<Future<OptimizationResult>> futures = parallelOptimizer.solveInParallel(constraints,sharedBounds);
        //-----------------------------------------------------------------------

        ArrayList<OptimizationResult> listOfResults = new ArrayList<>();
        for (Future<OptimizationResult> f : futures)
            listOfResults.add(f.get());

        double bestCost = initialUB;
        int best_id = -1;
        SolutionT best_solution = heuristicSolution;
        StringBuilder sb = new StringBuilder();
        boolean hasRunOutOfTime = false;
        for (OptimizationResult result : listOfResults) {
            double temp = result.getBestValue();
            if(temp < bestCost+1e-6){
                bestCost = temp;
                best_solution = result.getBestSolution();
                best_id = result.getID();
            }
            if(result.isTimeOut())
                hasRunOutOfTime = true;
            sb.append("Result " + result.toString()).append(System.lineSeparator());
        }
        if(hasRunOutOfTime){
            System.out.println("THE ALGORITHM HAS RUN OUT OF TIME");
            localWriter.setTimeOut(true);
        } else {
            localWriter.setTimeOut(false);
        }

        if(sharedBounds.getUB() > bestCost+1e-6) {
            if(bestCost < 1e-6)
                bestCost = sharedBounds.getUB();
        }

        boolean isFeasible = true;
        if(!Constants.MT_EXTRA_THREAD){
            if(best_id >= numThreads){
                //CONTINUE RUNNING THE ICEA
                int next_icea_coverage = coverage; //use this if the algorithm has not terminated in 4 iterations
                isFeasible = false;
            }

            //LOWER BOUND FOR THREADS 5...
            if(hasRunOutOfTime || !isFeasible){
                ICEAconstraint cons = new ICEAconstraint(coverage,data.getNodeNumber(),0);
                BTree bnBTree = new BTree();
                bnBTree.setICEAconstraint(cons);
                bnBTree.setExtraBound();
                bnBTree.setSharedBound(sharedBounds);
                bnBTree.setInitialUB(bestCost);
                bnBTree.solve();
                double gbl_lb = bnBTree.getbestLB();
                double loc_best = bnBTree.getObjValue();
                double rootLB = bnBTree.getRootLB();
                int nodes = bnBTree.getNodeNumber();
                double elapsed = bnBTree.getElapsed();
                boolean hasTimeOut = bnBTree.hasTimedOut();
                String str = "Result 0: Value="+loc_best+", LB(termination)="+gbl_lb
                        +", Nodes="+nodes+", Time="+elapsed+"s.";
//            System.out.println(str);
                if(loc_best < bestCost - 1e-6){
                    best_id = 0;
                    bestCost = loc_best;
                }
                sb.append(str).append(System.lineSeparator());

                OptimizationResult e = new OptimizationResult(rootLB,loc_best
                        ,new SolutionT(loc_best,bnBTree.getBestCrew(),bnBTree.getBestTower(),bnBTree.getRouteValues(),bnBTree.getWaits())
                        ,nodes,elapsed,0,gbl_lb,hasTimeOut);
                listOfResults.add(e);
            }
        }





        System.out.print(sb.toString());
        System.out.println("Best Cost at T#" +best_id +":"+bestCost);

        if(!Constants.ROBUST && !Constants.SOLVE_FOR_CREWS && !Constants.FIX_CREW_ROUTES && !Constants.NO_REPOSITIONING
                && !Constants.U_ICEA && !Constants.BRANCH_AND_PRICE && !Constants.COST_OF_PRIORITY
                && !Constants.COST_OF_PRIORITY_TOWER && !Constants.COST_OF_PRIORITY_CREW){
            Path filePath = Paths.get(
                    "summary/multithread/console",
                    LocalWriter.filename
            );

            try {
                // Creates all missing directories; does nothing if they already exist
                Files.createDirectories(filePath.getParent());

                try (BufferedWriter writer = Files.newBufferedWriter(filePath)) {
                    writer.write(sb.toString());
                    writer.newLine();
                    writer.write("Best Cost at " + best_id + ":" + bestCost);
                }

            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        System.out.println("BEST SOLUTION:");
        int numZonesAssignedToTowers = 0;
        if(best_solution!=null) {
            best_solution.print();
            numZonesAssignedToTowers = best_solution.getNumZonesAssignedToTowers();
        }
        parallelOptimizer.shutdown();
        double elapsed = (System.nanoTime() - start_time)*1e-9;
        localWriter.setFeasible(isFeasible);
        localWriter.writeIntermediateResults(bestCost,elapsed,numZonesAssignedToTowers,listOfResults);
        localWriter.writeSolution(best_solution);
    }
}
