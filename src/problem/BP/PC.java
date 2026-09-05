package problem.BP;

import ilog.concert.IloException;
import lib.StaticSharedValues;
import lib.TimerHelper;

public class PC {
    public boolean run(LP lp, BNode node, double ub) throws IloException {
//        System.out.println("Price and Cut Procedure");
        lp.construct(node);
        boolean timeout = priceCut(lp,node,ub);
//        System.out.println("_".repeat(50));

        if(timeout || lp.hasActiveSlaks())
            return false;
        return true;
    }

    private boolean priceCut(LP lp, BNode node, double ub) throws IloException {


        TimerHelper timeout = TimerHelper.getInstance();
        CG cg = new CG();
        boolean firstIteration = true;
        boolean restart = false;
        int cuttingIterationCounter = 0;
        double previousValue = 0;
        while (true) {
            if(timeout.hasTimedOut()){
                return true;
            }
            cuttingIterationCounter++;

            if(!lp.solve()) {
                lp.initializeSlacks();
                if(!lp.solve()) {
                    if(BendersCuts.toggle){
                        return false;
                    }
                    return true;
//                    throw new IllegalArgumentException(Msg.infeasibility);
                }
            }



                cg.run(lp);

            if(cuttingIterationCounter > 1){
                double curretnValue = lp.getCost();
                if(curretnValue < previousValue-1e-6) {
                    throw new IllegalArgumentException("this should not happen:" + lp.getCost() + " < " + previousValue);
                }
            }
            previousValue = lp.getCost();




            if(StaticSharedValues.isFirstIteration){
                if(lp.hasActiveSlaks()) {

                    return true;
                }
                StaticSharedValues.isFirstIteration = false;
                StaticSharedValues.lowerBound = lp.getCost();
            }


            firstIteration = false;
            if(lp.getCost() >= ub - 1e-6)
                    return true;

                restart = false;

            if(restart)
                continue;
            else
                break;
        }

        return false;
    }






}
