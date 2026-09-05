package problem.BP;

import ilog.concert.IloException;
import lib.Msg;
import problem.Constants;

public class bigM {

    public boolean solve(LP lp, int constraintID, int rangeID) throws IloException {
        if(!lp.solve())
            lp.initializeSlacks();
        if(constraintID > -1){
            lp.addSlacks(constraintID,rangeID);
        }
        if(lp.solve()){
            if(Constants.CONSOLE)
                System.out.println("bigM...");
            CG cg = new CG();
            cg.initializeHeuristicStructures();
            while (lp.hasActiveSlaks()){
                boolean ter = cg.h(lp);
                if(!ter) {
                    ter = cg.e(lp);
                    if(!ter){
                        if(Constants.CONSOLE)
                            System.out.println("false...");
                        Msg.todo("250509");
                        break;
                    }
                }
            }
            boolean retValue = !lp.hasActiveSlaks();
            return retValue;
        }
        return false;
    }
}
