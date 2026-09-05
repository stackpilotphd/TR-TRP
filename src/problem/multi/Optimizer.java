package problem.multi;

import ilog.concert.IloException;
import problem.BP.BTree;

import java.io.IOException;

public class Optimizer {
    private final ICEAconstraint extraConstraint;
    private final SharedBounds sharedBounds;

    public Optimizer(ICEAconstraint extraConstraint, SharedBounds sharedBounds) {
        this.extraConstraint = extraConstraint;
        this.sharedBounds = sharedBounds;
    }

    public OptimizationResult solve() throws IloException, InterruptedException, IOException {

        BTree bnBTree = new BTree();
        bnBTree.setSharedBound(sharedBounds);
        bnBTree.setICEAconstraint(extraConstraint);
        bnBTree.setInitialUB(999);
        bnBTree.solve();

        double localBest = bnBTree.getObjValue();
        double rootLB = bnBTree.getRootLB();

        int consID = extraConstraint.id;
        boolean hasTimedOut = bnBTree.hasTimedOut();

        return new OptimizationResult(rootLB,localBest
                ,new SolutionT(localBest,bnBTree.getBestCrew(),bnBTree.getBestTower(),bnBTree.getRouteValues(),bnBTree.getWaits())
                ,bnBTree.getNodeNumber(),bnBTree.getElapsed(),consID,bnBTree.getbestLB(),hasTimedOut);
    }
}
