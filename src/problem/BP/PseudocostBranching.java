package problem.BP;

import ilog.concert.IloException;
import lib.HF;
import problem.graph.Arc;
import problem.graph.Data;

import java.util.ArrayList;

/** Pseudocost branching with exactly two hierarchical strategies: tower arcs, then crew arcs. */
public final class PseudocostBranching {
    private static final double SCORE_EPSILON = 1e-10;
    private final Data data = Data.getInstance();
    private final PseudocostTable pseudocosts;

    public PseudocostBranching(PseudocostTable pseudocosts) {
        this.pseudocosts = pseudocosts;
    }

    public ArrayList<BNode> branch(LP lp, BNode node) throws IloException {
        double[][] towerValues = new double[data.getNodeNumber()][data.getNodeNumber() + 1];
        lp.getMap(towerValues, false);
        ArcCandidate candidate = selectBestArc(node, towerValues, false);
        if (candidate != null) return createChildren(lp, node, candidate);

        double[][] crewValues = new double[data.getTasks()][data.getTasks() + 1];
        lp.getMap(crewValues, true);
        candidate = selectBestArc(node, crewValues, true);
        return candidate == null ? null : createChildren(lp, node, candidate);
    }

    private ArcCandidate selectBestArc(BNode node, double[][] values, boolean crew) {
        ArcCandidate best = null;
        int endDepot = data.getDepotEnd(crew);

        for (int from = 0; from < values.length; from++) {
            for (int to = 1; to < values[from].length; to++) {
                if (from == to || (from == 0 && to == endDepot)) continue;
                if (crew) {
                    if (node.getOuterArcValue(from, to, true) != 0) continue;
                } else if (node.getFeasibleSchedule_arc()[0][from][to] != 0) {
                    continue;
                }

                double value = values[from][to];
                if (value > 1.0 + 1e-6 || !HF.is_fractionalHP(value)) continue;

                int variableId = arcVariableId(crew, from, to);
                double downDelta = value - Math.floor(value);
                double upDelta = Math.ceil(value) - value;
                PseudocostTable.Entry entry = pseudocosts.get(variableId);
                double downGain = entry.downAverage() * downDelta;
                double upGain = entry.upAverage() * upDelta;
                double score = Math.min(downGain, upGain) + 0.1 * Math.max(downGain, upGain);

                if (best == null
                        || score > best.score + SCORE_EPSILON
                        || (Math.abs(score - best.score) <= SCORE_EPSILON
                        && (from < best.from || (from == best.from && to < best.to)))) {
                    best = new ArcCandidate(crew, from, to, value, variableId, score);
                }
            }
        }
        return best;
    }

    private ArrayList<BNode> createChildren(LP lp, BNode node, ArcCandidate candidate)
            throws IloException {
        double parentBound = lp.getCost();
        Arc arc = new Arc(
                candidate.from, candidate.to, candidate.value, candidate.value, null);
        BT brancher = new BT();
        ArrayList<BNode> children = candidate.crew
                ? brancher.branchArcC(lp, node, arc)
                : brancher.branchArcT(lp, node, arc);
        attachArcMetadata(
                children,
                parentBound,
                candidate.variableId,
                candidate.value,
                candidate.crew,
                candidate.from,
                candidate.to);
        return children;
    }

    static int arcVariableId(boolean crew, int from, int to) {
        Data data = Data.getInstance();
        int towerArcCount = data.getNodeNumber() * (data.getNodeNumber() + 1);
        int localId = from * (crew ? data.getTasks() + 1 : data.getNodeNumber() + 1) + to;
        return crew ? towerArcCount + localId : localId;
    }

    static void attachArcMetadata(
            ArrayList<BNode> children,
            double parentBound,
            int variableId,
            double value,
            boolean crew,
            int from,
            int to
    ) {
        for (BNode child : children) {
            boolean up = child.getOuterArcValue(from, to, crew) > 0;
            double delta = up ? Math.ceil(value) - value : value - Math.floor(value);
            child.setPseudocostBranchMetadata(
                    parentBound,
                    variableId,
                    up ? BNode.BranchDirection.UP : BNode.BranchDirection.DOWN,
                    delta);
        }
    }

    private record ArcCandidate(
            boolean crew,
            int from,
            int to,
            double value,
            int variableId,
            double score
    ) { }
}
