package problem.BP;

import ilog.concert.IloException;
import lib.*;
import problem.Constants;
import problem.graph.Arc;
import problem.graph.Data;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class BT {

    private final Data data = Data.getInstance();

    private boolean timeOut;

    private String selectedStrongBranchingChildValues;

    private final ArrayList<StrongArcPrintRecord> strongArcPrintRecords = new ArrayList<>();

    private static class StrongArcPrintRecord {
        final Arc arc;
        final String text;

        StrongArcPrintRecord(Arc arc, String text) {
            this.arc = arc;
            this.text = text;
        }
    }

    /*
     * Strong-branching controls.
     */
    private static final double STRONG_BRANCHING_TIME_LIMIT_SECONDS = 60.0;
    private static final int MAX_STRONG_ARC_CANDIDATES = 12;
    private static final int MAX_STRONG_VISIT_CANDIDATES = 12;
    /** Hard deterministic work budget; the 60-second limit remains a safety cutoff. */
    private static final int MAX_STRONG_BRANCH_PROBES = 24;
    private static final double SCORE_TIE_EPSILON = 1e-10;
    private static final double STRONG_TOLERANCE = 0.05;
    private static final int MAX_HEURISTIC_CG_ITERATIONS_IN_STRONG = 5;
    private static final int MAX_EXACT_CG_ITERATIONS_IN_STRONG = 3;
    private static final double STRONG_ACCEPT_DELTA = 0.11;
    private static final double PARENT_COST_TOLERANCE = 1e-6;


    private double INITIAL_PARENT_LP_COST;
    private boolean isThreadFiveOrGreater;
    public ArrayList<BNode> branch(LP lp, BNode node, boolean isThreadFiveOrGreater) throws IloException {

        this.isThreadFiveOrGreater = isThreadFiveOrGreater;


        this.INITIAL_PARENT_LP_COST = lp.getCost();

        if (Constants.BT_CONSOLE) {
            System.out.println("Selecting strategy...");
        }

        BranchChoice choice;
        choice = selectStrategy(lp, node);
        printSelectedStrongBranchingChildValues(choice);




        if (Constants.BT_CONSOLE) {
            System.out.println("Strategy=" + choice.strategy + ";Constructing nodes...");
        }

        if (choice.strategy < 0) {
            throw new IllegalArgumentException(Msg.brErr);
        }

        ArrayList<BNode> nodes;

        switch (choice.strategy) {
            case 2 -> {
                nodes = branchTotalY(lp, node, choice.totalYBranch);
                if (Constants.BT_CONSOLE) {
                    System.out.println("Branching on total tower visits: " + choice.totalYBranch.value);
                }
            }
            case 3 -> {
                nodes = branchCandidate(lp, node, choice.candidate);
                if (Constants.BT_CONSOLE) {
                    System.out.println("Branching on customer " + choice.candidate.i);
                }
            }
            case 4 -> {
                nodes = branchArcT(lp, node, choice.arc);
                if (Constants.BT_CONSOLE) {
                    System.out.println("Branching on tower arc:" + choice.arc.from() + "," + choice.arc.to());
                }
            }
            case 5 -> {
                nodes = branchArcC(lp, node, choice.arc);
                if (Constants.BT_CONSOLE) {
                    System.out.println("Branching on crew arc:" + choice.arc.from() + "," + choice.arc.to());
                }
            }

            default -> throw new IllegalStateException(
                    "Branching failure at node " + node.getIceaID()
                            + ": no fractional visit, tower arc, or crew arc was found."
            );
        }

        return nodes;
    }

    private void printSelectedStrongBranchingChildValues(BranchChoice choice) {
        if (!Constants.BT_CONSOLE) {
            return;
        }

        if (selectedStrongBranchingChildValues == null) {
            System.out.println(
                    "[StrongBranching] No strong-branching child values were recorded for selected strategy="
                            + choice.strategy
                            + ". This may be a fallback branch or totalY branch."
            );
            return;
        }

        System.out.println(selectedStrongBranchingChildValues);
    }

    private void registerStrongArcPrint(Arc arc, String text) {
        if (arc == null || text == null) {
            return;
        }

        strongArcPrintRecords.add(new StrongArcPrintRecord(arc, text));
    }

    private void markSelectedStrongArcPrint(Arc arc) {
        if (arc == null) {
            return;
        }

        for (StrongArcPrintRecord record : strongArcPrintRecords) {
            if (record.arc == arc) {
                selectedStrongBranchingChildValues = record.text;
                return;
            }
        }
    }

    private String formatStrongResult(String label, StrongResult result) {
        if (result == null) {
            return label + " = <not evaluated>";
        }

        if (result.infeasible) {
            return label + " = INFEASIBLE";
        }

        return label
                + " cost=" + result.childCost
                + ", delta=" + result.delta;
    }

    private String formatStrongArcProbe(
            boolean crew,
            Arc arc,
            Integer forcedVisit,
            double parentCost,
            StrongResult minus,
            StrongResult plus,
            StrongResult extra,
            double minimumChange,
            double score
    ) {
        return "[StrongBranching:selected arc candidate]"
                + System.lineSeparator()
                + "  type        = " + (crew ? "crewArc" : "towerArc")
                + System.lineSeparator()
                + "  arc         = (" + arc.from() + "," + arc.to() + ")"
                + System.lineSeparator()
                + "  forcedVisit = " + forcedVisit
                + System.lineSeparator()
                + "  parentCost  = " + parentCost
                + System.lineSeparator()
                + "  " + formatStrongResult("child[-]", minus)
                + System.lineSeparator()
                + "  " + formatStrongResult("child[+]", plus)
                + System.lineSeparator()
                + "  " + formatStrongResult("child[extra]", extra)
                + System.lineSeparator()
                + "  minDelta    = " + minimumChange
                + System.lineSeparator()
                + "  score       = " + score;
    }

    private String formatStrongVisitProbe(
            int i,
            double originalValue,
            double parentCost,
            StrongResult minus,
            StrongResult plus,
            double score
    ) {
        return "[StrongBranching:selected visit candidate]"
                + System.lineSeparator()
                + "  visit       = " + i
                + System.lineSeparator()
                + "  value       = " + originalValue
                + System.lineSeparator()
                + "  parentCost  = " + parentCost
                + System.lineSeparator()
                + "  " + formatStrongResult("child[y=0]", minus)
                + System.lineSeparator()
                + "  " + formatStrongResult("child[y=1]", plus)
                + System.lineSeparator()
                + "  score       = " + score;
    }



    private BranchChoice selectStrategy(LP lp, BNode node) throws IloException {
        startStrongBranchingDeadline();
        timeOut = false;

        selectedStrongBranchingChildValues = null;
        strongArcPrintRecords.clear();

        if(isThreadFiveOrGreater) {
            TotalYBranch totalYBranch = checkTotalY(lp);
            if (totalYBranch != null) {
                return BranchChoice.totalY(totalYBranch);
            }
        }


        Candidate candidate = new Candidate();
        checkVisit(lp, candidate);

        if (candidate.i != -1) {
            return BranchChoice.visit(candidate);
        }



        Arc towerArc = checkArcT(lp, node);
        if (towerArc != null) {
            return BranchChoice.towerArc(towerArc);
        }

        Arc crewArc = checkArcC(lp, node);
        if (crewArc != null) {
            return BranchChoice.crewArc(crewArc);
        }

        throw new IllegalStateException(
                "Branching failure at node " + node.getIceaID()
                        + ": no fractional visit, tower arc, or crew arc was found."
        );
    }

    private static class TowerSizeBranch {
        final int size;
        final double value;
        final int floor;
        final int ceil;

        TowerSizeBranch(int size, double value) {
            this.size = size;
            this.value = value;
            this.floor = (int) Math.floor(value);
            this.ceil = (int) Math.ceil(value);
        }
    }

    private static class CrewSizeBranch {
        final int size;
        final double value;
        final int floor;
        final int ceil;

        CrewSizeBranch(int size, double value) {
            this.size = size;
            this.value = value;
            this.floor = (int) Math.floor(value);
            this.ceil = (int) Math.ceil(value);
        }
    }

    private static class BranchChoice {
        final int strategy;
        final Candidate candidate;
        final Arc arc;
        final TotalYBranch totalYBranch;
        final TowerSizeBranch towerSizeBranch;
        final CrewSizeBranch crewSizeBranch;

        private BranchChoice(
                int strategy,
                Candidate candidate,
                Arc arc,
                TotalYBranch totalYBranch,
                TowerSizeBranch towerSizeBranch,
                CrewSizeBranch crewSizeBranch
        ) {
            this.strategy = strategy;
            this.candidate = candidate;
            this.arc = arc;
            this.totalYBranch = totalYBranch;
            this.towerSizeBranch = towerSizeBranch;
            this.crewSizeBranch = crewSizeBranch;
        }

        static BranchChoice totalY(TotalYBranch totalYBranch) {
            return new BranchChoice(2, null, null, totalYBranch, null, null);
        }

        static BranchChoice visit(Candidate candidate) {
            return new BranchChoice(3, candidate, null, null, null, null);
        }

        static BranchChoice towerArc(Arc arc) {
            return new BranchChoice(4, null, arc, null, null, null);
        }

        static BranchChoice crewArc(Arc arc) {
            return new BranchChoice(5, null, arc, null, null, null);
        }

        static BranchChoice towerSize(TowerSizeBranch towerSizeBranch) {
            return new BranchChoice(6, null, null, null, towerSizeBranch, null);
        }

        static BranchChoice crewSize(CrewSizeBranch crewSizeBranch) {
            return new BranchChoice(8, null, null, null, null, crewSizeBranch);
        }
    }

    private static class TotalYBranch {
        final double value;
        final int floor;
        final int ceil;

        TotalYBranch(double value) {
            this.value = value;
            this.floor = (int) Math.floor(value);
            this.ceil = (int) Math.ceil(value);
        }
    }

    public static class Candidate {
        int i = -1;
        int q;
        double v;
    }

    /* ------------------------------------------------------------------
     * Crew branching.
     * ------------------------------------------------------------------ */

    public Arc checkArcC(LP lp, BNode node) throws IloException {
        if (BendersCuts.toggle) {
            double[][] map = new double[data.getTasks()][data.getTasks() + 1];
            lp.getMap(map, true);

            Arc arc = findBestFractionalArc(
                    node,
                    map,
                    true,
                    1,
                    data.getTasks(),
                    1,
                    data.getTasks(),
                    false
            );

            if (arc != null) {
                return arc;
            }

            arc = findBestFractionalArc(
                    node,
                    map,
                    true,
                    0,
                    1,
                    1,
                    data.getTasks(),
                    false
            );

            if (arc != null) {
                return arc;
            }

            arc = findBestFractionalArc(
                    node,
                    map,
                    true,
                    1,
                    data.getTasks(),
                    data.getDepotEnd(true),
                    data.getDepotEnd(true) + 1,
                    false
            );

            if (arc != null) {
                Msg.incompleteMethod();
                return arc;
            }

            return null;
        }

        return checkCrewArc(lp, node);
    }

    public Arc checkCrewArc(LP lp, BNode node) throws IloException {
        boolean shouldRunStrong =
                Constants.STRONG_BRANCHING;

        double[][] map = new double[data.getTasks()][data.getTasks() + 1];
        lp.getMap(map, true);

        if (shouldRunStrong) {
            timeOut = false;

            double startTimeStrong = System.nanoTime();
            Arc strongArc = checkArc2(startTimeStrong, lp, node, map);
            double end = System.nanoTime();

            StaticSharedValues.strongBranchingCPU += (end - startTimeStrong);

            if (strongArc != null) {
                return strongArc;
            }
        }


        if (Utility.algo == 29) {
            Arc arc = findBestFractionalArc(
                    node,
                    map,
                    true,
                    0,
                    1,
                    1,
                    data.getTasks(),
                    false
            );

            if (arc != null) {
                return arc;
            }

            arc = findBestFractionalArc(
                    node,
                    map,
                    true,
                    1,
                    data.getTasks(),
                    1,
                    data.getTasks(),
                    false
            );

            if (arc != null) {
                return arc;
            }

            return findBestFractionalArc(
                    node,
                    map,
                    true,
                    1,
                    data.getTasks(),
                    data.getDepotEnd(true),
                    data.getDepotEnd(true) + 1,
                    false
            );
        }

        return findBestFractionalArc(
                node,
                map,
                true,
                0,
                data.getTasks(),
                1,
                data.getTasks() + 1,
                false
        );
    }

    private Arc checkArc2(
            double startTimeStrong,
            LP lp,
            BNode node,
            double[][] map
    ) throws IloException {
        ArrayList<Arc> testedArcs = new ArrayList<>(3);
        double parentCost = INITIAL_PARENT_LP_COST;

        Arc startArc = evaluateStrongArcBlock(
                startTimeStrong,
                lp,
                node,
                map,
                true,
                0,
                1,
                1,
                data.getTasks(),
                parentCost,
                false
        );

        if (startArc != null) {
            if (startArc.getMinimumChange() >= STRONG_ACCEPT_DELTA) {
                markSelectedStrongArcPrint(startArc);
                return startArc;
            }
            testedArcs.add(startArc);
        }

        if (!timeOut) {
            Arc endArc = evaluateStrongArcBlock(
                    startTimeStrong,
                    lp,
                    node,
                    map,
                    true,
                    1,
                    data.getTasks(),
                    data.getDepotEnd(true),
                    data.getDepotEnd(true) + 1,
                    parentCost,
                    false
            );

            if (endArc != null) {
                if (endArc.getMinimumChange() >= STRONG_ACCEPT_DELTA) {
                    markSelectedStrongArcPrint(endArc);
                    return endArc;
                }
                testedArcs.add(endArc);
            }
        }

        if (!timeOut) {
            Arc internalArc = evaluateStrongArcBlock(
                    startTimeStrong,
                    lp,
                    node,
                    map,
                    true,
                    1,
                    data.getTasks(),
                    1,
                    data.getTasks(),
                    parentCost,
                    false
            );

            if (internalArc != null) {
                if (internalArc.getMinimumChange() >= STRONG_ACCEPT_DELTA) {
                    markSelectedStrongArcPrint(internalArc);
                    return internalArc;
                }
                testedArcs.add(internalArc);
            }
        }

        return bestByMinimumChange(testedArcs);
    }

    /* ------------------------------------------------------------------
     * Tower arc branching.
     * ------------------------------------------------------------------ */

    public Arc checkArcT(LP lp, BNode node) throws IloException {
        double[][] map = new double[data.getNodeNumber()][data.getNodeNumber() + 1];
        lp.getMap(map, false);

        if (Constants.STRONG_BRANCHING && Utility.algo == 29) {
            return checkArc1(lp, node, map);
        }

        return findBestFractionalArc(
                node,
                map,
                false,
                0,
                data.getNodeNumber(),
                1,
                data.getNodeNumber() + 1,
                true
        );
    }

    private Arc checkArc1(
            LP lp,
            BNode node,
            double[][] map
    ) throws IloException {
        Arc fallback = findBestFractionalArc(
                node,
                map,
                false,
                0,
                data.getNodeNumber(),
                1,
                data.getNodeNumber() + 1,
                true
        );

        if (fallback == null) {
            return null;
        }



        timeOut = false;

        double startTimeStrong = System.nanoTime();
        double parentCost = INITIAL_PARENT_LP_COST;

        ArrayList<Arc> testedArcs = new ArrayList<>(3);

        Arc startArc = evaluateStrongArcBlock(
                startTimeStrong,
                lp,
                node,
                map,
                false,
                0,
                1,
                1,
                data.getNodeNumber(),
                parentCost,
                true
        );

        if (startArc != null) {
            if (startArc.getMinimumChange() >= STRONG_ACCEPT_DELTA) {
                markSelectedStrongArcPrint(startArc);
                return startArc;
            }
            testedArcs.add(startArc);
        }

        if (!timeOut) {
            Arc endArc = evaluateStrongArcBlock(
                    startTimeStrong,
                    lp,
                    node,
                    map,
                    false,
                    1,
                    data.getNodeNumber(),
                    data.getDepotEnd(false),
                    data.getDepotEnd(false) + 1,
                    parentCost,
                    true
            );

            if (endArc != null) {
                if (endArc.getMinimumChange() >= STRONG_ACCEPT_DELTA) {
                    markSelectedStrongArcPrint(endArc);
                    return endArc;
                }
                testedArcs.add(endArc);
            }
        }

        if (!timeOut) {
            Arc internalArc = evaluateStrongArcBlock(
                    startTimeStrong,
                    lp,
                    node,
                    map,
                    false,
                    1,
                    data.getNodeNumber(),
                    1,
                    data.getNodeNumber(),
                    parentCost,
                    true
            );

            if (internalArc != null) {
                if (internalArc.getMinimumChange() >= STRONG_ACCEPT_DELTA) {
                    markSelectedStrongArcPrint(internalArc);
                    return internalArc;
                }
                testedArcs.add(internalArc);
            }
        }

        Arc best = bestByMinimumChange(testedArcs);

        return best != null ? best : fallback;
    }

    /* ------------------------------------------------------------------
     * Shared arc-candidate utilities.
     * ------------------------------------------------------------------ */

    private boolean isAllowedArcCandidate(
            BNode node,
            int i,
            int j,
            boolean crew,
            boolean useFeasibleScheduleArc
    ) {
        if (i == j) {
            return false;
        }

        if (i == 0 && j == data.getDepotEnd(crew)) {
            return false;
        }

        if (useFeasibleScheduleArc) {
            return node.getFeasibleSchedule_arc()[0][i][j] != -1;
        }

        return node.getOuterArcValue(i, j, crew) != -2;
    }

    private Arc findBestFractionalArc(
            BNode node,
            double[][] map,
            boolean crew,
            int lb1,
            int ub1,
            int lb2,
            int ub2,
            boolean useFeasibleScheduleArc
    ) {
        int bestI = -1;
        int bestJ = -1;
        double bestValue = -1.0;
        double bestFracCost = Double.MAX_VALUE;

        for (int i = lb1; i < ub1; i++) {
            for (int j = lb2; j < ub2; j++) {
                if (!isAllowedArcCandidate(node, i, j, crew, useFeasibleScheduleArc)) {
                    continue;
                }

                double v = map[i][j];

                if (v > 1.0 + 1e-6) {
                    continue;
                }

                if (!HF.is_fractionalHP(v)) {
                    continue;
                }

                double fracCost = HF.frac_cost(v);

                if (fracCost < bestFracCost - SCORE_TIE_EPSILON
                        || (Math.abs(fracCost - bestFracCost) <= SCORE_TIE_EPSILON
                        && (bestI < 0 || i < bestI || (i == bestI && j < bestJ)))) {
                    bestFracCost = fracCost;
                    bestI = i;
                    bestJ = j;
                    bestValue = v;
                }
            }
        }

        if (bestI == -1) {
            return null;
        }

        return new Arc(bestI, bestJ, bestValue, bestValue, null);
    }

    private ArrayList<TupleFC> getCandidateTuples(
            BNode node,
            double[][] map,
            boolean crew,
            int lb1,
            int ub1,
            int lb2,
            int ub2,
            double tolerance,
            boolean useFeasibleScheduleArc
    ) {
        ArrayList<TupleFC> all = new ArrayList<>();
        double min = Double.MAX_VALUE;

        for (int i = lb1; i < ub1; i++) {
            for (int j = lb2; j < ub2; j++) {
                if (!isAllowedArcCandidate(node, i, j, crew, useFeasibleScheduleArc)) {
                    continue;
                }

                double v = map[i][j];

                if (v > 1.0 + 1e-6) {
                    continue;
                }

                if (!HF.is_fractionalHP(v)) {
                    continue;
                }

                double fracCost = HF.frac_cost(v);

                TupleFC tuple = new TupleFC(i, j, fracCost);
                tuple.v2 = v;

                all.add(tuple);

                if (fracCost < min) {
                    min = fracCost;
                }
            }
        }

        if (all.isEmpty()) {
            return all;
        }

        double cutoff = min + tolerance;
        ArrayList<TupleFC> selected = new ArrayList<>();

        for (TupleFC tuple : all) {
            if (tuple.v <= cutoff) {
                selected.add(tuple);
            }
        }

        selected.sort(Comparator
                .comparingDouble((TupleFC a) -> a.v)
                .thenComparingInt(a -> a.i)
                .thenComparingInt(a -> a.j));

        return selected;
    }

    private ArrayList<TupleFC> limitTupleCandidates(
            ArrayList<TupleFC> candidates,
            int maxCandidates
    ) {
        if (candidates.size() <= maxCandidates) {
            return candidates;
        }

        return new ArrayList<>(candidates.subList(0, maxCandidates));
    }

    private Arc evaluateStrongArcBlock(
            double startTimeStrong,
            LP lp,
            BNode node,
            double[][] map,
            boolean crew,
            int lb1,
            int ub1,
            int lb2,
            int ub2,
            double parentCost,
            boolean useFeasibleScheduleArc
    ) throws IloException {
        ArrayList<TupleFC> candidates = getCandidateTuples(
                node,
                map,
                crew,
                lb1,
                ub1,
                lb2,
                ub2,
                STRONG_TOLERANCE,
                useFeasibleScheduleArc
        );

        candidates = limitTupleCandidates(candidates, MAX_STRONG_ARC_CANDIDATES);

        if (candidates.isEmpty()) {
            return null;
        }

        return getArcFromCandidateList(
                startTimeStrong,
                lp,
                candidates,
                parentCost,
                crew
        );
    }

    private Arc getArcFromCandidateList(
            double startTimeStrong,
            LP lp,
            ArrayList<TupleFC> candidates,
            double parentCost,
            boolean crew
    ) throws IloException {
        if (candidates.isEmpty()) {
            return null;
        }

        BNode node = lp.getBNode();

        double bestScore = 0.001;
        Arc best = null;
        String bestPrint = null;

        CG cg = new CG();
        cg.initializeHeuristicStructures();

        for (TupleFC candidate : candidates) {
            if (strongBranchingTimeExpired()) {
                timeOut = true;
                return best;
            }


            Arc arc = new Arc(candidate.i, candidate.j, candidate.v2, candidate.v2, null);

            Integer forcedVisit = null;

            if (!crew) {
                forcedVisit = getTowerAlternativeVisitIfApplicable(node, arc);
            }

            int requiredProbes = forcedVisit == null ? 2 : 3;
            if (!reserveStrongBranchingProbes(requiredProbes)) {
                return best;
            }

            StrongResult minus = evaluateArcBranch(
                    lp,
                    cg,
                    arc,
                    0,
                    0,
                    crew,
                    parentCost,
                    forcedVisit
            );


            StrongResult plus = evaluateArcBranch(
                    lp,
                    cg,
                    arc,
                    1,
                    1,
                    crew,
                    parentCost,
                    forcedVisit
            );



            boolean threeChildren = forcedVisit != null;

            StrongResult extra = null;

            if (threeChildren) {
                extra = evaluateVisitBranch(
                        lp,
                        cg,
                        forcedVisit,
                        0,
                        0,
                        parentCost
                );


            }

            if (!threeChildren) {
                if (minus.infeasible && plus.infeasible) {
                    continue;
                }
            } else {
                if (minus.infeasible && plus.infeasible && extra.infeasible) {
                    continue;
                }
            }

            double cMinus = minus.infeasible ? 1.0 : minus.delta;
            double cPlus = plus.infeasible ? 1.0 : plus.delta;

            double score;
            double minimumChange;

            if (threeChildren) {
                double cExtra = extra.infeasible ? 1.0 : extra.delta;
                minimumChange = Math.min(cMinus, Math.min(cPlus, cExtra));
                score = strongBranchingScore(cMinus, cPlus, cExtra);
            } else {
                minimumChange = Math.min(cMinus, cPlus);
                score = strongBranchingScore(cMinus, cPlus);
            }

            arc.setTime(score);
            arc.setMinimumChange(minimumChange);

            if (isBetterArcScore(score, arc, bestScore, best)) {
                bestScore = score;
                best = arc;

                String candidatePrint = formatStrongArcProbe(
                        crew,
                        arc,
                        forcedVisit,
                        parentCost,
                        minus,
                        plus,
                        extra,
                        minimumChange,
                        score
                );
                bestPrint = candidatePrint;
            }
        }

        if (best != null) {
            registerStrongArcPrint(best, bestPrint);
        }
        return best;
    }

    private Arc bestByMinimumChange(ArrayList<Arc> arcs) {
        Arc best = null;
        double bestDelta = -Double.MAX_VALUE;

        for (Arc arc : arcs) {
            if (arc == null) {
                continue;
            }

            double delta = arc.getMinimumChange();

            if (delta > bestDelta + SCORE_TIE_EPSILON
                    || (Math.abs(delta - bestDelta) <= SCORE_TIE_EPSILON
                    && compareArcs(arc, best) < 0)) {
                bestDelta = delta;
                best = arc;
            }
        }
        markSelectedStrongArcPrint(best);
        return best;
    }

    private boolean hasStrongBranchingTimedOut(double startTimeStrong) {
        return (System.nanoTime() - startTimeStrong) * 1e-9
                > STRONG_BRANCHING_TIME_LIMIT_SECONDS;
    }

    private boolean isFreeVisit(BNode node, int i) {
        return node.getPackingLB(i) < 1e-6
                && node.getPackingUB(i) > 1.0 - 1e-6;
    }

    private boolean isTowerCustomerIndex(int i) {
        return i >= 1 && i < data.getNodeNumber();
    }

    private Integer getTowerAlternativeVisitIfApplicable(BNode node, Arc arc) {
        int i = arc.from();
        int j = arc.to();
        int endDepot = data.getDepotEnd(false);

        if (i == 0 && j == endDepot) {
            return null;
        }

        if (i == 0) {
            return isTowerCustomerIndex(j) && isFreeVisit(node, j) ? j : null;
        }

        if (j == endDepot) {
            return isTowerCustomerIndex(i) && isFreeVisit(node, i) ? i : null;
        }

        if (isTowerCustomerIndex(i)
                && isTowerCustomerIndex(j)
                && isFreeVisit(node, i)
                && isFreeVisit(node, j)) {
            return i;
        }

        return null;
    }

    private StrongResult evaluateArcBranch(
            LP lp,
            CG cg,
            Arc arc,
            int lb,
            int ub,
            boolean crew,
            double parentCost,
            Integer forcedVisit
    ) throws IloException {
        if (strongBranchingTimeExpired()) {
            timeOut = true;
            return new StrongResult(0.0, true);
        }

        BNode node = lp.getBNode();

        LP.ArcModificationState arcState = lp.snapshotArcModification(arc, lb, ub, crew);
        lp.modifyArc(arc, lb, ub, crew);

        if (forcedVisit != null) {
            lp.modifyA(forcedVisit, 1, 1);
        }

        try {
            boolean feasible = solveTemporaryStrongBranch(lp, cg);

            if (!feasible) {
                return new StrongResult(0.0, true);
            }


            double childCost = lp.getCost();
            double delta = childCost - parentCost;

            if (delta < -1e-4) {
                logNegativeStrongBranchDelta(
                        lp,
                        delta,
                        parentCost,
                        "ARC "
                                + (crew ? "crew" : "tower")
                                + " "
                                + arc.from()
                                + ","
                                + arc.to()
                                + " -> ["
                                + lb
                                + ","
                                + ub
                                + "]"
                                + (forcedVisit == null ? "" : "; forcedVisit=" + forcedVisit),
                        arc,
                        null,
                        lb,
                        ub,
                        crew,
                        forcedVisit
                );
                delta = 0.0;
            }

            return new StrongResult(Math.max(0.0, delta), false, childCost);
        } finally {
            if (forcedVisit != null) {
                lp.modifyA(
                        forcedVisit,
                        (int) node.getPackingLB(forcedVisit),
                        (int) node.getPackingUB(forcedVisit)
                );
            }

            lp.restoreArcModification(arcState);
        }
    }

    private StrongResult evaluateVisitBranch(
            LP lp,
            CG cg,
            int i,
            int lb,
            int ub,
            double parentCost
    ) throws IloException {
        BNode node = lp.getBNode();

        lp.modifyA(i, lb, ub);

        try {
            boolean feasible = solveTemporaryStrongBranch(lp, cg);

            if (!feasible) {
                return new StrongResult(0.0, true);
            }

            double childCost = lp.getCost();
            double delta = childCost - parentCost;

            if (delta < -1e-4) {
                logNegativeStrongBranchDelta(
                        lp,
                        delta,
                        parentCost,
                        "VISIT " + i + " -> [" + lb + "," + ub + "]",
                        null,
                        i,
                        lb,
                        ub,
                        false,
                        null
                );
                delta = 0.0;
            }

            return new StrongResult(Math.max(0.0, delta), false, childCost);
        } finally {
            lp.modifyA(
                    i,
                    (int) node.getPackingLB(i),
                    (int) node.getPackingUB(i)
            );
        }
    }

    private boolean solveTemporaryStrongBranch(
            LP lp,
            CG cg
    ) throws IloException {
        if (strongBranchingTimeExpired()) {
            timeOut = true;
            return false;
        }

        if (!lp.solve()) {
            StaticSharedValues.strongBranchingSubproblemsSolved++;
            return false;
        }

        if (strongBranchingTimeExpired()) {
            timeOut = true;
            return false;
        }

        int heuristicIter = 0;

        while (!strongBranchingTimeExpired() && cg.h(lp)) {
            heuristicIter++;

            if (heuristicIter >= MAX_HEURISTIC_CG_ITERATIONS_IN_STRONG) {
                break;
            }
        }

        int iter = 0;

        while (!strongBranchingTimeExpired() && cg.e(lp)) {
            iter++;

            if (iter >= MAX_EXACT_CG_ITERATIONS_IN_STRONG) {
                break;
            }
        }

        if (strongBranchingTimeExpired()) {
            timeOut = true;
            return false;
        }

        boolean feasible = lp.solve();
        StaticSharedValues.strongBranchingSubproblemsSolved++;

        return feasible && !lp.hasActiveSlaks();
    }

    private double strongBranchingScore(double... deltas) {
        double min = Double.MAX_VALUE;
        double max = -Double.MAX_VALUE;
        boolean hasNearZero = false;

        for (double delta : deltas) {
            min = Math.min(min, delta);
            max = Math.max(max, delta);

            if (delta < 1e-6) {
                hasNearZero = true;
            }
        }

        if (hasNearZero) {
            return (1.0 - Constants.pseudoCostBranchingCoefficient) * min
                    + Constants.pseudoCostBranchingCoefficient * max;
        }

        return Math.sqrt(min * max);
    }

    private void logNegativeStrongBranchDelta(
            LP lp,
            double delta,
            double parentCost,
            String childDescription,
            Arc arc,
            Integer visitId,
            int lb,
            int ub,
            boolean crew,
            Integer forcedVisit
    ) throws IloException {
        if(true)
            return;
        lp.getBNode().printBranchStrings();

        double childCostThatGaveNegativeDelta = lp.getCost();
        BNode childNode = createDiagnosticChildNode(
                lp,
                childDescription,
                arc,
                visitId,
                lb,
                ub,
                crew,
                forcedVisit
        );

        LP childCheckLp = createDiagnosticLP(lp);
        LP motherCheckLp = createDiagnosticLP(lp);
        boolean childCheckFeasible = false;
        boolean motherCheckFeasible = false;
        double checkCost = Double.NaN;
        double checkCost2 = Double.NaN;
        String childDiagnosticError = null;
        String motherDiagnosticError = null;

        try {
            PC pc = new PC();
            childCheckFeasible = pc.run(childCheckLp, childNode, Double.MAX_VALUE);
            checkCost = childCheckLp.getCost();
        } catch (Exception e) {
            childDiagnosticError = stackTraceString(e);
        }

        try {
            PC pc2 = new PC();
            motherCheckFeasible = pc2.run(motherCheckLp, lp.getBNode(), Double.MAX_VALUE);
            checkCost2 = motherCheckLp.getCost();
        } catch (Exception e) {
            motherDiagnosticError = stackTraceString(e);
        }

        try (FileWriter writer = new FileWriter(LocalWriter.filename + "negative.txt", true)) {
            writer.write("Time: " + LocalDateTime.now() + System.lineSeparator());
            writer.write("child       = " + childDescription + System.lineSeparator());
            writer.write("delta       = " + delta + System.lineSeparator());
            writer.write("parent_cost = " + parentCost + System.lineSeparator());
            writer.write("child_cost  = " + childCostThatGaveNegativeDelta + System.lineSeparator());
            writer.write("node_id     = " + lp.getBNode().getIceaID() + System.lineSeparator());
            writer.write("depth       = " + lp.getBNode().getDepth() + System.lineSeparator());
            writer.write("branch_chain:" + System.lineSeparator());
            BNode cursor = lp.getBNode();
            ArrayList<String> branchStrings = new ArrayList<>();
            while (cursor != null) {
                if (cursor.getBranchStrig() != null) {
                    branchStrings.add(cursor.getBranchStrig());
                }
                cursor = cursor.getFather();
            }
            for (int idx = branchStrings.size() - 1; idx >= 0; idx--) {
                writer.write("  " + branchStrings.get(idx) + System.lineSeparator());
            }
            writer.write("diagnostic_child_branch = " + childNode.getBranchStrig() + System.lineSeparator());
            writer.write("strong_child_solution:" + System.lineSeparator());
            writeDiagnosticSolution(writer, lp);
            writer.write("full_child_check_feasible = " + childCheckFeasible + System.lineSeparator());
            writer.write("full_child_check_cost     = " + checkCost + System.lineSeparator());
            writer.write("full_child_check_delta    = " + (checkCost - parentCost) + System.lineSeparator());
            writer.write("full_child_check_error    = " + childDiagnosticError + System.lineSeparator());
            writer.write("full_child_check_cost <= strong_child_cost = "
                    + (checkCost <= childCostThatGaveNegativeDelta + 1e-6)
                    + System.lineSeparator());
            writeDiagnosticSolution(writer, childCheckLp);
            writer.write("mother_check_feasible = " + motherCheckFeasible + System.lineSeparator());
            writer.write("mother_check_cost     = " + checkCost2 + System.lineSeparator());
            writer.write("mother_check_gap      = " + (checkCost2 - parentCost) + System.lineSeparator());
            writer.write("mother_check_error    = " + motherDiagnosticError + System.lineSeparator());
            writeDiagnosticSolution(writer, motherCheckLp);
            writer.write("----------------------------------------" + System.lineSeparator());
        } catch (IOException e) {
            e.printStackTrace();
        }

        childCheckLp.clear();
        motherCheckLp.clear();
//        System.exit(7);
    }

    private BNode createDiagnosticChildNode(
            LP lp,
            String childDescription,
            Arc arc,
            Integer visitId,
            int lb,
            int ub,
            boolean crew,
            Integer forcedVisit
    ) {
        BNode child = lp.getBNode().copy();

        if (visitId != null) {
            child.setPackingBounds(visitId, lb, ub);
            if (ub < 1e-6) {
                child.addOneMoreBannedVisit();
            } else {
                child.setPositive(true);
            }
        } else {
            if (forcedVisit != null) {
                child.setPackingBounds(forcedVisit, 1, 1);
            }

            child.setFeasibleArcValue(arc, lb, ub, crew);

            if (ub > 1e-6) {
                child.setPositive(true);
            }
        }

        child.setInitialSolution(lp, lp.getReliefRoutes(), lp.getCrewRoutes());
        child.setBranchStrig("DBG " + childDescription);

        return child;
    }

    private LP createDiagnosticLP(LP source) {
        LP result = new LP();
        result.setICEAconsraint(source.getIceaConstrain());
        result.setSharedBound(source.getSharedBound());
        result.setExtraBound(source.isExtraBound());
        return result;
    }

    private void writeDiagnosticSolution(
            FileWriter writer,
            LP lp
    ) throws IOException {
        writer.write("  cost = " + lp.getCost() + System.lineSeparator());
        writer.write("  active_tower_routes:" + System.lineSeparator());
        if (lp.getActive_Rset() == null) {
            writer.write("    <null>" + System.lineSeparator());
        } else {
            for (VarR var : lp.getActive_Rset()) {
                writeDiagnosticRoute(writer, var);
            }
        }
        writer.write("  active_crew_routes:" + System.lineSeparator());
        if (lp.getActive_Cset() == null) {
            writer.write("    <null>" + System.lineSeparator());
        } else {
            for (VarR var : lp.getActive_Cset()) {
                writeDiagnosticRoute(writer, var);
            }
        }
    }

    private void writeDiagnosticRoute(
            FileWriter writer,
            VarR var
    ) throws IOException {
        Route route = var.getRoute();
        if (route == null) {
            writer.write("    v=" + var.getValue()
                    + "; cost=" + var.getCost()
                    + "; route=<null>"
                    + System.lineSeparator());
            return;
        }

        writer.write("    v=" + var.getValue()
                + "; cost=" + var.getCost()
                + "; route=" + route.getPattern()
                + "; wait=" + route.getWaitBooleans()
                + "; arrivals=" + route.getArrivals()
                + System.lineSeparator());
    }

    private String stackTraceString(Exception e) {
        StringWriter stringWriter = new StringWriter();
        e.printStackTrace(new PrintWriter(stringWriter));
        return stringWriter.toString();
    }

    private static class StrongResult {
        final double delta;
        final boolean infeasible;
        final double childCost;

        StrongResult(double delta, boolean infeasible) {
            this(delta, infeasible, Double.NaN);
        }

        StrongResult(double delta, boolean infeasible, double childCost) {
            this.delta = delta;
            this.infeasible = infeasible;
            this.childCost = childCost;
        }
    }

    private static class TupleFC {
        int i;
        int j;
        double v;
        double v2;

        TupleFC(int i, int j, double v) {
            this.i = i;
            this.j = j;
            this.v = v;
        }
    }

    private static class VisitTuple {
        int id;
        double v;
        double originalValue;

        VisitTuple(int id, double v, double originalValue) {
            this.id = id;
            this.v = v;
            this.originalValue = originalValue;
        }
    }

    /* ------------------------------------------------------------------
     * Visit branching.
     * ------------------------------------------------------------------ */

    public void checkVisit(LP lp, Candidate cand) throws IloException {
        cand.i = -1;

        BNode node = lp.getBNode();

        this.timeOut = false;

        ArrayList<VisitTuple> fractionalVisits = new ArrayList<>();
        VisitTuple bestFallback = null;

        for (int i = 1; i < data.getNodeNumber(); i++) {

            if (!isFreeVisit(node, i)) {
                continue;
            }

            double v = lp.getYValue(i);

            if (!HF.is_fractionalHP(v)) {
                continue;
            }

            double fracCost = HF.frac_cost(v);
            VisitTuple tuple = new VisitTuple(i, fracCost, v);
            fractionalVisits.add(tuple);

            if (bestFallback == null
                    || fracCost < bestFallback.v - SCORE_TIE_EPSILON
                    || (Math.abs(fracCost - bestFallback.v) <= SCORE_TIE_EPSILON
                    && i < bestFallback.id)) {
                bestFallback = tuple;
            }
        }

        if (bestFallback == null) {
            return;
        }

        if (Constants.STRONG_BRANCHING) {
            double startTimeStrong = System.nanoTime();

            VisitTuple strongTuple = checkVisitStrong(
                    startTimeStrong,
                    fractionalVisits,
                    lp
            );

            if (strongTuple != null) {
                cand.i = strongTuple.id;
                cand.v = strongTuple.originalValue;
                return;
            }
        }

        cand.i = bestFallback.id;
        cand.v = bestFallback.originalValue;
    }

    private VisitTuple checkVisitStrong(
            double startTimeStrong,
            ArrayList<VisitTuple> candidates,
            LP lp
    ) throws IloException {
        if (candidates.isEmpty()) {
            return null;
        }

        candidates.sort(Comparator
                .comparingDouble((VisitTuple a) -> a.v)
                .thenComparingInt(a -> a.id));

        if (candidates.size() > MAX_STRONG_VISIT_CANDIDATES) {
            candidates = new ArrayList<>(
                    candidates.subList(0, MAX_STRONG_VISIT_CANDIDATES)
            );
        }

        double parentCost = INITIAL_PARENT_LP_COST;
        double bestScore = 0.001;
        VisitTuple best = null;
        String bestPrint = null;

        CG cg = new CG();
        cg.initializeHeuristicStructures();

        for (VisitTuple candidate : candidates) {
            if (strongBranchingTimeExpired()) {
                timeOut = true;
                return best;
            }

            if (!reserveStrongBranchingProbes(2)) {
                return best;
            }

            int i = candidate.id;


            StrongResult minus = evaluateVisitBranch(
                    lp,
                    cg,
                    i,
                    0,
                    0,
                    parentCost
            );



            StrongResult plus = evaluateVisitBranch(
                    lp,
                    cg,
                    i,
                    1,
                    1,
                    parentCost
            );


            if (minus.infeasible && plus.infeasible) {
                continue;
            }

            double cMinus = minus.infeasible ? 1.0 : minus.delta;
            double cPlus = plus.infeasible ? 1.0 : plus.delta;

            double score = strongBranchingScore(cMinus, cPlus);

            if (score > bestScore + SCORE_TIE_EPSILON
                    || (Math.abs(score - bestScore) <= SCORE_TIE_EPSILON
                    && (best == null || candidate.id < best.id))) {
                String candidatePrint = formatStrongVisitProbe(
                        i,
                        candidate.originalValue,
                        parentCost,
                        minus,
                        plus,
                        score
                );

                bestScore = score;
                best = candidate;
                bestPrint = candidatePrint;
            }
        }
        selectedStrongBranchingChildValues = bestPrint;
        return best;
    }

    /* ------------------------------------------------------------------
     * Branch construction.
     * ------------------------------------------------------------------ */




    private boolean isIntegerFractional(double v) {
        return Math.abs(v - Math.rint(v)) > 1e-6;
    }

    private TotalYBranch checkTotalY(LP lp) {
        double totalY = lp.getTotalActiveY();

        if (!HF.is_fractionalHP(totalY)) {
            return null;
        }

        return new TotalYBranch(totalY);
    }

    private ArrayList<BNode> branchTotalY(
            LP lp,
            BNode node,
            TotalYBranch branch
    ) {
        ArrayList<BNode> nodes = new ArrayList<>();
        int[] parentBounds = node.getTotsBounds();
        int parentLb = parentBounds == null ? 0 : parentBounds[0];
        int parentUb = parentBounds == null ? data.getNodeNumber() : parentBounds[1];

        addTotalYChild(
                nodes,
                node,
                lp,
                parentLb,
                Math.min(parentUb, branch.floor),
                "TY<= " + branch.floor
        );
        addTotalYChild(
                nodes,
                node,
                lp,
                Math.max(parentLb, branch.ceil),
                parentUb,
                "TY>= " + branch.ceil
        );

        return nodes;
    }

    private void addTotalYChild(
            ArrayList<BNode> nodes,
            BNode node,
            LP lp,
            int lb,
            int ub,
            String branchString
    ) {
        if (lb > ub) {
            return;
        }

        BNode child = node.copy();
        child.setTotBound(lb, ub);
        child.setInitialSolution(lp, lp.getReliefRoutes(), lp.getCrewRoutes());
        child.setBranchStrig(branchString);
        nodes.add(child);
    }

    public ArrayList<BNode> branchCandidate(
            LP lp,
            BNode node,
            Candidate cand
    ) throws IloException {

        if (!isFreeVisit(node, cand.i)) {
            throw new IllegalStateException(
                    "Trying to branch on already fixed visit T"
                            + cand.i
                            + ". Bounds are ["
                            + node.getPackingLB(cand.i)
                            + ","
                            + node.getPackingUB(cand.i)
                            + "]. Branch string="
                            + node.getBranchStrig()
            );
        }

        ArrayList<BNode> nodes = new ArrayList<>();

        modifyCandidate(nodes, node, lp, cand.i, 0, 0);
        modifyCandidate(nodes, node, lp, cand.i, 1, 1);

        return nodes;
    }

    protected void modifyCandidate(
            ArrayList<BNode> nodes,
            BNode node,
            LP lp,
            int i,
            int lb,
            int ub
    ) throws IloException {
        lp.modifyA(i, lb, ub);

        try {
            boolean hasSolved = solveChildRelaxation(lp);

            BNode child = node.copy();
            child.setPackingBounds(i, lb, ub);

            if (hasSolved) {
                child.setInitialSolution(lp, lp.getReliefRoutes(), lp.getCrewRoutes());
            }

            nodes.add(child);

            String string = "T" + i + ":";

            if (ub < 1e-6) {
                string += "-1";
                child.addOneMoreBannedVisit();
            } else {
                string += "+1";
                child.setPositive(true);
            }

            child.setBranchStrig(string);
        } finally {
            lp.modifyA(
                    i,
                    (int) node.getPackingLB(i),
                    (int) node.getPackingUB(i)
            );
        }
    }

    public ArrayList<BNode> branchArcC(
            LP lp,
            BNode node,
            Arc arc
    ) throws IloException {
        ArrayList<BNode> nodes = new ArrayList<>();

        modifyArcC(nodes, node, lp, arc, 0, 0);
        modifyArcC(nodes, node, lp, arc, 1, 1);

        return nodes;
    }

    private void modifyArcC(
            ArrayList<BNode> nodes,
            BNode node,
            LP lp,
            Arc arc,
            int lb,
            int ub
    ) throws IloException {
        LP.ArcModificationState arcState = lp.snapshotArcModification(arc, lb, ub, true);
        lp.modifyArc(arc, lb, ub, true);

        try {
            boolean hasSolved = solveChildRelaxation(lp);

            BNode child = node.copy();
            child.setFeasibleArcValue(arc, lb, ub, true);

            if (hasSolved) {
                child.setInitialSolution(lp, lp.getReliefRoutes(), lp.getCrewRoutes());
            }

            nodes.add(child);

            String string = "CA" + arc.from() + "," + arc.to() + ":";

            if (ub < 1e-6) {
                string += "-1";
            } else {
                string += "+1";
                child.setPositive(true);
            }

            child.setBranchStrig(string);
        } finally {
            lp.restoreArcModification(arcState);
        }
    }

    public ArrayList<BNode> branchArcT(
            LP lp,
            BNode node,
            Arc arc
    ) throws IloException {
        Integer alternativeVisit = getTowerAlternativeVisitIfApplicable(node, arc);

        if (alternativeVisit != null) {
            return branchAlternativeArc(lp, node, alternativeVisit, arc);
        }

        ArrayList<BNode> nodes = new ArrayList<>();

        modifyArcT(nodes, node, lp, arc, 0, 0);
        modifyArcT(nodes, node, lp, arc, 1, 1);

        return nodes;
    }

    private void modifyArcT(
            ArrayList<BNode> nodes,
            BNode node,
            LP lp,
            Arc arc,
            int lb,
            int ub
    ) throws IloException {
        LP.ArcModificationState arcState = lp.snapshotArcModification(arc, lb, ub, false);
        lp.modifyArc(arc, lb, ub, false);

        try {
            boolean hasSolved = solveChildRelaxation(lp);

            BNode child = node.copy();
            child.setFeasibleArcValue(arc, lb, ub, false);

            if (hasSolved) {
                child.setInitialSolution(lp, lp.getReliefRoutes(), lp.getCrewRoutes());
            }

            nodes.add(child);

            String string = "TA" + arc.from() + "," + arc.to() + ":";

            if (ub < 1e-6) {
                string += "-1";
            } else {
                string += "+1";
                child.setPositive(true);
            }

            child.setBranchStrig(string);
        } finally {
            lp.restoreArcModification(arcState);
        }
    }

    private ArrayList<BNode> branchAlternativeArc(
            LP lp,
            BNode node,
            int yID,
            Arc arc
    ) throws IloException {
        ArrayList<BNode> nodes = new ArrayList<>();

        /*
         * Boussier, Feillet, and Gendreau style three-child branching:
         * 1. yID = 0
         * 2. yID = 1 and arc = 0
         * 3. yID = 1 and arc = 1
         */
        modifyCandidate(nodes, node, lp, yID, 0, 0);
        modifyArcAlternative(nodes, node, lp, yID, arc, 0, 0);
        modifyArcAlternative(nodes, node, lp, yID, arc, 1, 1);

        return nodes;
    }

    private void modifyArcAlternative(
            ArrayList<BNode> nodes,
            BNode node,
            LP lp,
            int yID,
            Arc arc,
            int lb,
            int ub
    ) throws IloException {
        LP.ArcModificationState arcState = lp.snapshotArcModification(arc, lb, ub, false);
        lp.modifyA(yID, 1, 1);
        lp.modifyArc(arc, lb, ub, false);

        try {
            boolean hasSolved = solveChildRelaxation(lp);

            BNode child = node.copy();
            child.setPackingBounds(yID, 1, 1);
            child.setFeasibleArcValue(arc, lb, ub, false);

            if (hasSolved) {
                child.setInitialSolution(lp, lp.getReliefRoutes(), lp.getCrewRoutes());
            }

            nodes.add(child);

            String string = "TA" + arc.from() + "," + arc.to() + "(y" + yID + "):";

            if (ub < 1e-6) {
                string += "-1";
            } else {
                string += "+1";
                child.setPositive(true);
            }

            child.setBranchStrig(string);
        } finally {
            lp.restoreArcModification(arcState);
            lp.modifyA(
                    yID,
                    (int) node.getPackingLB(yID),
                    (int) node.getPackingUB(yID)
            );
        }
    }

    private boolean solveChildRelaxation(LP lp) throws IloException {
        boolean hasSolved = false;

        if (lp.solve()) {
            if (!lp.hasActiveSlaks()) {
                hasSolved = true;
            }
        }

        if (!hasSolved) {
            hasSolved = solve(lp, -1, 0);
        }

        return hasSolved;
    }

    /* ------------------------------------------------------------------
     * Big-M / slack recovery solve.
     * ------------------------------------------------------------------ */

    public boolean solve(
            LP lp,
            int constraintID,
            int rangeID
    ) throws IloException {
        if (!lp.solve()) {
            lp.initializeSlacks();
        }

        if (constraintID > -1) {
            lp.addSlacks(constraintID, rangeID);
        }

        if (lp.solve()) {
            if (Constants.BT_CONSOLE) {
                System.out.println("bigM...");
            }

            CG cg = new CG();
            cg.initializeHeuristicStructures();

            while (lp.hasActiveSlaks()) {
                boolean terminated = cg.h(lp);

                if (!terminated) {
                    terminated = cg.e(lp);

                    if (!terminated) {
                        if (Constants.BT_CONSOLE) {
                            System.out.println("false...");
                        }
                        break;
                    }
                }
            }

            return !lp.hasActiveSlaks();
        }

        return false;
    }

    private long strongBranchingDeadlineNanos;
    private int remainingStrongBranchingProbes;

    private void startStrongBranchingDeadline() {
        remainingStrongBranchingProbes = MAX_STRONG_BRANCH_PROBES;
        strongBranchingDeadlineNanos =
                System.nanoTime()
                        + (long) (STRONG_BRANCHING_TIME_LIMIT_SECONDS * 1_000_000_000L);
    }

    private boolean strongBranchingTimeExpired() {
        return System.nanoTime() >= strongBranchingDeadlineNanos;
    }

    private boolean reserveStrongBranchingProbes(int probes) {
        if (remainingStrongBranchingProbes < probes) {
            return false;
        }
        remainingStrongBranchingProbes -= probes;
        return true;
    }

    private boolean isBetterArcScore(double score, Arc arc, double bestScore, Arc best) {
        return score > bestScore + SCORE_TIE_EPSILON
                || (Math.abs(score - bestScore) <= SCORE_TIE_EPSILON
                && compareArcs(arc, best) < 0);
    }

    private int compareArcs(Arc left, Arc right) {
        if (right == null) return -1;
        int fromComparison = Integer.compare(left.from(), right.from());
        return fromComparison != 0
                ? fromComparison
                : Integer.compare(left.to(), right.to());
    }


    private static final String DEFAULT_BRANCHING_DEBUG_FILE = "branchingDebug.txt";

    private static final Pattern DEBUG_LINE_PATTERN = Pattern.compile(
            "^\\s*Br\\((\\d+)\\)\\s*:\\s*([^:]+)\\s*:\\s*([+-]1)\\s*$"
    );

    private static final Pattern VISIT_DEBUG_PATTERN = Pattern.compile(
            "^T(\\d+)$"
    );

    private static final Pattern CREW_ARC_DEBUG_PATTERN = Pattern.compile(
            "^CA(\\d+),(\\d+)$"
    );

    private static final Pattern TOWER_ARC_DEBUG_PATTERN = Pattern.compile(
            "^TA(\\d+),(\\d+)(?:\\(y(\\d+)\\))?$"
    );




    public void imposeFullBranchingDebugPath(BNode node) throws IOException {
        imposeBranchingDebugUpToDepth(node, Integer.MAX_VALUE);
    }


    public void imposeBranchingDebugUpToDepth(BNode node, int maxDepth) throws IOException {
        Path debugFile = Paths.get(System.getProperty("user.dir"), DEFAULT_BRANCHING_DEBUG_FILE);
        imposeBranchingDebugUpToDepth(node, debugFile, maxDepth);
    }

    public boolean imposeBranchingDebugAtDepth(BNode node, int targetDepth) throws IOException {
        Path debugFile = Paths.get(System.getProperty("user.dir"), DEFAULT_BRANCHING_DEBUG_FILE);
        return imposeBranchingDebugAtDepth(node, debugFile, targetDepth);
    }

    public void imposeBranchingDebugUpToDepth(
            BNode node,
            Path debugFile,
            int maxDepth
    ) throws IOException {
        ArrayList<DebugBranchDecision> decisions = readBranchingDebugFile(debugFile);

        decisions.sort(Comparator.comparingInt(d -> d.depth));

        DebugBranchDecision lastApplied = null;

        for (DebugBranchDecision decision : decisions) {
            if (decision.depth > maxDepth) {
                continue;
            }

            imposeDebugDecision(node, decision);
            lastApplied = decision;
        }

        if (lastApplied != null) {
            node.setBranchStrig(lastApplied.toBranchString());
        }
    }

    public boolean imposeBranchingDebugAtDepth(
            BNode node,
            Path debugFile,
            int targetDepth
    ) throws IOException {
        ArrayList<DebugBranchDecision> decisions = readBranchingDebugFile(debugFile);

        DebugBranchDecision selected = null;

        for (DebugBranchDecision decision : decisions) {
            if (decision.depth == targetDepth) {
                if (selected != null) {
                    throw new IllegalArgumentException(
                            "branchingDebug.txt contains more than one decision at depth "
                                    + targetDepth
                    );
                }

                selected = decision;
            }
        }

        if (selected == null) {
            return false;
        }

        imposeDebugDecision(node, selected);
        node.setBranchStrig(selected.toBranchString());

        return true;
    }

    private ArrayList<DebugBranchDecision> readBranchingDebugFile(Path debugFile) throws IOException {
        if (!Files.exists(debugFile)) {
            throw new IOException("Branching debug file does not exist: " + debugFile.toAbsolutePath());
        }

        List<String> lines = Files.readAllLines(debugFile);
        ArrayList<DebugBranchDecision> decisions = new ArrayList<>();

        for (int lineIndex = 0; lineIndex < lines.size(); lineIndex++) {
            String line = lines.get(lineIndex);

            if (line == null || line.trim().isEmpty()) {
                continue;
            }

            decisions.add(parseBranchingDebugLine(line, lineIndex + 1));
        }

        return decisions;
    }

    private DebugBranchDecision parseBranchingDebugLine(String line, int lineNumber) {
        Matcher lineMatcher = DEBUG_LINE_PATTERN.matcher(line);

        if (!lineMatcher.matches()) {
            throw new IllegalArgumentException(
                    "Invalid branching-debug line " + lineNumber + ": " + line
                            + System.lineSeparator()
                            + "Expected format: Br(depth):Decision:+1 or Br(depth):Decision:-1"
            );
        }

        int depth = Integer.parseInt(lineMatcher.group(1));
        String decisionName = lineMatcher.group(2).replaceAll("\\s+", "");
        int fixedValue = parseDebugFixedValue(lineMatcher.group(3));

        Matcher visitMatcher = VISIT_DEBUG_PATTERN.matcher(decisionName);

        if (visitMatcher.matches()) {
            int visitId = Integer.parseInt(visitMatcher.group(1));

            return DebugBranchDecision.visit(
                    depth,
                    visitId,
                    fixedValue
            );
        }

        Matcher crewArcMatcher = CREW_ARC_DEBUG_PATTERN.matcher(decisionName);

        if (crewArcMatcher.matches()) {
            int from = Integer.parseInt(crewArcMatcher.group(1));
            int to = Integer.parseInt(crewArcMatcher.group(2));

            return DebugBranchDecision.arc(
                    depth,
                    DebugBranchType.CREW_ARC,
                    from,
                    to,
                    null,
                    fixedValue
            );
        }

        Matcher towerArcMatcher = TOWER_ARC_DEBUG_PATTERN.matcher(decisionName);

        if (towerArcMatcher.matches()) {
            int from = Integer.parseInt(towerArcMatcher.group(1));
            int to = Integer.parseInt(towerArcMatcher.group(2));

            Integer alternativeVisitId = null;

            if (towerArcMatcher.group(3) != null) {
                alternativeVisitId = Integer.parseInt(towerArcMatcher.group(3));
            }

            return DebugBranchDecision.arc(
                    depth,
                    DebugBranchType.TOWER_ARC,
                    from,
                    to,
                    alternativeVisitId,
                    fixedValue
            );
        }

        throw new IllegalArgumentException(
                "Unknown branching decision on line " + lineNumber + ": " + decisionName
                        + System.lineSeparator()
                        + "Supported decision names: T<i>, CA<i>,<j>, TA<i>,<j>, TA<i>,<j>(y<k>)"
        );
    }

    private int parseDebugFixedValue(String sign) {
        if ("+1".equals(sign)) {
            return 1;
        }

        if ("-1".equals(sign)) {
            return 0;
        }

        throw new IllegalArgumentException("Unsupported branching value: " + sign);
    }

    private void imposeDebugDecision(BNode node, DebugBranchDecision decision) {
        switch (decision.type) {
            case VISIT -> imposeDebugVisitDecision(
                    node,
                    decision.visitId,
                    decision.fixedValue
            );

            case CREW_ARC -> imposeDebugArcDecision(
                    node,
                    decision.from,
                    decision.to,
                    decision.fixedValue,
                    true
            );

            case TOWER_ARC -> {
                if (decision.alternativeVisitId != null) {
                    imposeDebugVisitDecision(node, decision.alternativeVisitId, 1);
                }

                imposeDebugArcDecision(
                        node,
                        decision.from,
                        decision.to,
                        decision.fixedValue,
                        false
                );
            }

            default -> throw new IllegalStateException("Unsupported debug branch type: " + decision.type);
        }
    }

    private void imposeDebugVisitDecision(
            BNode node,
            int visitId,
            int fixedValue
    ) {
        validateBinaryValue(fixedValue);


        boolean wasAlreadyFixedToZero =
                node.getPackingLB(visitId) < 1e-6
                        && node.getPackingUB(visitId) < 1e-6;

        node.setPackingBounds(visitId, fixedValue, fixedValue);

        if (fixedValue == 0) {
            if (!wasAlreadyFixedToZero) {
                node.addOneMoreBannedVisit();
            }
        } else {
            node.setPositive(true);
        }
    }

    private void imposeDebugArcDecision(
            BNode node,
            int from,
            int to,
            int fixedValue,
            boolean crew
    ) {
        validateBinaryValue(fixedValue);

        Arc arc = new Arc(from, to, fixedValue, fixedValue, null);

        node.setFeasibleArcValue(
                arc,
                fixedValue,
                fixedValue,
                crew
        );

        if (fixedValue == 1) {
            node.setPositive(true);
        }
    }

    private void validateBinaryValue(int fixedValue) {
        if (fixedValue != 0 && fixedValue != 1) {
            throw new IllegalArgumentException(
                    "Debug branching value must be 0 or 1, but got " + fixedValue
            );
        }
    }

    private enum DebugBranchType {
        VISIT,
        CREW_ARC,
        TOWER_ARC
    }

    private static class DebugBranchDecision {
        final int depth;
        final DebugBranchType type;

        final int visitId;

        final int from;
        final int to;
        final Integer alternativeVisitId;

        final int fixedValue;

        private DebugBranchDecision(
                int depth,
                DebugBranchType type,
                int visitId,
                int from,
                int to,
                Integer alternativeVisitId,
                int fixedValue
        ) {
            this.depth = depth;
            this.type = type;
            this.visitId = visitId;
            this.from = from;
            this.to = to;
            this.alternativeVisitId = alternativeVisitId;
            this.fixedValue = fixedValue;
        }

        static DebugBranchDecision visit(
                int depth,
                int visitId,
                int fixedValue
        ) {
            return new DebugBranchDecision(
                    depth,
                    DebugBranchType.VISIT,
                    visitId,
                    -1,
                    -1,
                    null,
                    fixedValue
            );
        }

        static DebugBranchDecision arc(
                int depth,
                DebugBranchType type,
                int from,
                int to,
                Integer alternativeVisitId,
                int fixedValue
        ) {
            return new DebugBranchDecision(
                    depth,
                    type,
                    -1,
                    from,
                    to,
                    alternativeVisitId,
                    fixedValue
            );
        }

        String toBranchString() {
            String suffix = fixedValue == 1 ? "+1" : "-1";

            return switch (type) {
                case VISIT -> "T" + visitId + ":" + suffix;

                case CREW_ARC -> "CA" + from + "," + to + ":" + suffix;

                case TOWER_ARC -> {
                    String result = "TA" + from + "," + to;

                    if (alternativeVisitId != null) {
                        result += "(y" + alternativeVisitId + ")";
                    }

                    yield result + ":" + suffix;
                }
            };
        }
    }



}
