package problem.multi;

import java.util.concurrent.atomic.AtomicReference;

public class SharedBounds {
    private final AtomicReference<Double> globalUB = new AtomicReference<>(Double.POSITIVE_INFINITY);
    private final AtomicReference<BestBound> best =
            new AtomicReference<>(new BestBound(Double.POSITIVE_INFINITY, -1));

    public void updateBest(double candidateValue, int candidateID) {
        best.getAndUpdate(current -> {
            if (candidateValue < current.value - 1e-6) {
                return new BestBound(candidateValue, candidateID);
            }
            return current;
        });
    }

    public SharedBounds(double initial_ub) {
        globalUB.getAndAccumulate(initial_ub, Math::min);
    }

    // Update if new bound is better
    public void updateUB(double newUB) {
        globalUB.getAndAccumulate(newUB, Math::min);
    }

    public double getUB() {
        return best.get().value;
    }

    public int getBestID(){
        return best.get().threadId;
    }

    private volatile boolean hasSicendTerminated, hasThiredTerminated, hasFourthTerminated;
    public void setHasFourthThreadTerminated(boolean b) {
        hasFourthTerminated = b;
    }
    public void setHasThirdThreadTerminated(boolean b) {
        hasThiredTerminated = b;
    }

    public void setSecondThreadTerminated(boolean b) {
        hasSicendTerminated = b;
    }
    public boolean hasTerminatedSicend(){
        return hasSicendTerminated;
    }

    public boolean hasTerminatedForuth() {
        return hasFourthTerminated;
    }

    public boolean hasTerminatedThired() {
        return hasThiredTerminated;
    }




}
