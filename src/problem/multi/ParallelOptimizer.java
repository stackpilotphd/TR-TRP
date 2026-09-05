package problem.multi;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class ParallelOptimizer {

    final ExecutorService executor;

    public ParallelOptimizer(int numThreads) {
        this.executor = Executors.newFixedThreadPool(numThreads);
    }

    public List<Future<OptimizationResult>> solveInParallel(List<ICEAconstraint> extraICEAconstraintss, SharedBounds sharedBounds) {
        List<Future<OptimizationResult>> futures = new ArrayList<>();

        for (ICEAconstraint c : extraICEAconstraintss) {
            Callable<OptimizationResult> task = () -> {
                Optimizer opt = new Optimizer(c,sharedBounds);
                return opt.solve();  // returns best solution for this subproblem
            };
            futures.add(executor.submit(task));
        }

        return futures;
    }

    public void shutdown() {
        executor.shutdown();
    }
}
