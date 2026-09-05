package problem.milp.genericSolverTools;


import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

public class SolverConfig {

    public final String solverName;
    public final int threads;
    public final int timeLimit;
    public final boolean log, console, separateLog, solution;
    public final String logFile, logPath, solPath;


    public SolverConfig(boolean console, String solverName, int threads, int timeLimit, boolean log
            , boolean separateLog, boolean solution, String logFile, String logPath, String solPath) {
        this.console = console;
        this.solverName = solverName;
        this.threads = threads;
        this.timeLimit = timeLimit;
        this.log = log;
        this.separateLog = separateLog;
        this.logFile = logFile;
        this.logPath = logPath;
        this.solPath = solPath;
        this.solution = solution;
    }

    public static SolverConfig load(String file) throws IOException {

        Properties prop = new Properties();

        try (FileInputStream fis = new FileInputStream(file)) {
            prop.load(fis);
        }

        String solverName = prop.getProperty("solver.name");

        int timeLimit = Integer.parseInt(
                prop.getProperty("solver.timelimit", "0"));

        int threads = Integer.parseInt(
                prop.getProperty("solver.threads", "0"));

        boolean log = Boolean.parseBoolean(
                prop.getProperty("solver.log", "true"));

        boolean console = Boolean.parseBoolean(
                prop.getProperty("solver.console", "true"));

        boolean separateLog = Boolean.parseBoolean(
                prop.getProperty("solver.separateLog", "false"));

        boolean solution = Boolean.parseBoolean(
                prop.getProperty("solver.solution", "false"));

        String logFile = prop.getProperty("solver.logfile", "logfile.log");

        String logPath = prop.getProperty("solver.logpath", "");

        String solPath = prop.getProperty("solver.solpath", "");

        return new SolverConfig(
                console,
                solverName,
                threads,
                timeLimit,
                log,
                separateLog,
                solution,
                logFile,
                logPath
                , solPath
        );
    }
}
