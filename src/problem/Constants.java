package problem;

public class Constants {
    public static final double CREW_WORK_LENGTH = 600;
    public static final int MAX_DEPTH = 25;
    public static final double M = 1e6;
    public static final boolean DEPTH_FIRST_SEARCH = false;
    public static final boolean NON_ELEMENTARY_CREW = false;
    public static final boolean NON_ELEMENTARY_TOWER = true;
    public static final boolean IMPOSE_MAXIMUM_TOWER_ROUTE_LENGTH = true;
    public static final boolean TWO_CYCLE_ELEMINATION = false;
    public static final double DECIMALS = 6;
    public static final double TOWER_REDUCED_ACCEPTANCE_THRESHOLD = 1e-5;
    public static boolean WRITE_FOR_CREWS = false;
    public static final int DELTA_PULSE = 15; //
    public static final int LOWER_BOUND_PULSE = 300; //
    public static final int MAX_COL = 50;
    public static final boolean KEEP_INTEGER_Ys = true;
    public static final double pseudoCostBranchingCoefficient = 1. /6.;
    public static final boolean CONSOLE = false;
    public static boolean BT_CONSOLE = false;
    public static final boolean RUN_COMPUTATIONAL_STUDY = true;
    public static boolean DETERMINE_MINIMUM_NUMBER_CREWS = false;
    public static boolean RELAX_MIP = false;
    public static boolean NO_REPOSITIONING = false;
    public static final boolean ACTIVE_ACCELERATION = true;
    public static final boolean SHOULD_RUN_MIP_HEURISTIC = ACTIVE_ACCELERATION;
    public static boolean USE_NODE_ELIMINATION_CREW_PULSE = ACTIVE_ACCELERATION;
    public static final boolean SERVICE_BOUND_CUT = ACTIVE_ACCELERATION;
    public static final boolean STRONG_BRANCHING = ACTIVE_ACCELERATION;
    public static final boolean GUIDED_PRICING_HEURISTIC = ACTIVE_ACCELERATION;
    public static final int GUIDED_HEURISTIC_ITERATIONS = 10;
    public static boolean SHOULD_RUN_BENDERS_HEURISTIC = false;
    public static final double BENDERS_TOTAL_TILIM = 300.;
    public static final double BENDERS_SINGLE_ITERATION_TILIM = 60.;
    public static final int BENDERS_MAX_DEPTH = 10;
    public static final boolean SHOULD_WRITE_SOLUTION = true;
    public static final int THREADS = 5;
    public static boolean MT_EXTRA_THREAD = true;
    public static boolean SOLVE_FOR_CREWS;
    public static boolean FIX_CREW_ROUTES =false ;
    public static boolean FIX_CREW_AND_TOWER_ROUTES = false;
    public static boolean BRANCH_AND_PRICE = false;
    public static boolean ROBUST = false;
    public static final int R_TOGGLE = 0;
    public static int BUDGET = 5;
    public static double ALPHA = 0.35;
    public static boolean COST_OF_PRIORITY = false;
    public static boolean COST_OF_PRIORITY_CREW = false;
    public static boolean COST_OF_PRIORITY_TOWER = false;
    public static boolean UPPER_BOUNDING_HEURISTIC = false;
    public static boolean SOLUTION_ANALYSIS = false;
    public static boolean U_ICEA = false;
}
