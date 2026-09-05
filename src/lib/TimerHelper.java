package lib;

public class TimerHelper {
    private static TimerHelper instance;
    private double gbl_timer;
    private double in_timer;
    private boolean timeout;
    private double tiLim;
    private TimerHelper() {
        // private constructor to prevent instantiation
    }

    public static TimerHelper getInstance() {
        if (instance == null) {
            instance = new TimerHelper();
        }
        return instance;
    }

    public double startGlobal(double _tiLim) {
        gbl_timer = System.nanoTime();
        timeout = false;
        tiLim = _tiLim;
        return gbl_timer;
    }
    
    public void startInner(){
            in_timer = System.nanoTime();
    }
    
    public boolean hasTimedOut() {
        if(timeout)
            return true;
        if (System.nanoTime() - gbl_timer > tiLim) {
            System.out.println("Time Out. Total Runtime:" + (System.nanoTime() - gbl_timer) * 1e-9);
            timeout = true;
            return true;
        }
        return false;
    }

    public double getTimePassedInner(){
            return System.nanoTime() - in_timer;
    }

    public double getTimePassedSecondsInner(){
        return getTimePassedInner() * 1e-9;
    }

    public boolean hasTimedOutInner(double tiLim) {
        if (System.nanoTime() - in_timer > tiLim) {
            System.out.println("Timeout:" + (System.nanoTime() - in_timer) * 1e-9);
            return true;
        }
        return false;
    }
    
    public double getTime() {
        return System.nanoTime();
    }
    
    public double getTimePassed(double since) {
        return System.nanoTime() - since;
    }
    
    public double getTimePassedSeconds(double since){
        return getTimePassed(since) * 1e-9;
    }

    public double getTimeInSeconds(double time) {
        return time*1e-9;
    }

    public boolean hasTimedOut(double timer, double tiLim) {
        if (System.nanoTime() - timer > tiLim) {
            System.out.println("Timeout:" + (System.nanoTime() - timer) * 1e-9);
            return true;
        }
        return false;
    }

    public void setNewTilim(double v) {
        timeout = false;
        tiLim = v;
    }

    public double getTiLim(){
        return tiLim;
    }
}
