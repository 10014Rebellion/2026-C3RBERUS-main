// REBELLION 10014

package frc.lib.telemetry;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.telemetry.TelemetryConstants.Severity;

import java.util.HashSet;
import java.util.Set;

public class Telemetry extends TelemetryRecordOutput {
    private static final Set<TelemetryError> activeErrors = new HashSet<>();

    private Telemetry() {}

    // TODO: ADD PROPER TELEM LATER
    public static void log(String pMessage) {
        System.out.println("~~~ " + pMessage + " ~~~");
    }

    /*
     * Call when error is created
     * @param Error to resolve
     */
    public static void reportIssue(TelemetryError pError) {
        if (activeErrors.add(pError)) {
            Throwable trace = new Throwable();
            publishError(pError, trace);
        }
    }

    /*
     * Call when error is resolved
     * @param Error to resolve
     */
    public static void clearIssue(TelemetryError pError) {
        if (activeErrors.remove(pError)) {
            publishClear(pError);
        }
    }

    /*
     * Reports or clears messages based on a condition. Reports it if the condition is true, clears it if the condition is false
     * @param Condition on wether or not it should report
     * @param The error enum to report
     * @return Simply returns the condition that was passed in
     */
    public static boolean conditionReport(boolean pCondition, TelemetryError pError) {
        if (pCondition) {
            reportIssue(pError);
        } else {
            clearIssue(pError);
        }

        return pCondition;
    }

    // TODO: IMPLEMENT PROPER EXCEPTION THROWING LATER
    public static Exception reportException(Exception pException) {
        pException.getStackTrace();
        
        System.out.println("!!! ^^^ EXCEPTION ABOVE ^^^!!!\n");
        return pException;
    }

    // TODO: IMPLEMENT PROPER TELEM LATER
    private static void publishError(TelemetryError pError, Throwable pStackTrace) {
        if (pError.severity() == Severity.ERROR
                || pError.severity() == Severity.FATAL
                || pError.severity() == Severity.MOMENTS_AWAY_FROM_CATASTROPHE) {
            DriverStation.reportError("<<< " + pError.message() + " >>>", false);
        } else {
            DriverStation.reportWarning("<<< " + pError.message() + " >>>", false);
        }

        System.out.println("---------------------\n\n<<< ISSUE WITH SEVERITY: " + pError.severity() + " >>> \n");

        if (pStackTrace != null) pStackTrace.printStackTrace(System.out);

        System.out.println("\n---------------------");
    }

    // TODO: IMPLEMENT PROPER TELEM LATER
    private static void publishClear(TelemetryError pError) {
        System.out.println("||| CLEARED: " + pError.message() + " |||");
    }

    public static int getNumberOfActiveErrors() {
        return activeErrors.size();
    }

    public static int getNumberOfActiveErrors(Severity pErrorSeverity) {
        int count = 0;
        for (TelemetryError error : activeErrors) if (error.severity() == pErrorSeverity) count++;
        return count;
    }

    public static int getNumberOfActiveErrors(Class<? extends TelemetryError> pErrorClass) {
        int count = 0;
        for (TelemetryError lError : activeErrors) if (pErrorClass.isInstance(lError)) count++;
        return count;
    }
}
