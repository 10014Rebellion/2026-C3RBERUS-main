// REBELLION 10014

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;
    public static final boolean kTuningMode = !DriverStation.isFMSAttached();

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class DashboardConstants {
        public static final boolean kDashboardEnabled = true;
        public static final String kDashboardPath = "dashboard";
        public static final int kDashboardPort = 5800;

        public static final boolean kDeployServerEnabled = false;
        public static final String kDeployServerPath = "";
        public static final int kDeployServerPort = 5801;
    }
}
