package frc.robot.bindings;

import frc.lib.commandlogic.RebelTrigger;
import frc.lib.telemetry.Telemetry;
import frc.robot.bindings.GeneralControllerValues.DriveValues;
import frc.robot.bindings.GeneralControllerValues.IntakeValues;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;

public class TriggerManager {
    private static FlywheelsSS mFlywheelsSS;

    public TriggerManager(FlywheelsSS pFlywheelsSS) {
        this.mFlywheelsSS = pFlywheelsSS;
    }

    public static void initDriverTriggers() {
        try {
            RebelTrigger.newTrigger(() -> IntakeValues.vIntakeDeployPressed.getAsBoolean() && (DriveValues.vDriveRotInput.getAsDouble() == 1), DriveValues.vDriveRotInput)
                .whileTrue();
        
        } catch(NullPointerException e) {
            Telemetry.reportException(e);
        }
    }

    public static void initAutoTriggers() {
        RebelTrigger.newTrigger(mFlywheelsSS);
    }
}
