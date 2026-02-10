package frc.robot.systems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.tuning.LoggedTunableNumber;

public class SpeedErrorController {
    public static final LoggedTunableNumber tXP = new LoggedTunableNumber("AutoAlign/X/kP", 0.0);
    public static final LoggedTunableNumber tXD = new LoggedTunableNumber("AutoAlign/X/kD", 0.0);
    public static final LoggedTunableNumber tXI = new LoggedTunableNumber("AutoAlign/X/kI", 0.0);
    public static final LoggedTunableNumber tXIZone = new LoggedTunableNumber("AutoAlign/X/kIZone", 0.0);
    public static final LoggedTunableNumber tXIRange = new LoggedTunableNumber("AutoAlign/X/kIRange", 0.0);

    public static final LoggedTunableNumber tXToleranceMPS = new LoggedTunableNumber("AutoAlign/X/ToleranceMPS", 0.03);

    public static final LoggedTunableNumber tYP = new LoggedTunableNumber("AutoAlign/Y/kP", 0.0);
    public static final LoggedTunableNumber tYD = new LoggedTunableNumber("AutoAlign/Y/kD", 0.0);
    public static final LoggedTunableNumber tYI = new LoggedTunableNumber("AutoAlign/Y/kI", 0.0);
    public static final LoggedTunableNumber tYIZone = new LoggedTunableNumber("AutoAlign/Y/kIZone", 0.0);
    public static final LoggedTunableNumber tYIRange = new LoggedTunableNumber("AutoAlign/Y/kIRange", 0.0);
    public static final LoggedTunableNumber tYToleranceMPS = new LoggedTunableNumber("AutoAlign/Y/ToleranceMPS", 0.05);

    public static final LoggedTunableNumber tOmegaP = new LoggedTunableNumber("AutoAlign/Omega/kP", 0.0);
    public static final LoggedTunableNumber tOmegaD = new LoggedTunableNumber("AutoAlign/Omega/kD", 0.0);
    public static final LoggedTunableNumber tOmegaI = new LoggedTunableNumber("AutoAlign/Omega/kI", 0.0);
    public static final LoggedTunableNumber tOmegaIZone = new LoggedTunableNumber("AutoAlign/Omega/kIZone", 0.0);
    public static final LoggedTunableNumber tOmegaIRange = new LoggedTunableNumber("AutoAlign/Omega/kIRange", 0.0);
    public static final LoggedTunableNumber tOmegaToleranceDPS = new LoggedTunableNumber("AutoAlign/Omega/ToleranceDPS", 1.5);

    private final PIDController tXController;
    private final PIDController tYController;
    private final PIDController tOmegaController;

    public SpeedErrorController() {
        this.tXController = new PIDController(tXP.get(), tXI.get(), tXD.get());
        tXController.setIntegratorRange(-tXIRange.get(), tXIRange.get());
        tXController.setIZone(tXIZone.get());

        this.tYController = new PIDController(tYP.get(), tYI.get(), tYD.get());
        tYController.setIntegratorRange(-tYIRange.get(), tYIRange.get());
        tYController.setIZone(tYIZone.get());

        this.tOmegaController = new PIDController(tOmegaP.get(), tOmegaI.get(), tOmegaD.get());
        tOmegaController.setIntegratorRange(-tOmegaIRange.get(), tOmegaIRange.get());
        tOmegaController.setIZone(tOmegaIZone.get());
    }

    public ChassisSpeeds correctSpeed(ChassisSpeeds measured, ChassisSpeeds setpoint) {
        double vXCorrectionMPS = tXController.calculate(measured.vxMetersPerSecond, setpoint.vxMetersPerSecond);
        double vYCorrectionMPS = tYController.calculate(measured.vyMetersPerSecond, setpoint.vyMetersPerSecond);
        double omegaCorrectionRadPS = tOmegaController.calculate(measured.omegaRadiansPerSecond, setpoint.omegaRadiansPerSecond);

        return new ChassisSpeeds(
            setpoint.vxMetersPerSecond + vXCorrectionMPS,
            setpoint.vyMetersPerSecond + vYCorrectionMPS,
            setpoint.omegaRadiansPerSecond + omegaCorrectionRadPS
        );
    }

    public void updateControllers() {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                tXController.setPID(tXP.get(), tXI.get(), tXD.get());
                tXController.setIntegratorRange(-tXIRange.get(), tXIRange.get());
                tXController.setIZone(tXIZone.get());
            },
            tXP,
            tXI,
            tXD,
            tXIRange,
            tXIZone);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                tYController.setPID(tYP.get(), tYI.get(), tYD.get());
                tYController.setIntegratorRange(-tYIRange.get(), tYIRange.get());
                tYController.setIZone(tYIZone.get());
            },
            tYP,
            tYI,
            tYD,
            tYIRange,
            tYIZone);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                tOmegaController.setPID(tOmegaP.get(), tOmegaI.get(), tOmegaD.get());
                tOmegaController.setIntegratorRange(-tOmegaIRange.get(), tOmegaIRange.get());
                tOmegaController.setIZone(tOmegaIZone.get());
            },
            tOmegaP,
            tOmegaI,
            tOmegaD,
            tOmegaIRange,
            tOmegaIZone);
    }
}
