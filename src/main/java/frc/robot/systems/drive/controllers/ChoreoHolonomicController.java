package frc.robot.systems.drive.controllers;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.tuning.LoggedTunableNumber;

public class ChoreoHolonomicController {
    public static final LoggedTunableNumber tXP = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/kP", 3.75);
    public static final LoggedTunableNumber tXD = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/kD", 0.0);
    public static final LoggedTunableNumber tXI = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/kI", 0.0);
    public static final LoggedTunableNumber tXIZone = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/kIZone", 0.0);
    public static final LoggedTunableNumber tXIRange = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/kIRange", 0.0);

    public static final LoggedTunableNumber tXS = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/kS", 0.0);
    public static final LoggedTunableNumber tXV = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/kV", 1.0);

    public static final LoggedTunableNumber tXToleranceMeters = 
        new LoggedTunableNumber("ChoreoHolonomicController/X/ToleranceMeters", 0.03);

    public static final LoggedTunableNumber tYP = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/kP", 3.75);
    public static final LoggedTunableNumber tYD = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/kD", 0.0);
    public static final LoggedTunableNumber tYI = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/kI", 0.0);
    public static final LoggedTunableNumber tYIZone = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/kIZone", 0.0);
    public static final LoggedTunableNumber tYIRange = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/kIRange", 0.0);

    public static final LoggedTunableNumber tYS = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/kS", 0.0);
    public static final LoggedTunableNumber tYV = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/kV", 1.0);

    public static final LoggedTunableNumber tYToleranceMeters = 
        new LoggedTunableNumber("ChoreoHolonomicController/Y/ToleranceMeters", 0.05);

    public static final LoggedTunableNumber tOmegaP = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kP", 3.0);
    public static final LoggedTunableNumber tOmegaD = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kD", 0.0);

    public static final LoggedTunableNumber tOmegaI = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kI", 0.0);
    public static final LoggedTunableNumber tOmegaIZone = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kIZone", 0.0);
    public static final LoggedTunableNumber tOmegaIRange = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kIRange", 0.0);

    public static final LoggedTunableNumber tOmegaMaxVDPS = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kMaxVDPS", 180);
    public static final LoggedTunableNumber tOmegaMaxADPSS = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kMaxVDPSS", 360);

    public static final LoggedTunableNumber tOmegaS = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kS", 0.0);
    public static final LoggedTunableNumber tOmegaV = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/kV", 1.0);

    public static final LoggedTunableNumber tOmegaToleranceDegrees = 
        new LoggedTunableNumber("ChoreoHolonomicController/Omega/ToleranceDegrees", 1.5);

    public static final LoggedTunableNumber tDistanceToleranceMeters = new LoggedTunableNumber("AutoAlign/Distance/ToleranceMeters", 0.03);

    public static final LoggedTunableNumber tFFRadius = new LoggedTunableNumber("AutoAlign/ffRadius", 1.0);

    private PIDController tXController;
    private PIDController tYController;
    private PIDController tOmegaController;

    public ChoreoHolonomicController() {
        this.tXController = new PIDController(
            tXP.get(), tXI.get(), tXD.get());
        tXController.setIntegratorRange(-tXIRange.get(), tXIRange.get());
        tXController.setIZone(tXIZone.get());
        tXController.setTolerance(tXToleranceMeters.get());

        this.tYController = new PIDController(
            tYP.get(), 
            tYI.get(), 
            tYD.get());
        tYController.setIntegratorRange(
            -tYIRange.get(), 
            tYIRange.get());
        tYController.setIZone(tYIZone.get());
        tYController.setTolerance(tYToleranceMeters.get());

        this.tOmegaController = new PIDController(
            tOmegaP.get(), 
            tOmegaI.get(), 
            tOmegaD.get());
        tOmegaController.enableContinuousInput(
            -Math.PI, 
            Math.PI);
        tOmegaController.setIntegratorRange(
            -tOmegaIRange.get(), 
            tOmegaIRange.get());
        tOmegaController.setIZone(tOmegaIZone.get());
        tOmegaController.setTolerance(tOmegaToleranceDegrees.get());
    }

    public ChassisSpeeds calculateFromSwerveSample(SwerveSample pSample, Pose2d pCurrentPose) {
        return calculate(
            new Pose2d(
                pSample.x,
                pSample.y,
                Rotation2d.fromRadians(pSample.heading)
            ), 
            pSample.getChassisSpeeds(), 
            pCurrentPose);
    }

    /* Uses 3 PID controllers to set the chassis speeds */
    public ChassisSpeeds calculate(Pose2d pGoalPose, ChassisSpeeds ff, Pose2d pCurrentPose) {
        double ffScalar = 1.0;

        Logger.recordOutput("Drive/Choreo/GoalPose", pGoalPose);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            (tXController.calculate(
                pCurrentPose.getX(),
                pGoalPose.getX())
            + ffScalar * ff.vxMetersPerSecond),
            (tYController.calculate(
                pCurrentPose.getY(),
                pGoalPose.getY())
            + ffScalar * ff.vyMetersPerSecond),
            tOmegaController.calculate(
                pCurrentPose.getRotation().getRadians(),
                pGoalPose.getRotation().getRadians())
            + ffScalar * ff.omegaRadiansPerSecond,
            pCurrentPose.getRotation());
    }

    ////////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/HolonomicController/AtGoal")
    public boolean atSetpoint() {
        return tXController.atSetpoint() && tYController.atSetpoint() && tOmegaController.atSetpoint();
    }

    ////////////////////////// SETTERS \\\\\\\\\\\\\\\\\\\\\\\\\\\\
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

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tXController.setTolerance(tXToleranceMeters.get());
                    tYController.setTolerance(tYToleranceMeters.get());
                    tOmegaController.setTolerance(tOmegaToleranceDegrees.get());
                },
                tXToleranceMeters,
                tYToleranceMeters,
                tOmegaToleranceDegrees);
    }
}
