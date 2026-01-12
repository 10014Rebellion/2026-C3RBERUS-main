// REBELLION 10014

package frc.robot.systems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import java.util.function.Supplier;

public class HeadingController {
    public static final LoggedTunableNumber mSnapP = new LoggedTunableNumber("SwerveHeadingController/Snap/kP", 4.0);
    public static final LoggedTunableNumber mSnapI = new LoggedTunableNumber("SwerveHeadingController/Snap/kI", 0.0);
    public static final LoggedTunableNumber mSnapD = new LoggedTunableNumber("SwerveHeadingController/Snap/kD", 0.0);
    public static final LoggedTunableNumber mSnapMaxVDPS =
            new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxV", 1000.0);
    public static final LoggedTunableNumber mSnapMaxADPSS =
            new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxA", 1000.0);

    // public static final LoggedTunableNumber stablizingP =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kP", 2.5);
    // public static final LoggedTunableNumber stablizingI =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kI", 0.0);
    // public static final LoggedTunableNumber stablizingD =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kD", 0.0);

    public static final LoggedTunableNumber mToleranceDegrees =
            new LoggedTunableNumber("SwerveHeadingController/Tolerance", 0.75);

    private ProfiledPIDController mSnapController;

    private PIDController mStabilizingController;

    private Supplier<Rotation2d> mGoal;

    public HeadingController() {
        mSnapController = new ProfiledPIDController(
                mSnapP.get(),
                mSnapI.get(),
                mSnapD.get(),
                new TrapezoidProfile.Constraints(mSnapMaxVDPS.get(), mSnapMaxADPSS.get()));

        mSnapController.enableContinuousInput(0, 360);
        mSnapController.setTolerance(1.0);

        // stabilizingController =
        //     new PIDController(stablizingP.get(), stablizingI.get(), stablizingD.get());
        // stabilizingController.enableContinuousInput(0, 360);
        // stabilizingController.setTolerance(0.0);
    }

    public void setHeadingGoal(Supplier<Rotation2d> pGoalSupplier) {
        mGoal = pGoalSupplier;
    }

    public void reset(Rotation2d pRobotRotation, Rotation2d pRobotRotationPerSecond) {
        mSnapController.reset(pRobotRotation.getDegrees(), pRobotRotationPerSecond.getDegrees());
    }

    // Designed for large jumps toward a setpoint, kind of like an azimuth alignment
    public double getSnapOutput(Rotation2d pRobotRotation) {
        Telemetry.log(
                "Drive/HeadingController/HeadingSetpoint",
                Rotation2d.fromDegrees(mSnapController.getSetpoint().position));

        double pidOutput = mSnapController.calculate(
                pRobotRotation.getDegrees(), mGoal.get().getDegrees());
        double ffOutput = mSnapController.getSetpoint().velocity;
        double output = Math.toRadians(pidOutput + ffOutput);

        double setpointErrorDegrees = mSnapController.getSetpoint().position - pRobotRotation.getDegrees();
        double goalErrorDegrees = mSnapController.getGoal().position - pRobotRotation.getDegrees();

        double adjustedOutput = output;
        if (Math.abs(goalErrorDegrees) < mToleranceDegrees.get()) adjustedOutput *= 0.0;

        Telemetry.log("Drive/HeadingController/unAdjustedOutput", output);

        Telemetry.log("Drive/HeadingController/setpointErrorDegrees", setpointErrorDegrees);
        Telemetry.log("Drive/HeadingController/goalErrorDegrees", goalErrorDegrees);

        Telemetry.log("Drive/HeadingController/adjustedOutput", adjustedOutput);

        Telemetry.log("Drive/HeadingController/pidOutput", pidOutput);
        Telemetry.log("Drive/HeadingController/ffOutput", ffOutput);

        return output;
    }

    // Designed for shoot on move and short distance. In most cases velocityDPS is 0.
    public double getStabilizingOutput(Rotation2d pRobotRotation, double pVelocityDPS) {
        return Math.toRadians(mStabilizingController.calculate(
                        pRobotRotation.getDegrees(), mGoal.get().getDegrees())
                + pVelocityDPS);
    }

    public void updateHeadingController() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    mSnapController.setPID(mSnapP.get(), mSnapI.get(), mSnapD.get());
                    mSnapController.setConstraints(new Constraints(mSnapMaxVDPS.get(), mSnapMaxADPSS.get()));
                },
                mSnapP,
                mSnapI,
                mSnapD,
                mSnapMaxVDPS,
                mSnapMaxADPSS);

        // LoggedTunableNumber.ifChanged(hashCode(), () -> {
        //     stabilizingController.setPID(stablizingP.get(), stablizingI.get(), stablizingD.get());
        // }, stablizingP, stablizingI, stablizingD);
    }
}
