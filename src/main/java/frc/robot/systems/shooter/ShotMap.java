package frc.robot.systems.shooter;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.game.GameGoalPoseChooser;

public class ShotMap {
    private static ShotMap mInstance;
    
    public static ShotMap getInstance() {
        if(mInstance == null) {
            mInstance = new ShotMap();
        }
        return mInstance;
    }

    private Supplier<Pose2d> robotPose = () -> new Pose2d();
    public double offsetM = 0.0;
    public double velocityRPSOffset = -4.8; // negative = slowed down (weaker). positive = sped up (farther)

    public record ShotMapSetpoint(double distance, Rotation2d pHoodSetpoint, Rotation2d flywheelSpeedPS) {}

    public ShotMapSetpoint[] setpoints = new ShotMapSetpoint[] {
        new ShotMapSetpoint(1.25, Rotation2d.fromDegrees(7.0), Rotation2d.fromRotations(50.0 + velocityRPSOffset)),
        new ShotMapSetpoint(1.75, Rotation2d.fromDegrees(8.5), Rotation2d.fromRotations(51.5 + velocityRPSOffset)),
        new ShotMapSetpoint(2.25, Rotation2d.fromDegrees(10.5), Rotation2d.fromRotations(53.0 + velocityRPSOffset)),
        new ShotMapSetpoint(2.75, Rotation2d.fromDegrees(12.0), Rotation2d.fromRotations(54.5 + velocityRPSOffset)),
        new ShotMapSetpoint(3.25, Rotation2d.fromDegrees(12.0), Rotation2d.fromRotations(57.5 + velocityRPSOffset)),
        new ShotMapSetpoint(3.75, Rotation2d.fromDegrees(14.0), Rotation2d.fromRotations(62.5 + velocityRPSOffset)),
        new ShotMapSetpoint(4.17, Rotation2d.fromDegrees(14.0), Rotation2d.fromRotations(65.5 + velocityRPSOffset)),
        // new ShotMapSetpoint(5.05, Rotation2d.fromDegrees(16.0), Rotation2d.fromRotations(65.5 + velocityRPSOffset))
    };

    public InterpolatingDoubleTreeMap distanceVelRPSMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap distanceAngleDegMap = new InterpolatingDoubleTreeMap();

    private ShotMap() {
        for(int i = 0; i < setpoints.length; i++) {
            distanceVelRPSMap.put(setpoints[i].distance(), setpoints[i].flywheelSpeedPS.getRotations());
        }

        for(int i = 0; i < setpoints.length; i++) {
            distanceAngleDegMap.put(setpoints[i].distance(), setpoints[i].pHoodSetpoint().getDegrees());
        }

        CommandScheduler.getInstance().getActiveButtonLoop()
            .bind(() -> {
                Logger.recordOutput("ShotMap/DistanceToHub", distanceToHub());
                Logger.recordOutput("ShotMap/AngleOfHood", getHoodAngle());
                Logger.recordOutput("ShotMap/FlywheelRPS", getFlywheelVel());
            });
    }

    public double distanceToHub() {
        return 
        Math.max(setpoints[0].distance() + 0.01,
            Math.min(
                setpoints[setpoints.length - 1].distance() - 0.01,
                Math.hypot(
                    robotPose.get().getX() - GameGoalPoseChooser.getHub().getX(),
                    robotPose.get().getY() - GameGoalPoseChooser.getHub().getY()
            ))
        );
    }

    public Rotation2d getHoodAngle() {
        return Rotation2d.fromDegrees(distanceAngleDegMap.get(distanceToHub()));
    }

    public Rotation2d getFlywheelVel() {
        return Rotation2d.fromRotations(distanceVelRPSMap.get(distanceToHub()));
    }

    public void setPoseSupplier(Supplier<Pose2d> pPoseSup) {
        robotPose = pPoseSup;
    }

}
