package frc.robot.systems.shooter;

import java.util.function.Supplier;

import org.dyn4j.geometry.Rotatable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.game.GameGoalPoseChooser;

public class ShooterConstants {
    private static Supplier<Pose2d> robotPose = () -> new Pose2d();
    public double offsetM = 0.0;

    public record ShotMapSetpoint(double distance, Rotation2d pHoodSetpoint, Rotation2d flywheelSpeedPS) {}

    public static ShotMapSetpoint[] setpoints = new ShotMapSetpoint[] {
        new ShotMapSetpoint(1.25, Rotation2d.fromDegrees(7.0), Rotation2d.fromRotations(47.5)),
        new ShotMapSetpoint(1.75, Rotation2d.fromDegrees(8.5), Rotation2d.fromRotations(49.0)),
        new ShotMapSetpoint(2.25, Rotation2d.fromDegrees(10.5), Rotation2d.fromRotations(50.5)),
        new ShotMapSetpoint(2.75, Rotation2d.fromDegrees(12.0), Rotation2d.fromRotations(52.0)),
        new ShotMapSetpoint(3.25, Rotation2d.fromDegrees(12.0), Rotation2d.fromRotations(55.0)),
        new ShotMapSetpoint(3.75, Rotation2d.fromDegrees(14.0), Rotation2d.fromRotations(60.0)),
        new ShotMapSetpoint(4.17, Rotation2d.fromDegrees(14.0), Rotation2d.fromRotations(63.0)),
    };

    public static InterpolatingDoubleTreeMap distanceVelRPSMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap distanceAngleDegMap = new InterpolatingDoubleTreeMap();

    static {
        for(int i = 0; i < setpoints.length; i++) {
            distanceVelRPSMap.put(setpoints[i].distance(), setpoints[i].flywheelSpeedPS.getRotations());
        }

        for(int i = 0; i < setpoints.length; i++) {
            distanceAngleDegMap.put(setpoints[i].distance(), setpoints[i].pHoodSetpoint().getDegrees());
        }
    }

    public static double distanceToHub() {
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

    public static Rotation2d getHoodAngle() {
        return Rotation2d.fromDegrees(distanceAngleDegMap.get(3.0));
    }

    public static Rotation2d getFlywheelVel() {
        return Rotation2d.fromRotations(distanceVelRPSMap.get(3.0));
    }

    public static void setPoseSupplier(Supplier<Pose2d> pPoseSup) {
        robotPose = pPoseSup;
    }

}
