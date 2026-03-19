package frc.robot.systems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.units.Rot2d;

public class ShooterConstants {
    public record EntireShotMapSample(double distanceMeters, Rotation2d hoodAngleDeg, Rotation2d flywheelSpeed, double timeSeconds) {}


    // distances we trust the shooter to make it in.
    public static final double kMinValidShotDistanceMeters = 1.34; // TODO: TUNE ME
    public static final double kMaxValidShotDistanceMeters = 5.60; // TODO: TUNE ME

    public static final double kMinTofDistanceMeters = 1.38; // TODO: TUNE ME
    public static final double kMaxTofDistanceMeters = 5.68; // TODO: TUNE ME

   // ShooterYawOffset is the fixed yaw of shooter relative to robot forward.
   // Example: shooter points forward -> Rotation2d.kZero
   // Example: shooter points left -> Rotation2d.fromDegrees(90)
   public static final Rotation2d kShooterYawOffset = Rotation2d.kZero;


    // Hood angle tuning table (distance -> hood pitch)
    public static final EntireShotMapSample[] kShotMapSamplesWithToF = {
        new EntireShotMapSample(0.00, Rot2d.fromRot(0.5), Rot2d.fromRot(0.5), 0.00),
    };
}
