package frc.robot.systems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.MotionMagicFOCController;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.PDConstants;

public class ShooterConstants {
    /** 
     * Empirically tuned hood angle for a given horizontal shooter-to-hub distance.
     * Distance is in meters, angle is in degrees.
     */
    public record HoodAngleSample(double distanceMeters, double hoodAngleDeg) {}

    /**
     * Empirically tuned flywheel speed for a given horizontal shooter-to-hub distance.
     * Distance is in meters, speed units are shooter-specific (RPM, rad/s, etc).
     */
    public record FlywheelSpeedSample(double distanceMeters, double flywheelSpeed) {}

    /**
     * Estimated projectile time of flight for a given horizontal distance.
     * Distance is in meters, time is in seconds.
     */
    public record TimeOfFlightSample(double distanceMeters, double timeSeconds) {}

    // distances we trust the shooter to make it in.
    public static final double kMinValidShotDistanceMeters = 1.34; // TODO: TUNE ME
    public static final double kMaxValidShotDistanceMeters = 5.60; // TODO: TUNE ME

    public static final double kMinTofDistanceMeters = 1.38; // TODO: TUNE ME
    public static final double kMaxTofDistanceMeters = 5.68; // TODO: TUNE ME


    // Hood angle tuning table (distance → hood pitch)
    public static final HoodAngleSample[] kHoodAngleSamples = {
        new HoodAngleSample(0.0, 0.0), // TODO: TUNE ME
    };

    // Flywheel speed tuning table (distance → exit velocity)
    public static final FlywheelSpeedSample[] kFlywheelSpeedSamples = {
        new FlywheelSpeedSample(0.0, 0.0), // TODO: TUNE ME
    };

    // Ballistic time-of-flight lookup (distance → seconds)
    public static final TimeOfFlightSample[] kTimeOfFlightSamples = {
        new TimeOfFlightSample(0.0, 0.0) // TODO: TUNE ME
    };


    public static class IndexerConstants {
        public static final BasicMotorHardware kIndexerLeaderConfig = new BasicMotorHardware(
            53,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(40, 50)
        );

        public static final FollowerMotorHardware kIndexerFollowerConfig = new FollowerMotorHardware(
            54,
            kIndexerLeaderConfig,
            MotorAlignmentValue.Opposed
        );
    }

    public static class Hood {

    }

    public static class FlywheelConstants {
        public static final BasicMotorHardware kFlywheelLeaderConfig = new BasicMotorHardware(
            51,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(60, 75)
        );

        public static final FollowerMotorHardware kFlywheelFollowerConfig = new FollowerMotorHardware(
            52,
            kFlywheelLeaderConfig,
            MotorAlignmentValue.Opposed
        );

        public static final MotionMagicFOCController kFlywheelControlConfig = new MotionMagicFOCController(
            0,
            new PDConstants(6, 0),
            new MotionMagicConstants(0, 1000, 10000)
        );

    }
}
