package frc.robot.systems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.MotionMagicFOCControllerFF;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RelativeCANCoderHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
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

   // ShooterYawOffset is the fixed yaw of shooter relative to robot forward.
   // Example: shooter points forward -> Rotation2d.kZero
   // Example: shooter points left -> Rotation2d.fromDegrees(90)
   public static final Rotation2d kShooterYawOffset = Rotation2d.kZero;


    // Hood angle tuning table (distance -> hood pitch)
    public static final HoodAngleSample[] kHoodAngleSamples = {
        new HoodAngleSample(0.0, 0.0), // TODO: TUNE ME
    };

    // Flywheel speed tuning table (distance -> exit velocity)
    public static final FlywheelSpeedSample[] kFlywheelSpeedSamples = {
        new FlywheelSpeedSample(0.0, 0.0), // TODO: TUNE ME
    };

    // Ballistic time-of-flight lookup (distance -> seconds)
    public static final TimeOfFlightSample[] kTimeOfFlightSamples = {
        new TimeOfFlightSample(0.0, 0.0) // TODO: TUNE ME
    };


    public static class FuelPumpConstants {

        public static final double kToleranceRPS = 3.0;

        public enum FuelPumpStates {
            DISCONNECTED(null),
            INDEXING(() -> Rotation2d.fromRotations(0)), // TODO: TUNE ME!
            UNJAMMING(() -> Rotation2d.fromRotations(0)), // TODO: TUNE ME!
            STOPPED(() -> Rotation2d.fromRotations(0));

            Supplier<Rotation2d> mDesiredRPS;
            FuelPumpStates(Supplier<Rotation2d> pDesiredRPS) {
                this.mDesiredRPS = pDesiredRPS;
            }

            public Supplier<Rotation2d> getRPS() {
                return mDesiredRPS == null ? () -> Rotation2d.kZero : mDesiredRPS;
            }

            public Rotation2d getRotRPS() {
                return mDesiredRPS == null ? Rotation2d.kZero : mDesiredRPS.get();
            }

            public double getValueRPS() {
                return mDesiredRPS == null ? 0 : mDesiredRPS.get().getRotations();
            }
        }

        public static final BasicMotorHardware kFuelPumpLeaderConfig = new BasicMotorHardware(
            53,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(40, 50)
        );

        public static final FollowerMotorHardware kFuelPumpFollowerConfig = new FollowerMotorHardware(
            54,
            kFuelPumpLeaderConfig,
            MotorAlignmentValue.Opposed
        );

        public static final MotionMagicFOCControllerFF kFuelPumpControlConfig = new MotionMagicFOCControllerFF(
            0,
            new PDConstants(0, 0),
            new SimpleMotorFeedforward(0.37, 0, 0),
            new MotionMagicConstants(0, 1000, 10000)
        );
    }

    public static class HoodConstants {

        public static final Rotation2d kIncrementStepAmount = Rotation2d.fromDegrees(1.75);
        public static final double kToleranceRotations = 0.5;

        public static final BasicMotorHardware kHoodConfig = new BasicMotorHardware(
            55,
            Constants.kSubsystemsCANBus,
            133.0 / 9.0, 
            InvertedValue.Clockwise_Positive, 
            NeutralModeValue.Brake, 
            new CurrentLimits(40, 50)
        );

        public static final ArmControllerMotionMagic kHoodControlConfig = new ArmControllerMotionMagic(
            0, // not currently used
            new PDConstants(5, 0), // Tuned for C3RBERUS!
            new MotionMagicConstants(100, 200, 0),  // Tuned for C3RBERUS!
            new ArmFeedforward(0, 0.03, 0, 0) // Tuned for C3RBERUS!
        );

        public static final RotationSoftLimits kHoodLimits = new RotationSoftLimits(
            Rotation2d.fromDegrees(0.02), 
            Rotation2d.fromDegrees(23.3) // TUNE ME! 
        );
    }

    public static class FlywheelConstants {

        public static final double kMaxFlywheelTestedRPS = 112;
        public static final double kToleranceRPS = 3.0;
        public static final double kFFDampener = 0.9;

        public static final RelativeCANCoderHardware kCANCoderConfig = new RelativeCANCoderHardware(
            50,
            1,
            SensorDirectionValue.Clockwise_Positive
        );
        ;

        public static final BasicMotorHardware kFlywheelLeaderConfig = new BasicMotorHardware(
            51,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(60, 80)
        );

        public static final FollowerMotorHardware kFlywheelFollowerConfig = new FollowerMotorHardware(
            52,
            kFlywheelLeaderConfig,
            MotorAlignmentValue.Opposed
        );

        public static final MotionMagicFOCControllerFF kFlywheelControlConfig = new MotionMagicFOCControllerFF(
            0, // not currently used
            // original kP = 9 
            new PDConstants(9, 0), // Tuned for C3RBERUS!
            new SimpleMotorFeedforward(5.0, 0.1, 0), // Tuned for C3RBERUS!
            new MotionMagicConstants(0, 1000, 10000) // Tuned for C3RBERUS!
        );
    }
}
