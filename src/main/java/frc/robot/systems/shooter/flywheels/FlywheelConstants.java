package frc.robot.systems.shooter.flywheels;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.MotionMagicFOCControllerFF;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RelativeCANCoderHardware;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
// FlywheelStates enum migrated to per-behavior commands; use string keys here.

public class FlywheelConstants {
    public static final double kMaxFlywheelTestedRPS = 115;
    public static final double kToleranceRPS = 3.0;
    public static final double kBangBangTimeout = 0.25;

    public static final RelativeCANCoderHardware kCANCoderConfig = new RelativeCANCoderHardware(
        50,
        1,
        SensorDirectionValue.Clockwise_Positive
    ); 

    public static final BasicMotorHardware kFlywheelLeaderConfig = new BasicMotorHardware(
        51,
        RobotConstants.kSubsystemsCANBus,
        1,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        new CurrentLimits(40, 80)
    );

    public static final FollowerMotorHardware kFlywheelFollowerConfig = new FollowerMotorHardware(
        52,
        kFlywheelLeaderConfig,
        MotorAlignmentValue.Opposed
    );

    public static final MotionMagicFOCControllerFF kFlywheelControlConfig = new MotionMagicFOCControllerFF(
        0,
        new PDConstants(10.0, 0), // Tuned for C3RBERUS!
        new SimpleMotorFeedforward(0.37, 0.13, 0), // Tuned for C3RBERUS!
        new MotionMagicConstants(0.0, 1000.0, 0.0)
    );

    public static final HashMap<String, LoggedTunableNumber> kFlywheelSetpointToVoltageTuneable = new HashMap<String, LoggedTunableNumber>();
    public static final HashMap<String, Supplier<Rotation2d>> kFlywheelSetpointToVelocity = new HashMap<String, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tStandbyVoltage = new LoggedTunableNumber("Shooter/Flywheel/SetpointsVoltage/StandbyVoltage", 0.0);
    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Shooter/Flywheel/SetpointsVoltage/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tMaxVoltage = new LoggedTunableNumber("Shooter/Flywheel/SetpointsVoltage/MaxVoltage", 0.0);
    public static final LoggedTunableNumber tTuningVelocity = new LoggedTunableNumber("Shooter/Flywheel/TuneVelocityRPS", 0.0);
    public static final LoggedTunableNumber tFeedVelocity = new LoggedTunableNumber("Shooter/Flywheel/FeedVelocity", 60.0);
    public static final LoggedTunableNumber tOpponentFeedVelocity = new LoggedTunableNumber("Shooter/Flywheel/OpponentFeedVelocity", 110.0);
    public static final LoggedTunableNumber tCloseVelocity = new LoggedTunableNumber("Shooter/Flywheel/SetpointRPS/CloseVelocity", 47.5);
    public static final LoggedTunableNumber tTowerVelocity = new LoggedTunableNumber("Shooter/Flywheel/SetpointRPS/TowerVelocity", 57.5);
    public static final LoggedTunableNumber tBumpVelocity = new LoggedTunableNumber("Shooter/Flywheel/SetpointRPS/BumpVelocity", 0.0);
    public static final LoggedTunableNumber tMaxVelocity = new LoggedTunableNumber("Shooter/Flywheel/SetpointRPS/MaxVelocity", 0.0);
    public static final LoggedTunableNumber tStandbyVelocity = new LoggedTunableNumber("Shooter/Flywheel/SetpointRPS/StandbyVelocity", 17.5);


    static {
    kFlywheelSetpointToVoltageTuneable.put("STANDBY_VOLTAGE", tStandbyVoltage);
    kFlywheelSetpointToVoltageTuneable.put("TUNING_VOLTAGE", tTuningVoltage);
    kFlywheelSetpointToVoltageTuneable.put("MAX_VOLTAGE", tMaxVoltage);
    kFlywheelSetpointToVelocity.put("TUNING_VELOCITY", () -> Rotation2d.fromRotations(tTuningVelocity.get()));
    kFlywheelSetpointToVelocity.put("FEED_VELOCITY", () -> Rotation2d.fromRotations(tFeedVelocity.get()));
    kFlywheelSetpointToVelocity.put("OPPONENT_FEED_VELOCITY", () -> Rotation2d.fromRotations(tOpponentFeedVelocity.get()));
    kFlywheelSetpointToVelocity.put("CLOSE_VELOCITY", () -> Rotation2d.fromRotations(tCloseVelocity.get()));
    kFlywheelSetpointToVelocity.put("TOWER_VELOCITY", () -> Rotation2d.fromRotations(tTowerVelocity.get()));
    kFlywheelSetpointToVelocity.put("BUMP_VELOCITY", () -> Rotation2d.fromRotations(tBumpVelocity.get()));
    kFlywheelSetpointToVelocity.put("MAX_VELOCITY", () -> Rotation2d.fromRotations(tMaxVelocity.get()));
    kFlywheelSetpointToVelocity.put("STANDBY_VELOCITY", () -> Rotation2d.fromRotations(tStandbyVelocity.get()));
    }
}
