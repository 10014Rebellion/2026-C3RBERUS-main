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
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;

public class FlywheelConstants {
    public static final double kMaxFlywheelTestedRPS = 112;
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
        new CurrentLimits(60, 80)
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

    public static final HashMap<FlywheelStates, LoggedTunableNumber> kFlywheelSetpointToVoltageTuneable = new HashMap<FlywheelStates, LoggedTunableNumber>();
    public static final HashMap<FlywheelStates, Supplier<Rotation2d>> kFlywheelSetpointToVelocity = new HashMap<FlywheelStates, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tStandbyVoltage = new LoggedTunableNumber("Flywheels/SetpointsVoltage/StandbyVoltage", 0.0);
    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Flywheels/SetpointsVoltage/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tMaxVoltage = new LoggedTunableNumber("Flywheels/SetpointsVoltage/MaxVoltage", 0.0);
    public static final LoggedTunableNumber tTuningVelocity = new LoggedTunableNumber("Flywheels/TuneVelocityRPS", 0.0);
    public static final LoggedTunableNumber tFeedVelocity = new LoggedTunableNumber("Flywheels/FeedVelocity", 0.0);
    public static final LoggedTunableNumber tCloseVelocity = new LoggedTunableNumber("Flywheels/SetpointsRPS/CloseVelocity", 0.0);
    public static final LoggedTunableNumber tTowerVelocity = new LoggedTunableNumber("Flywheels/SetpointsRPS/CloseVelocity", 0.0);
    public static final LoggedTunableNumber tBumpVelocity = new LoggedTunableNumber("Flywheels/SetpointsRPS/CloseVelocity", 0.0);
    public static final LoggedTunableNumber tMaxVelocity = new LoggedTunableNumber("Flywheels/SetpointsRPS/MaxVelocity", 0.0);

    static {
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.STANDBY_VOLTAGE, tStandbyVoltage);
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.TUNING_VOLTAGE, tTuningVoltage);
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.MAX_VOLTAGE, tMaxVoltage);
        kFlywheelSetpointToVelocity.put(FlywheelStates.TUNING_VELOCITY, () -> Rotation2d.fromRotations(tTuningVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.FEED_VELOCITY, () -> Rotation2d.fromRotations(tFeedVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.CLOSE_VELOCITY, () -> Rotation2d.fromRotations(tCloseVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.TOWER_VELOCITY, () -> Rotation2d.fromRotations(tTowerVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.BUMP_VELOCITY, () -> Rotation2d.fromRotations(tBumpVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.MAX_VELOCITY, () -> Rotation2d.fromRotations(tMaxVelocity.get()));
    }
}
