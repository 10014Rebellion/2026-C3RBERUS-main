package frc.lib.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class HardwareRecords {
    public static record CurrentLimits(
        double supplyCurrentLimit,
        double statorCurrentLimit
    ) {}

    public static record PositionSoftLimits(
        double forwardLimitM,
        double backwardLimitM
    ) {}

    public static record RotationSoftLimits(
        Rotation2d forwardLimit,
        Rotation2d backwardLimit
    ) {}

    public static record MotionConstraints(
        double maxVelocity,
        double maxAcceleration,
        double maxJerk
    ) {}

    public static record BasicMotorHardware(
        int motorID, 
        CANBus canBus, 
        double rotorToMechanismRatio,
        InvertedValue direction,
        NeutralModeValue neutralMode,
        CurrentLimits currentLimit
    ) {}

    public static record FollowerMotorHardware(
        int motorID,
        BasicMotorHardware leaderConfig,
        MotorAlignmentValue alignmentValue
    ) {}

    public static record CANCoderHardware(
        int cancoderID,
        double cancoderToMechanismRatio
    ) {}

    public static record ElevatorController(
        int slot,
        PDConstants pdController,
        ElevatorFeedforward feedforward,
        MotionConstraints constraints
    ) {}

    public static record ArmController(
        int slot,
        PDConstants pdController,
        ArmFeedforward kFeedforward
    ) {}

    public static record SimpleController(
        int slot,
        PDConstants pdController,
        SimpleMotorFeedforward kFeedforward
    ) {}

    public static record MotionMagicFOCController(
        int slot,
        PDConstants pdController,
        MotionMagicConstants motionMagicConstants
    ) {}

    public static record MotionMagicConstants(
        double maxVelocity,
        double maxAcceleration,
        double maxJerk
    ) {}

    public static record PDConstants(
        double kP,
        double kD
    ) {} 
}
