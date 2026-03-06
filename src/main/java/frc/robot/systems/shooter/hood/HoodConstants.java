package frc.robot.systems.shooter.hood;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.robot.Constants;

public class HoodConstants {
    public static final Rotation2d kIncrementStepAmount = Rotation2d.fromDegrees(2);
    public static final double kToleranceRotations = 0.5;

    public static final BasicMotorHardware kHoodConfig = new BasicMotorHardware(
        55, // Motor CAN ID
        Constants.kSubsystemsCANBus, // CANBus
        (133.0 / 9.0) * (20.0 / 12.0), // Rotor to Mechanism Gear Ratio
        InvertedValue.Clockwise_Positive, // Direction
        NeutralModeValue.Brake, // Neutral Mode 
        new CurrentLimits(
            40, // Supply
            50 // Stator
        )
    );

    public static final ArmControllerMotionMagic kHoodControlConfig = new ArmControllerMotionMagic(
        0, // not currently used
        new PDConstants(22, 0), // Tuned for C3RBERUS!
        new MotionMagicConstants(100, 1000, 0),  // Tuned for C3RBERUS!
        new ArmFeedforward(0, 0.06, 0, 0) // Tuned for C3RBERUS!
    );

    public static final RotationSoftLimits kHoodLimits = new RotationSoftLimits(
        Rotation2d.fromDegrees(0.02), 
        Rotation2d.fromDegrees(21.2) // Tuned for C3RBERUS!
    );
}
