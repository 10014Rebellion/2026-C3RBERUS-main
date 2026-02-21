package frc.robot.systems.intake;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CANdiEncoder;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;

public class IntakeConstants {
    public static class PivotConstants {
        public static final BasicMotorHardware kPivotMotorConfig = new BasicMotorHardware(
            41, // TODO: TUNE ME;
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.Clockwise_Positive,
            NeutralModeValue.Brake,
            new CurrentLimits(30, 40)
        );

        public static final CANdiEncoder kPivotEncoderConfig = new CANdiEncoder(
            40, 
            FeedbackSensorSourceValue.FusedCANdiPWM1,
            1,
            Rotation2d.fromRotations(0.335)
            );

        public static final ArmControllerMotionMagic kPivotController = new ArmControllerMotionMagic(
            0, 
            new PDConstants(5, 0), 
            new MotionMagicConstants(5, 20, 0), 
            new ArmFeedforward(0, 0, 0, 0)
        );

        public static final RotationSoftLimits kPivotLimits = new RotationSoftLimits(
            Rotation2d.fromRotations(0.294), // Negative voltage limit
            Rotation2d.fromRotations(-0.088) // Positive voltage limit
        );
    }

    public static class RollerConstants {
        public final static BasicMotorHardware kRollerMotorConfig = new BasicMotorHardware(
            42,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(30, 40)
        );
    }
    
}
