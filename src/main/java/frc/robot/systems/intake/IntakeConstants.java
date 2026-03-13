package frc.robot.systems.intake;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotStates;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CANdiEncoder;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.lib.tuning.LoggedTunableNumber;

public class IntakeConstants {
    public static class PivotConstants {
        public static Rotation2d kPivotMotorToleranceRotations = Rotation2d.fromDegrees(2.0);

        public static final BasicMotorHardware kPivotMotorConfig = new BasicMotorHardware(
            41, // TODO: TUNE ME;
            Constants.kSubsystemsCANBus,
            13.5,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Brake,
            new CurrentLimits(60, 80)
        );

        public static final CANdiEncoder kPivotEncoderConfig = new CANdiEncoder(
            40, 
            FeedbackSensorSourceValue.FusedCANdiPWM1,
            1.0,
            Rotation2d.fromRotations(-0.241699 + 0.055555) // 0.055555 is to account for the fact that our CG is 20deg off the ground
        );

        // WITH POSITION VOLTAGE
        public static final ArmControllerMotionMagic kPivotController = (!Constants.isSim()) ?
            new ArmControllerMotionMagic(
                0, 
                new PDConstants(19.0, 0.0), 
                new MotionMagicConstants(0, 0, 0), // NOT USED
                new ArmFeedforward(0.5, 0.6, 0, 0)
            )
                :
            new ArmControllerMotionMagic(
                0, 
                new PDConstants(3, 0.0), 
                new MotionMagicConstants(300.0, 300.0, 0), // NOT USED
                new ArmFeedforward(0.0, 3.0, 0, 0)
            );

        public static final RotationSoftLimits kPivotLimits = new RotationSoftLimits(
            Rotation2d.fromRotations(-0.068), // Negative voltage limit
            Rotation2d.fromRotations(0.318) // Positive voltage limit
        );

        public static final double kArmLengthMeters = 0.5376418;
        public static final double kArmMass = 5.0;

        public static final LoggedTunableNumber tPivotTuningVoltage = new LoggedTunableNumber("Intake/Tuning/TuneVoltage", 0.0);
        public static final LoggedTunableNumber tPivotTuningAmp = new LoggedTunableNumber("Intake/Tuning/TuneAmperage", 0.0);

        public static final LoggedTunableNumber tStowSetpointDeg  = new LoggedTunableNumber(
            "Hood/Setpoint/StowSetpointDegrees", 114.48);
        public static final Supplier<Rotation2d> kStowSetpointSup = () -> Rotation2d.fromDegrees(tStowSetpointDeg.get());

        public static final LoggedTunableNumber tCompactSetpointDeg  = new LoggedTunableNumber(
            "Hood/Setpoint/CompactSetpointDegrees", 25.0);
        public static final Supplier<Rotation2d> kCompactSetpointSup = () -> Rotation2d.fromDegrees(tCompactSetpointDeg.get());

        public static final LoggedTunableNumber tIntakeSetpointDeg  = new LoggedTunableNumber(
            "Hood/Setpoint/IntakeSetpointDegrees", -24.48);
        public static final Supplier<Rotation2d> kIntakeSetpointSup = () -> Rotation2d.fromDegrees(tIntakeSetpointDeg.get());

        public static final LoggedTunableNumber tTuningShotSetpointDeg  = new LoggedTunableNumber(
            "Hood/Setpoint/TuningShotSetpointDegrees", 0.0);
        public static final Supplier<Rotation2d> kTuningShotSetpointSup = () -> Rotation2d.fromDegrees(tTuningShotSetpointDeg.get());

        public static final HashMap<IntakePivotStates, Supplier<Rotation2d>> kStateToSetpointMapHood = new HashMap<IntakePivotStates, Supplier<Rotation2d>>();

        static {
            kStateToSetpointMapHood.put(IntakePivotStates.STOW, kStowSetpointSup);
            kStateToSetpointMapHood.put(IntakePivotStates.COMPACT, kCompactSetpointSup);
            kStateToSetpointMapHood.put(IntakePivotStates.INTAKE, kIntakeSetpointSup);
            kStateToSetpointMapHood.put(IntakePivotStates.TUNING_SETPOINT, kTuningShotSetpointSup);
        }
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

        public static final LoggedTunableNumber tIdleTuningVoltage = new LoggedTunableNumber("Intake/Voltage/IDLE", 0.0);
        public static final LoggedTunableNumber tIntakeTuningVoltage = new LoggedTunableNumber("Intake/Voltage/INTAKE", 11.014);
        public static final LoggedTunableNumber tOuttakeTuningVoltage = new LoggedTunableNumber("Intake/Voltage/OUTTAKE", -10.014);
        public static final LoggedTunableNumber tRollerTuningVoltage = new LoggedTunableNumber("Intake/Voltage/TUNING", 0.0);

        public static final HashMap<IntakeRollerState, LoggedTunableNumber> kStateToIntakeVoltage = new HashMap<>();

        static {
            kStateToIntakeVoltage.put(IntakeRollerState.IDLE, tIdleTuningVoltage);
            kStateToIntakeVoltage.put(IntakeRollerState.INTAKE, tIntakeTuningVoltage);
            kStateToIntakeVoltage.put(IntakeRollerState.OUTTAKE, tOuttakeTuningVoltage);
            kStateToIntakeVoltage.put(IntakeRollerState.TUNING, tRollerTuningVoltage);
        }
    }
}
