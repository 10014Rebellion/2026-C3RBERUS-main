package frc.robot.systems.IntakeRoller;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollerConstants {
    public static int kInnerIntakeRollerMotorID = 0;
    public static int kOuterIntakeRollerMotorID = 1;

    public static int kInnerSensorDIOPort = 3;
    public static int kOuterSensorDIOPort =4;

    public static NeutralModeValue kIdleMode = NeutralModeValue.Brake;
    public static boolean kInnerInverted = true;
    public static boolean kOuterInverted = true;

    public static double kIntakeSpeed = 12;

    public static int kSmartCurrentLimit = 60;
    public static int kSecondaryCurrentLimit = 75;

    public static final double kIntakeVolts = 12;

    public static final double kJamDetectTimeSeconds = 1;
    public static final double kUnjamTimeSeconds = 0.25;
    public static final double kUnjamVolts = 6;

    public static final IntakeRollerHardware outerIntakeRollersHardware = new 
    IntakeRollerHardware(kOuterIntakeRollerMotorID, "drivetrain");

    public static final IntakeRollerHardware innerIntakeRollersHardware = new 
    IntakeRollerHardware(kInnerIntakeRollerMotorID, "drivetrain");

    public static final IntakeRollerConfiguration innerMotorConfiguration=
    new IntakeRollerConfiguration(
        kSmartCurrentLimit,
        kSecondaryCurrentLimit, 
        kIdleMode,
        kInnerInverted
    );
    
    public static final IntakeRollerConfiguration outerMotorConfiguration=
    new IntakeRollerConfiguration(
        kSmartCurrentLimit,
        kSecondaryCurrentLimit, 
        kIdleMode,
        kOuterInverted
    );

    public record IntakeRollerHardware(
        int motorPort,
        String canBus
    ){}
    
    public record IntakeRollerConfiguration(
        int smartLimit,
        int secondaryLimit,
        NeutralModeValue idleMode,
        boolean inverted
    ){}
}
