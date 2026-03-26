package frc.robot.systems.shooter.hood;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
// HoodStates enum was migrated to command factories; constants use string IDs now.

public class HoodConstants {
    public static final BasicMotorHardware kHoodConfig = new BasicMotorHardware(
        55, // Motor CAN ID
        RobotConstants.kSubsystemsCANBus, // CANBus
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
        new PDConstants(300, 5), // Tuned for C3RBERUS!
        new MotionMagicConstants(4000.0 / 360.0, 4000. / 360.0, 0),  // Tuned for C3RBERUS!
        new ArmFeedforward(0.9, 0.06, 0, 0) // Tuned for C3RBERUS!
    );

    public static final RotationSoftLimits kHoodLimits = new RotationSoftLimits(
        Rotation2d.fromDegrees(0.02), 
        Rotation2d.fromDegrees(21.2) // Tuned for C3RBERUS!
    );

    public static final Rotation2d kAdjustStepAmount = Rotation2d.fromDegrees(2);
    public static final Rotation2d kTolerance = Rotation2d.fromDegrees(.5);
    public static final double kHoodLength = Units.feetToMeters(8.061);
    public static final double kHoodMass = Units.inchesToMeters(1.258);

    // Only add states that have constant setpoint throughout a real match.
    public static final HashMap<String, Supplier<Rotation2d>> kStateToSetpointMapHood = new HashMap<String, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Hood/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tTuningAmp = new LoggedTunableNumber("Hood/TuneAmperage", 0.0);
    public static final LoggedTunableNumber tTuningShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/TuningShotSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tMaxSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/MaxSetpointDegrees", kHoodLimits.forwardLimit().getDegrees());
    public static final LoggedTunableNumber tMidSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/MidSetpointDegrees", kHoodLimits.forwardLimit().getDegrees() / 2.0);
    public static final LoggedTunableNumber tMinSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/MinSetpointDegrees", kHoodLimits.backwardLimit().getDegrees());
    public static final LoggedTunableNumber tCloseShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/CloseShotSetpointDegrees", 7.0);
    public static final LoggedTunableNumber tTowerShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/TowerShotSetpointDegrees", 15.0);
    public static final LoggedTunableNumber tBumpShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/BumpShotSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tIncrementSpeedDPS = new LoggedTunableNumber("Hood/IncrementSpeedDPS", 1.0);

    static {
    kStateToSetpointMapHood.put("MAX", () -> Rotation2d.fromDegrees(tMaxSetpointDeg.get()));
    kStateToSetpointMapHood.put("MID", () -> Rotation2d.fromDegrees(tMidSetpointDeg.get()));
    kStateToSetpointMapHood.put("MIN", () -> Rotation2d.fromDegrees(tMinSetpointDeg.get()));
    kStateToSetpointMapHood.put("CLOSE_SHOT", () -> Rotation2d.fromDegrees(tCloseShotSetpointDeg.get()));
    kStateToSetpointMapHood.put("TOWER_SHOT", () -> Rotation2d.fromDegrees(tTowerShotSetpointDeg.get()));
    kStateToSetpointMapHood.put("BUMP_SHOT", () -> Rotation2d.fromDegrees(tBumpShotSetpointDeg.get()));
    kStateToSetpointMapHood.put("TUNING_SETPOINT", () -> Rotation2d.fromDegrees(tTuningShotSetpointDeg.get()));
    }

}
