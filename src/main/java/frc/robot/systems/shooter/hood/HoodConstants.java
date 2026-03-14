package frc.robot.systems.shooter.hood;

import java.util.HashMap;
import java.util.function.Supplier;

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
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;

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
        new PDConstants(22, 0), // Tuned for C3RBERUS!
        new MotionMagicConstants(100, 1000, 0),  // Tuned for C3RBERUS!
        new ArmFeedforward(0, 0.06, 0, 0) // Tuned for C3RBERUS!
    );

    public static final RotationSoftLimits kHoodLimits = new RotationSoftLimits(
        Rotation2d.fromDegrees(0.02), 
        Rotation2d.fromDegrees(21.2) // Tuned for C3RBERUS!
    );

    public static final Rotation2d kAdjustStepAmount = Rotation2d.fromDegrees(2);
    public static final double kToleranceRotations = 0.5;
    private static Rotation2d kRotationIncrementSetpoint = Rotation2d.fromDegrees(0.0);
    
    public static Rotation2d getRotationalIncrementSetpoint() {
        return kRotationIncrementSetpoint;
    }

    public static void setRotationalIncrementSetpoint(Rotation2d pRotationIncrementSetpoint) {
        kRotationIncrementSetpoint = pRotationIncrementSetpoint;
    }

    private static Rotation2d kHoldAngleSetpoint = Rotation2d.fromDegrees(0.0);

    public static Rotation2d getHoldAngleSetpoint() {
        return kHoldAngleSetpoint;
    }

    public static void setHoodAngleSetpoint(Rotation2d pHoldAngleSetpoint) {
        kHoldAngleSetpoint = pHoldAngleSetpoint;
    }

    private static Rotation2d kAutoShootSetpoint = Rotation2d.fromDegrees(0.0);

    public static Rotation2d getAutoShootSetpoint() {
        return kAutoShootSetpoint;
    }

    public static void setAutoShootSetpoint(Rotation2d pAutoShootSetpoint) {
        kAutoShootSetpoint = pAutoShootSetpoint;
    }

    public static final HashMap<HoodStates, Supplier<Rotation2d>> kStateToSetpointMapHood = new HashMap<HoodStates, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Hood/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tTuningAmp = new LoggedTunableNumber("Hood/TuneAmperage", 0.0);
    public static final LoggedTunableNumber tMaxSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/MaxSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tMidSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/MidSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tMinSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/MinSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tCloseShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/CloseShotSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tTowerShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/TowerShotSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tBumpShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/BumpShotSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tTuningShotSetpointDeg  = new LoggedTunableNumber("Hood/Setpoint/TuningShotSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tIncrementSpeedDPS = new LoggedTunableNumber("Hood/IncrementSpeedDPS", 1.0);

    static {
        kStateToSetpointMapHood.put(HoodStates.MAX, () -> Rotation2d.fromDegrees(tMaxSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.MID, () -> Rotation2d.fromDegrees(tMidSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.MIN, () -> Rotation2d.fromDegrees(tMinSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.CLOSE_SHOT, () -> Rotation2d.fromDegrees(tCloseShotSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.TOWER_SHOT, () -> Rotation2d.fromDegrees(tTowerShotSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.BUMP_SHOT, () -> Rotation2d.fromDegrees(tBumpShotSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.INCREMENTING, () -> kRotationIncrementSetpoint);
        kStateToSetpointMapHood.put(HoodStates.DECREMENTING, () -> kRotationIncrementSetpoint);
        kStateToSetpointMapHood.put(HoodStates.STEP_INCREMENT, () -> kRotationIncrementSetpoint);
        kStateToSetpointMapHood.put(HoodStates.STEP_DECREMENT, () -> kRotationIncrementSetpoint);
        kStateToSetpointMapHood.put(HoodStates.HOLD_POSITION, () -> kHoldAngleSetpoint);
        kStateToSetpointMapHood.put(HoodStates.AUTO_SHOOT, () -> kAutoShootSetpoint);
    }

}
