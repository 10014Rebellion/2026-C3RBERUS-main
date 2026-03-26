package frc.robot.systems.shooter.fuelpump;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.SimpleController;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
public class FuelPumpConstants {
    public static final double kToleranceRPS = 3.0;
    public static final Rotation2d kRPSForShooting = Rotation2d.fromRotations(80);

    public static final BasicMotorHardware kFuelPumpLeaderConfig = new BasicMotorHardware(
        53,
        RobotConstants.kSubsystemsCANBus,
        1,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        new CurrentLimits(40, 80)
    );

    public static final FollowerMotorHardware kFuelPumpFollowerConfig = new FollowerMotorHardware(
        54,
        kFuelPumpLeaderConfig,
        MotorAlignmentValue.Opposed
    );

    public static final SimpleController kFuelPumpControlConfig = new SimpleController(
        0, 
        new PDConstants(3, 0), 
        new SimpleMotorFeedforward(0, 9.7)
    );

    public static final HashMap<String, LoggedTunableNumber> kStateToTuneableFuelPumpVolts = new HashMap<String, LoggedTunableNumber>();
    public static final HashMap<String, Supplier<Rotation2d>> kStateToTuneableFuelPumpVelocity = new HashMap<String, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Shooter/FuelPump/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tIntakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/IntakeVolts", 6);
    public static final LoggedTunableNumber tOuttakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/OuttakeVolts", -10.014);
    public static final LoggedTunableNumber tIntakeVelocity = new LoggedTunableNumber("Shooter/FuelPump/DesiredVelocity/IntakeVelocity", 40.0);

    static {
    kStateToTuneableFuelPumpVolts.put("INTAKE_VOLT", tIntakeVolts);
    kStateToTuneableFuelPumpVolts.put("OUTTAKE_VOLT", tOuttakeVolts);
    kStateToTuneableFuelPumpVelocity.put("INTAKE_VELOCITY", () -> Rotation2d.fromRotations(tIntakeVelocity.get()));
    }
}
