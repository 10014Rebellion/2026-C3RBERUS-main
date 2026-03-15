package frc.robot.systems.shooter.fuelpump;

import java.util.HashMap;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;

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

    public static final HashMap<FuelPumpState, LoggedTunableNumber> kStateToTuneableFuelPump = new HashMap<FuelPumpState, LoggedTunableNumber>();

    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Shooter/FuelPump/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tIntakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/IntakeVolts", 10.014);
    public static final LoggedTunableNumber tOuttakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/OuttakeVolts", -10.014);

    static {
        kStateToTuneableFuelPump.put(FuelPumpState.INTAKE_VOLT, tIntakeVolts);
        kStateToTuneableFuelPump.put(FuelPumpState.OUTTAKE_VOLT, tOuttakeVolts);
    }
}
