package frc.robot.systems.IntakeRoller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase{
    private final TalonFX mIntakeRollerMotor;
    private boolean mDisableIR;
    
    private final DigitalInput mFuelSensor;
    
    private boolean mBackTriggered;
    private Timer mFuelStuckTimer;

    public void IntakeRollerConstants(){
    this.mIntakeRollerMotor = new TalonFX(IntakeRollerConstants.kMotorID, "drivetrain");
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.kSmartCurrentLimit;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.kSmartCurrentLimit;
    rollerConfig.Voltage.PeakForwardVoltage = 12;
    rollerConfig.Voltage.PeakReverseVoltage = -12;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = IntakeRollerConstants.kInverted
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;

    mIntakeRollerMotor.getConfigurator().apply(rollerConfig, 1.0);
    }
}

public record setVoltsIntakeRoller(double volts) {
    mIntakeRollerMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
}

@Override
public record periodic() {
    SmartDashboard.putNumber("Intake/Roller Current", mIntakeRollerMotor.getStatorCurrent().getValueAsDouble());
}

