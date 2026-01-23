package frc.robot.systems.IntakeRoller.IOlayers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.systems.IntakeRoller.IntakeRollerConstants.IntakeRollerConfiguration;
import frc.robot.systems.IntakeRoller.IntakeRollerConstants.IntakeRollerHardware;
import frc.robot.systems.IntakeRoller.IOlayers.IntakeRollerIO.IntakeRollerIO.IntakeRollerIOInputs;

public class IntakeRollerIOTalonFX implements IntakeRollerIO{
    private final TalonFX kOuterMotor;
    private final TalonFX kInnerMotor;
    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    
    private IntakeRollerConfiguration indexerConfiguration;

    public IntakeRollerIOTalonFX(IntakeRollerHardware hardware, IntakeRollerConfiguration indexerConfiguration){
        kOuterMotor = new TalonFX(hardware.kMotorPort(), hardware.kCanBus());
        this.indexerConfiguration = indexerConfiguration;

        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = indexerConfiguration.kSmartLimit();
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = indexerConfiguration.kSmartLimit();
        motorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorConfiguration.Voltage.PeakReverseVoltage = -12;

        motorConfiguration.MotorOutput.NeutralMode = indexerConfiguration.kIdleMode();
        motorConfiguration.MotorOutput.Inverted =  indexerConfiguration.kInverted() 
            ? InvertedValue.CounterClockwise_Positive 
            : InvertedValue.Clockwise_Positive;

        kMotor.getConfigurator().apply(motorConfiguration, 1.0);

    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs){
  
        inputs.isMotorConnected = true;

        inputs.appliedVoltage = kMotor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = 0.0;
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = kMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorOutput = 0.0;
    }

    @Override
    public void setVoltage(double pVolts){
        pVolts = MathUtil.clamp(pVolts, -12.0, 12.0);
        kMotor.setVoltage(pVolts);
    }

    @Override
    public void stop(){
        kMotor.stopMotor();
    }

    
}
    

