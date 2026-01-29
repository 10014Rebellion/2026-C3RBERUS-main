package frc.robot.systems.IntakeRoller.IOlayers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.systems.IntakeRoller.IntakeRollerConstants;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
    
    private final TalonFX innerRoller;
    private final TalonFX outerRoller;
    private final DigitalInput fuelSensor;
    
    public IntakeRollerIOTalonFX() {
        innerRoller = new TalonFX(
            IntakeRollerConstants.kInnerIntakeRollerMotorID, 
            "drivetrain"
        );
        outerRoller = new TalonFX(
            IntakeRollerConstants.kOuterIntakeRollerMotorID, 
            "drivetrain"
        );
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.kSmartCurrentLimit;
        
        config.MotorOutput.Inverted = IntakeRollerConstants.kInnerInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
        innerRoller.getConfigurator().apply(config, 1);
        
        config.MotorOutput.Inverted = IntakeRollerConstants.kOuterInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
        outerRoller.getConfigurator().apply(config, 1);
        
        fuelSensor = new DigitalInput(IntakeRollerConstants.kOuterSensorDIOPort);
    }
    
    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.innerRollerVoltage = innerRoller.getMotorVoltage().getValueAsDouble();
        inputs.innerRollerCurrent = innerRoller.getSupplyCurrent().getValueAsDouble();
        inputs.innerRollerVelocity = innerRoller.getVelocity().getValueAsDouble();
        inputs.innerRollerTemperature = innerRoller.getDeviceTemp().getValueAsDouble();
        
        inputs.outerRollerVoltage = outerRoller.getMotorVoltage().getValueAsDouble();
        inputs.outerRollerCurrent = outerRoller.getSupplyCurrent().getValueAsDouble();
        inputs.outerRollerVelocity = outerRoller.getVelocity().getValueAsDouble();
        inputs.outerRollerTemperature = outerRoller.getDeviceTemp().getValueAsDouble();
        
        inputs.fuelDetected = !fuelSensor.get();
    }
    
    @Override
    public void setInnerVoltage(double volts) {
        innerRoller.setVoltage(volts);
    }
    
    @Override
    public void setOuterVoltage(double volts) {
        outerRoller.setVoltage(volts);
    }
    
    @Override
    public void stop() {
        innerRoller.setVoltage(0);
        outerRoller.setVoltage(0);
    }
}