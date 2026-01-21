package frc.robot.systems.flywheels;

import static frc.robot.systems.drive.DriveConstants.kCANBus;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.systems.drive.DriveConstants;
import frc.robot.systems.drive.modules.ModuleIO.ModuleInputs;
import frc.robot.systems.flywheels.FlywheelConstants.FlywheelHardwareConfiguration;

public class KrakenIO implements FlywheelIO{
    private final TalonFX mFlywheelLeftMotor;
    private StatusSignal<Voltage> mFlywheelLeftMotorVoltage;
    private StatusSignal<AngularVelocity> mFlywheelLeftMotorVelocity;


    private final TalonFX mFlywheelRightMotor;
    private StatusSignal<Voltage> mFlywheelRightMotorVoltage;
    private StatusSignal<AngularVelocity> mFlywheelRightMotorVelocity;

    private final VelocityVoltage motorSetpointRequestLeft = new VelocityVoltage(0);
    private final VelocityVoltage motorSetpointRequestRight = new VelocityVoltage(0);
    
    public KrakenIO(FlywheelHardwareConfiguration pFlywheelHardwareConfiguration){

        /*TODO: check if I need to add anything more for the configs! Also should I init the other logging inputs? What are the main things I need?*/
        
        /*Left MOTOR CONFIGS */
        mFlywheelLeftMotor = new TalonFX(pFlywheelHardwareConfiguration.kMotorID(), pFlywheelHardwareConfiguration.kCanBus());
        TalonFXConfiguration mLeftMotorConfig = new TalonFXConfiguration();
        mLeftMotorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kSmartCurrentLimit;
        mLeftMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kPeakVoltage;
        mLeftMotorConfig.Voltage.PeakReverseVoltage = -FlywheelConstants.kPeakVoltage;
        mLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mLeftMotorConfig.MotorOutput.Inverted = FlywheelConstants.kInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mFlywheelLeftMotor.getConfigurator().apply(mLeftMotorConfig);

        mFlywheelLeftMotorVoltage = mFlywheelLeftMotor.getMotorVoltage();
        mFlywheelLeftMotorVelocity = mFlywheelLeftMotor.getVelocity();


        /*Right MOTOR CONFIGS */
        mFlywheelRightMotor = new TalonFX(pFlywheelHardwareConfiguration.kMotorID(), pFlywheelHardwareConfiguration.kCanBus());
        TalonFXConfiguration mRightMotorConfig = new TalonFXConfiguration();
        mRightMotorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kSmartCurrentLimit;
        mRightMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kPeakVoltage;
        mRightMotorConfig.Voltage.PeakReverseVoltage = -FlywheelConstants.kPeakVoltage;
        mRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mRightMotorConfig.MotorOutput.Inverted = FlywheelConstants.kInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mFlywheelRightMotor.getConfigurator().apply(mRightMotorConfig);

        mFlywheelRightMotorVoltage = mFlywheelRightMotor.getMotorVoltage();
        mFlywheelRightMotorVelocity = mFlywheelRightMotor.getVelocity();

    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        /*TODO: init more values to log */
        inputs.iFlywheelLeftMotorVolts = mFlywheelLeftMotorVoltage.getValueAsDouble();
        inputs.iFlywheelLeftVelocityMPS = mFlywheelLeftMotorVelocity.getValueAsDouble();

        inputs.iFlywheelRightMotorVolts = mFlywheelRightMotorVoltage.getValueAsDouble();
        inputs.iFlywheelRightVelocityMPS = mFlywheelRightMotorVelocity.getValueAsDouble();

    }

 
    @Override
    public void setLeftFlywheelVolts(double volts) {
        mFlywheelLeftMotor.setVoltage(volts);
    }

    //TODO: see if I am doing this right
    @Override
    public void setLeftFlywheelPID(AngularVelocity setpointRPS) {
        var slotConfigLeft = new Slot0Configs();
        slotConfigLeft.kP = FlywheelConstants.kP;
        slotConfigLeft.kI = FlywheelConstants.kI;
        slotConfigLeft.kD = FlywheelConstants.kD;
        slotConfigLeft.kV = FlywheelConstants.kV;
        slotConfigLeft.kA = FlywheelConstants.kA;
        mFlywheelLeftMotor.getConfigurator().apply(slotConfigLeft);
        motorSetpointRequestLeft.withVelocity(setpointRPS);

        mFlywheelLeftMotor.setControl(motorSetpointRequestLeft);

    }
    
    @Override
    public void setRightFlywheelVolts(double volts) {
        mFlywheelRightMotor.setVoltage(volts);
    }

    //TODO: NOT SURE IF THIS METHOD WORKS RIGHT!!!
    @Override
    public void setRightFlywheelPID(AngularVelocity setpointRPS) {
        var slotConfigRight = new Slot0Configs();
        slotConfigRight.kP = FlywheelConstants.kP;
        slotConfigRight.kI = FlywheelConstants.kI;
        slotConfigRight.kD = FlywheelConstants.kD;
        slotConfigRight.kV = FlywheelConstants.kV;
        slotConfigRight.kA = FlywheelConstants.kA;
        mFlywheelRightMotor.getConfigurator().apply(slotConfigRight);

        motorSetpointRequestRight.withVelocity(setpointRPS);

        mFlywheelLeftMotor.setControl(motorSetpointRequestRight);
    }


    


}
