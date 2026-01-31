package frc.robot.systems.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.systems.flywheels.FlywheelConstants.FlywheelHardwareConfiguration;

public class FlywheelKrakenIO implements FlywheelIO{
    private final TalonFX mFlywheelLeftMotor;
    private StatusSignal<Voltage> mFlywheelLeftMotorVoltage;
    private StatusSignal<AngularVelocity> mFlywheelLeftMotorVelocity;
    private StatusSignal<Current> mFlywheelLeftMotorSupplyCurrent;
    private StatusSignal<Current> mFlywheelLeftMotorStatorCurrent;
    private StatusSignal<Current> mFlywheelLeftMotorTorqueCurrent;
    private StatusSignal<Temperature> mFlywheelLeftMotorTempCelsius;
    private StatusSignal<AngularAcceleration> mFlywheelLeftMotorAccelerationMPSS;
    private Rotation2d mFlywheelLeftMotorPosition;

    private final TalonFX mFlywheelRightMotor;
    private StatusSignal<Voltage> mFlywheelRightMotorVoltage;
    private StatusSignal<AngularVelocity> mFlywheelRightMotorVelocity;
    private StatusSignal<Current> mFlywheelRightMotorSupplyCurrent;
    private StatusSignal<Current> mFlywheelRightMotorStatorCurrent;
    private StatusSignal<Current> mFlywheelRightMotorTorqueCurrent;
    private StatusSignal<Temperature> mFlywheelRightMotorTempCelsius;
    private StatusSignal<AngularAcceleration> mFlywheelRightMotorAccelerationMPSS;
    private Rotation2d mFlywheelRightMotorPosition;


    private final VelocityVoltage motorSetpointRequestLeft = new VelocityVoltage(0);
    private final VelocityVoltage motorSetpointRequestRight = new VelocityVoltage(0);
    
    public FlywheelKrakenIO(FlywheelHardwareConfiguration pLeftFlywheelHardwareConfiguration, FlywheelHardwareConfiguration pRightFlywheelHardwareConfiguration){
        /*should I config anything else? */        
        /*LEFT MOTOR CONFIGS*/
        mFlywheelLeftMotor = new TalonFX(pLeftFlywheelHardwareConfiguration.kMotorID(), pLeftFlywheelHardwareConfiguration.kCanBus());
        TalonFXConfiguration mLeftMotorConfig = new TalonFXConfiguration();
        mLeftMotorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kSmartCurrentLimit;
        mLeftMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kPeakVoltage;
        mLeftMotorConfig.Voltage.PeakReverseVoltage = -FlywheelConstants.kPeakVoltage;
        mLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mLeftMotorConfig.MotorOutput.Inverted = FlywheelConstants.kInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mLeftMotorConfig.Slot0.kP = FlywheelConstants.LeftControlConfig.motorController().getP();
        mLeftMotorConfig.Slot0.kI = FlywheelConstants.LeftControlConfig.motorController().getI();
        mLeftMotorConfig.Slot0.kD = FlywheelConstants.LeftControlConfig.motorController().getD();
        mFlywheelLeftMotor.getConfigurator().apply(mLeftMotorConfig);

        mFlywheelLeftMotorVoltage = mFlywheelLeftMotor.getMotorVoltage();
        mFlywheelLeftMotorVelocity = mFlywheelLeftMotor.getVelocity();
        mFlywheelLeftMotorSupplyCurrent = mFlywheelLeftMotor.getSupplyCurrent();
        mFlywheelLeftMotorStatorCurrent = mFlywheelLeftMotor.getStatorCurrent();
        mFlywheelLeftMotorTorqueCurrent = mFlywheelLeftMotor.getTorqueCurrent();
        mFlywheelLeftMotorTempCelsius = mFlywheelLeftMotor.getDeviceTemp();
        mFlywheelLeftMotorAccelerationMPSS = mFlywheelLeftMotor.getAcceleration();
        mFlywheelLeftMotorPosition = Rotation2d.fromRotations(mFlywheelLeftMotor.getPosition().getValueAsDouble());

        /*RIGHT MOTOR CONFIGS*/
        mFlywheelRightMotor = new TalonFX(pRightFlywheelHardwareConfiguration.kMotorID(), pRightFlywheelHardwareConfiguration.kCanBus());
        mFlywheelRightMotor.setControl(new Follower(mFlywheelLeftMotor.getDeviceID(), FlywheelConstants.mFollowerAlignment));
        TalonFXConfiguration mRightMotorConfig = new TalonFXConfiguration();
        mRightMotorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kSmartCurrentLimit;
        mRightMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kPeakVoltage;
        mRightMotorConfig.Voltage.PeakReverseVoltage = -FlywheelConstants.kPeakVoltage;
        mRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mRightMotorConfig.MotorOutput.Inverted = !FlywheelConstants.kInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mRightMotorConfig.Slot0.kP = FlywheelConstants.RightControlConfig.motorController().getP();
        mRightMotorConfig.Slot0.kI = FlywheelConstants.RightControlConfig.motorController().getI();
        mRightMotorConfig.Slot0.kD = FlywheelConstants.RightControlConfig.motorController().getD();
        mFlywheelRightMotor.getConfigurator().apply(mRightMotorConfig);

        mFlywheelRightMotorVoltage = mFlywheelRightMotor.getMotorVoltage();
        mFlywheelRightMotorVelocity = mFlywheelRightMotor.getVelocity();
        mFlywheelRightMotorVoltage = mFlywheelRightMotor.getMotorVoltage();
        mFlywheelRightMotorVelocity = mFlywheelRightMotor.getVelocity();
        mFlywheelRightMotorSupplyCurrent = mFlywheelRightMotor.getSupplyCurrent();
        mFlywheelRightMotorStatorCurrent = mFlywheelRightMotor.getStatorCurrent();
        mFlywheelRightMotorTorqueCurrent = mFlywheelRightMotor.getTorqueCurrent();
        mFlywheelRightMotorTempCelsius = mFlywheelRightMotor.getDeviceTemp();
        mFlywheelRightMotorAccelerationMPSS = mFlywheelRightMotor.getAcceleration();
        mFlywheelRightMotorPosition = Rotation2d.fromRotations(mFlywheelRightMotor.getPosition().getValueAsDouble());

    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        /*LEFT MOTOR LOGERS*/
        BaseStatusSignal.refreshAll(
            mFlywheelLeftMotorVoltage, 
            mFlywheelLeftMotorVelocity, 
            mFlywheelLeftMotorSupplyCurrent,
            mFlywheelLeftMotorStatorCurrent,
            mFlywheelLeftMotorTorqueCurrent,
            mFlywheelLeftMotorTempCelsius,
            mFlywheelLeftMotorAccelerationMPSS,
            mFlywheelRightMotorVoltage, 
            mFlywheelRightMotorVelocity, 
            mFlywheelRightMotorSupplyCurrent,
            mFlywheelRightMotorStatorCurrent,
            mFlywheelRightMotorTorqueCurrent,
            mFlywheelRightMotorTempCelsius,
            mFlywheelRightMotorAccelerationMPSS
        );
        inputs.iFlywheelLeftMotorVolts = mFlywheelLeftMotorVoltage.getValueAsDouble();
        inputs.iFlywheelLeftVelocityMPS = mFlywheelLeftMotorVelocity.getValueAsDouble();
        inputs.iFlywheelLeftSupplyCurrentAmps = mFlywheelLeftMotorSupplyCurrent.getValueAsDouble();
        inputs.iFlywheelLeftStatorCurrentAmps = mFlywheelLeftMotorStatorCurrent.getValueAsDouble();
        inputs.iFlywheelLeftTorqueCurrentAmps = mFlywheelLeftMotorTorqueCurrent.getValueAsDouble();
        inputs.iFlywheelLeftTemperatureCelsius = mFlywheelLeftMotorTempCelsius.getValueAsDouble();
        inputs.iFlywheelLeftAccelerationMPSS = mFlywheelLeftMotorAccelerationMPSS.getValueAsDouble();
        inputs.iFlywheelLeftPosition = mFlywheelLeftMotorPosition;

        /*RIGHT MOTOR LOGGERS*/
        inputs.iFlywheelRightMotorVolts = mFlywheelRightMotorVoltage.getValueAsDouble();
        inputs.iFlywheelRightVelocityMPS = mFlywheelRightMotorVelocity.getValueAsDouble();
        inputs.iFlywheelRightSupplyCurrentAmps = mFlywheelRightMotorSupplyCurrent.getValueAsDouble();
        inputs.iFlywheelRightStatorCurrentAmps = mFlywheelRightMotorStatorCurrent.getValueAsDouble();
        inputs.iFlywheelRightTorqueCurrentAmps = mFlywheelRightMotorTorqueCurrent.getValueAsDouble();
        inputs.iFlywheelRightTemperatureCelsius = mFlywheelRightMotorTempCelsius.getValueAsDouble();
        inputs.iFlywheelRightAccelerationMPSS = mFlywheelRightMotorAccelerationMPSS.getValueAsDouble();
        inputs.iFlywheelRightPosition = mFlywheelRightMotorPosition;

    }

 
    @Override
    public void setLeftFlywheelVolts(double volts) {
        mFlywheelLeftMotor.setVoltage(volts);
    }

    @Override
    public void setLeftFlywheelPID(double kP, double kD, double kV, double kA) {
        var slotConfigLeft = new Slot0Configs();
        slotConfigLeft.kP = kP;
        slotConfigLeft.kD = kD;
        slotConfigLeft.kV = kV;
        slotConfigLeft.kA = kA;
        mFlywheelLeftMotor.getConfigurator().apply(slotConfigLeft);
    }

    @Override
    public void setLeftFlywheelVelocity(AngularVelocity setpointRPS, double pFF){
        motorSetpointRequestLeft
            .withVelocity(setpointRPS)
            .withFeedForward(pFF)
            .withSlot(0);
        mFlywheelLeftMotor.setControl(motorSetpointRequestLeft);
    }
    
    @Override
    public void setRightFlywheelVolts(double volts) {
        mFlywheelRightMotor.setVoltage(volts);
    }

    @Override
    public void setRightFlywheelPID(double kP, double kD, double kV, double kA) {
        var slotConfigRight = new Slot0Configs();
        slotConfigRight.kP = kP;
        slotConfigRight.kD = kD;
        slotConfigRight.kV = kV;
        slotConfigRight.kA = kA;
        mFlywheelRightMotor.getConfigurator().apply(slotConfigRight);
    }

    @Override
    public void setRightFlywheelVelocity(AngularVelocity setpointRPS, double pFF){
        motorSetpointRequestRight
            .withVelocity(setpointRPS)
            .withFeedForward(pFF)
            .withSlot(0);
        mFlywheelRightMotor.setControl(motorSetpointRequestLeft);
    }
}
