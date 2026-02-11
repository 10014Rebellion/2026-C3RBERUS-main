package frc.robot.systems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class IntakePivotIOKrakenX44 implements IntakePivotIO{
    private final TalonFX mIntakePivotMotor;
    private final VoltageOut mIntakePivotVoltageControl = new VoltageOut(0.0);
    
    private final StatusSignal<AngularVelocity> mIntakePivotVelocityMPS;
    private final StatusSignal<Voltage> mIntakePivotVoltage;
    private final StatusSignal<Current> mIntakePivotSupplyCurrent;
    private final StatusSignal<Current> mIntakePivotStatorCurrent;
    private final StatusSignal<Temperature> mIntakePivotTempCelsius;
    private final StatusSignal<AngularAcceleration> mIntakePivotAccelerationMPSS;
    
    public IntakePivotIOKrakenX44(BasicMotorHardware pConfig) {
        mIntakePivotMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var IntakeConfig = new TalonFXConfiguration();

        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        IntakeConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        IntakeConfig.MotorOutput.Inverted = pConfig.direction();

        IntakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        IntakeConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        mIntakePivotVelocityMPS = mIntakePivotMotor.getVelocity();
        mIntakePivotAccelerationMPSS = mIntakePivotMotor.getAcceleration();
        mIntakePivotVoltage = mIntakePivotMotor.getMotorVoltage();
        mIntakePivotSupplyCurrent = mIntakePivotMotor.getSupplyCurrent();
        mIntakePivotStatorCurrent = mIntakePivotMotor.getStatorCurrent();
        mIntakePivotTempCelsius = mIntakePivotMotor.getDeviceTemp();
        
        mIntakePivotMotor.getConfigurator().apply(IntakeConfig);
    }

    @Override
    public void updateInputs(IntakePivotInputs pInputs) {
        pInputs.iIsIntakePivotConnected = BaseStatusSignal.refreshAll(
            mIntakePivotVelocityMPS,
            mIntakePivotAccelerationMPSS,
            mIntakePivotVoltage,
            mIntakePivotSupplyCurrent,
            mIntakePivotStatorCurrent,
            mIntakePivotTempCelsius

        ).isOK();
        pInputs.iIntakePivotVelocityMPS = mIntakePivotVelocityMPS.getValueAsDouble();
        pInputs.iIntakePivotAccelerationMPSS = mIntakePivotAccelerationMPSS.getValueAsDouble();
        pInputs.iIntakePivotMotorVolts = mIntakePivotVoltage.getValueAsDouble();
        pInputs.iIntakePivotSupplyCurrentAmps = mIntakePivotSupplyCurrent.getValueAsDouble();
        pInputs.iIntakePivotStatorCurrentAmps = mIntakePivotStatorCurrent.getValueAsDouble();
        pInputs.iIntakePivotTempCelsius = mIntakePivotTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mIntakePivotMotor.setControl(mIntakePivotVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mIntakePivotMotor.stopMotor();
    }

}