package frc.robot.systems.conveyor;

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

public class ConveyorIOKrakenx44 implements ConveyorIO{
    private final TalonFX mConveyorMotor;
    private final VoltageOut mConveyorVoltageControl = new VoltageOut(0.0);
    
    private final StatusSignal<AngularVelocity> mConveyorVelocityMPS;
    private final StatusSignal<Voltage> mConveyorVoltage;
    private final StatusSignal<Current> mConveyorSupplyCurrent;
    private final StatusSignal<Current> mConveyorStatorCurrent;
    private final StatusSignal<Temperature> mConveyorTempCelsius;
    private final StatusSignal<AngularAcceleration> mConveyorAccelerationMPSS;
    
    public ConveyorIOKrakenx44(BasicMotorHardware pConfig) {
        mConveyorMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var ConveyorConfig = new TalonFXConfiguration();

        ConveyorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        ConveyorConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        ConveyorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ConveyorConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        ConveyorConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        ConveyorConfig.MotorOutput.Inverted = pConfig.direction();

        ConveyorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        ConveyorConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        mConveyorVelocityMPS = mConveyorMotor.getVelocity();
        mConveyorAccelerationMPSS = mConveyorMotor.getAcceleration();
        mConveyorVoltage = mConveyorMotor.getMotorVoltage();
        mConveyorSupplyCurrent = mConveyorMotor.getSupplyCurrent();
        mConveyorStatorCurrent = mConveyorMotor.getStatorCurrent();
        mConveyorTempCelsius = mConveyorMotor.getDeviceTemp();
        
        mConveyorMotor.getConfigurator().apply(ConveyorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mConveyorAccelerationMPSS,
            mConveyorVelocityMPS,
            mConveyorVoltage,
            mConveyorSupplyCurrent,
            mConveyorStatorCurrent,
            mConveyorTempCelsius);

        mConveyorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ConveyorInputs pInputs) {
        pInputs.iIsConveyorConnected = BaseStatusSignal.refreshAll(
            mConveyorVelocityMPS,
            mConveyorAccelerationMPSS,
            mConveyorVoltage,
            mConveyorSupplyCurrent,
            mConveyorStatorCurrent,
            mConveyorTempCelsius

        ).isOK();
        pInputs.iConveyorVelocityMPS = mConveyorVelocityMPS.getValueAsDouble();
        pInputs.iConveyorAccelerationMPSS = mConveyorAccelerationMPSS.getValueAsDouble();
        pInputs.iConveyorMotorVolts = mConveyorVoltage.getValueAsDouble();
        pInputs.iConveyorSupplyCurrentAmps = mConveyorSupplyCurrent.getValueAsDouble();
        pInputs.iConveyorStatorCurrentAmps = mConveyorStatorCurrent.getValueAsDouble();
        pInputs.iConveyorTempCelsius = mConveyorTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mConveyorMotor.setControl(mConveyorVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mConveyorMotor.stopMotor();
    }

}