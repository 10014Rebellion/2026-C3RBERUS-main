// REBELLION 10014

package frc.robot.systems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;

public class ClimbIOKrakenx44 implements ClimbIO {
    private final TalonFX mClimbMotor;
    private final VoltageOut mClimbVoltageControl = new VoltageOut(0.0);

    private final StatusSignal<AngularVelocity> mClimbVelocityMPS;
    private final StatusSignal<Voltage> mClimbVoltage;
    private final StatusSignal<Current> mClimbSupplyCurrent;
    private final StatusSignal<Current> mClimbStatorCurrent;
    private final StatusSignal<Temperature> mClimbTempCelsius;
    private final StatusSignal<AngularAcceleration> mClimbAccelerationMPSS;
    private final StatusSignal<Angle> mClimbPosition;

    public ClimbIOKrakenx44(BasicMotorHardware pConfig, PositionSoftLimits pSoftLimits) {
        mClimbMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var ClimbConfig = new TalonFXConfiguration();

        ClimbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        ClimbConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        ClimbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ClimbConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        ClimbConfig.Voltage.PeakForwardVoltage = 12;
        ClimbConfig.Voltage.PeakReverseVoltage = -12;

        ClimbConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        ClimbConfig.MotorOutput.Inverted = pConfig.direction();

        ClimbConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        ClimbConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        mClimbVelocityMPS = mClimbMotor.getVelocity();
        mClimbAccelerationMPSS = mClimbMotor.getAcceleration();
        mClimbVoltage = mClimbMotor.getMotorVoltage();
        mClimbSupplyCurrent = mClimbMotor.getSupplyCurrent();
        mClimbStatorCurrent = mClimbMotor.getStatorCurrent();
        mClimbTempCelsius = mClimbMotor.getDeviceTemp();
        mClimbPosition = mClimbMotor.getPosition();

        mClimbMotor.getConfigurator().apply(ClimbConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mClimbVelocityMPS, 
            mClimbAccelerationMPSS,
            mClimbVoltage,
            mClimbSupplyCurrent,
            mClimbStatorCurrent,
            mClimbTempCelsius,
            mClimbPosition);
        mClimbMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimbInputs pInputs) {
        pInputs.iIsClimbConnected = BaseStatusSignal.refreshAll(
            mClimbVelocityMPS,
            mClimbAccelerationMPSS,
            mClimbVoltage,
            mClimbSupplyCurrent,
            mClimbStatorCurrent,
            mClimbTempCelsius
        ).isOK();
        pInputs.iClimbVelocityMPS = mClimbVelocityMPS.getValueAsDouble();
        pInputs.iClimbAccelerationMPSS = mClimbAccelerationMPSS.getValueAsDouble();
        pInputs.iClimbMotorVolts = mClimbVoltage.getValueAsDouble();
        pInputs.iClimbSupplyCurrentAmps = mClimbSupplyCurrent.getValueAsDouble();
        pInputs.iClimbStatorCurrentAmps = mClimbStatorCurrent.getValueAsDouble();
        pInputs.iClimbTempCelsius = mClimbTempCelsius.getValueAsDouble();
        pInputs.iClimbPositionMeters = mClimbPosition.getValueAsDouble();
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mClimbMotor.setControl(mClimbVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mClimbMotor.stopMotor();
    }
}
