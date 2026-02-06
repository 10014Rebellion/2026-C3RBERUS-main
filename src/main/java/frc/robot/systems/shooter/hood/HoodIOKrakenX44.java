package frc.robot.systems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;

public class HoodIOKrakenX44 implements HoodIO{
    private final TalonFX mHoodMotor;
    private final VoltageOut mHoodVoltageControl = new VoltageOut(0.0);
    private final MotionMagicTorqueCurrentFOC mHoodPositionControl = new MotionMagicTorqueCurrentFOC(0.0);
    private final StatusSignal<ControlModeValue> mHoodControlMode;
    private final StatusSignal<Angle> mHoodPosition;
    private final StatusSignal<AngularVelocity> mHoodVelocityRPS;
    private final StatusSignal<AngularAcceleration> mHoodAccelerationRPSS;
    private final StatusSignal<Voltage> mHoodVoltage;
    private final StatusSignal<Current> mHoodSupplyCurrent;
    private final StatusSignal<Current> mHoodStatorCurrent;
    private final StatusSignal<Temperature> mHoodTempCelsius;

    public HoodIOKrakenX44(BasicMotorHardware pMotorHardware, RotationSoftLimits pSoftLimits) {
        this.mHoodMotor = new TalonFX(pMotorHardware.motorID(), pMotorHardware.canBus());

        TalonFXConfiguration HoodConfig = new TalonFXConfiguration();

        HoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.SupplyCurrentLimit = pMotorHardware.currentLimit().supplyCurrentLimit();
        HoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.StatorCurrentLimit = pMotorHardware.currentLimit().statorCurrentLimit();

        HoodConfig.MotorOutput.NeutralMode = pMotorHardware.neutralMode();
        HoodConfig.MotorOutput.Inverted = pMotorHardware.direction();

        HoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        HoodConfig.Feedback.SensorToMechanismRatio = pMotorHardware.rotorToMechanismRatio();

        mHoodMotor.getConfigurator().apply(HoodConfig);

        mHoodControlMode = mHoodMotor.getControlMode();
        mHoodPosition = mHoodMotor.getPosition();
        mHoodVelocityRPS = mHoodMotor.getVelocity();
        mHoodAccelerationRPSS = mHoodMotor.getAcceleration();
        mHoodVoltage = mHoodMotor.getMotorVoltage();
        mHoodSupplyCurrent = mHoodMotor.getSupplyCurrent();
        mHoodStatorCurrent = mHoodMotor.getStatorCurrent();
        mHoodTempCelsius = mHoodMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mHoodControlMode,
            mHoodPosition, 
            mHoodVelocityRPS,
            mHoodAccelerationRPSS,
            mHoodVoltage,
            mHoodSupplyCurrent,
            mHoodStatorCurrent,
            mHoodTempCelsius
        );

        mHoodMotor.optimizeBusUtilization();
    }


    @Override
    public void updateInputs(HoodInputs pInputs) {
        pInputs.iIsHoodConnected = BaseStatusSignal.refreshAll(
            mHoodControlMode,
            mHoodPosition,
            mHoodVelocityRPS,
            mHoodAccelerationRPSS,
            mHoodVoltage,
            mHoodSupplyCurrent,
            mHoodStatorCurrent,
            mHoodTempCelsius
        ).isOK();
        pInputs.iHoodControlMode = mHoodControlMode.getValue().toString();
        pInputs.iHoodAngle = Rotation2d.fromRotations(mHoodPosition.getValueAsDouble());
        pInputs.iHoodVelocityRPS = mHoodVelocityRPS.getValueAsDouble();
        pInputs.iHoodAccelerationRPSS = mHoodAccelerationRPSS.getValueAsDouble();
        pInputs.iHoodMotorVolts = mHoodVoltage.getValueAsDouble();
        pInputs.iHoodSupplyCurrentAmps = mHoodSupplyCurrent.getValueAsDouble();
        pInputs.iHoodStatorCurrentAmps = mHoodStatorCurrent.getValueAsDouble();
        pInputs.iHoodTempCelsius = mHoodTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorPosition(double pPosition, double pFeedforward) {
        mHoodMotor.setControl(mHoodPositionControl.withPosition(pPosition).withFeedForward(pFeedforward));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mHoodMotor.setControl(mHoodVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mHoodMotor.stopMotor();
    }
}