package frc.robot.systems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;

public class HoodIOKrakenX44 implements HoodIO{
    private final TalonFX mHoodMotor;
    private final VoltageOut mHoodVoltageControl = new VoltageOut(0.0);
    private final PositionDutyCycle mHoodPositionControl = new PositionDutyCycle(0.0);
    private final StatusSignal<ControlModeValue> mHoodControlMode;
    private final StatusSignal<Angle> mHoodPosition;
    private final StatusSignal<AngularVelocity> mHoodVelocityRPS;
    private final StatusSignal<AngularAcceleration> mHoodAccelerationRPSS;
    private final StatusSignal<Voltage> mHoodVoltage;
    private final StatusSignal<Current> mHoodSupplyCurrent;
    private final StatusSignal<Current> mHoodStatorCurrent;
    private final StatusSignal<Temperature> mHoodTempCelsius;
    private final RotationSoftLimits mRotationSoftLimits;

    public HoodIOKrakenX44(BasicMotorHardware pMotorHardware, ArmControllerMotionMagic pController, RotationSoftLimits pSoftLimits) {
        this.mHoodMotor = new TalonFX(pMotorHardware.motorID(), pMotorHardware.canBus());
        this.mRotationSoftLimits = pSoftLimits;
        
        TalonFXConfiguration HoodConfig = new TalonFXConfiguration();

        HoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.SupplyCurrentLimit = pMotorHardware.currentLimit().supplyCurrentLimit();
        HoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.StatorCurrentLimit = pMotorHardware.currentLimit().statorCurrentLimit();

        HoodConfig.MotorOutput.NeutralMode = pMotorHardware.neutralMode();
        HoodConfig.MotorOutput.Inverted = pMotorHardware.direction();

        HoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        HoodConfig.Feedback.RotorToSensorRatio = 1;
        HoodConfig.Feedback.SensorToMechanismRatio = pMotorHardware.rotorToMechanismRatio();

        HoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        HoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mRotationSoftLimits.forwardLimit().getRotations();
        HoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        HoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mRotationSoftLimits.backwardLimit().getRotations();

        HoodConfig.MotionMagic.MotionMagicAcceleration = 0.0;
        HoodConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        HoodConfig.MotionMagic.MotionMagicJerk = 0.0;

        HoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        HoodConfig.Slot0.kP = pController.pdController().kP();
        HoodConfig.Slot0.kD = pController.pdController().kD();

        mHoodMotor.getConfigurator().apply(HoodConfig);

        mHoodControlMode = mHoodMotor.getControlMode();
        mHoodPosition = mHoodMotor.getPosition();
        mHoodVelocityRPS = mHoodMotor.getVelocity();
        mHoodAccelerationRPSS = mHoodMotor.getAcceleration();
        mHoodVoltage = mHoodMotor.getMotorVoltage();
        mHoodSupplyCurrent = mHoodMotor.getSupplyCurrent();
        mHoodStatorCurrent = mHoodMotor.getStatorCurrent();
        mHoodTempCelsius = mHoodMotor.getDeviceTemp();

        mHoodMotor.setPosition(0.0);

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
        pInputs.iHoodAngle = getPos();
        pInputs.iHoodVelocityRPS = mHoodVelocityRPS.getValueAsDouble();
        pInputs.iHoodAccelerationRPSS = mHoodAccelerationRPSS.getValueAsDouble();
        pInputs.iHoodMotorVolts = mHoodVoltage.getValueAsDouble();
        pInputs.iHoodSupplyCurrentAmps = mHoodSupplyCurrent.getValueAsDouble();
        pInputs.iHoodStatorCurrentAmps = mHoodStatorCurrent.getValueAsDouble();
        pInputs.iHoodTempCelsius = mHoodTempCelsius.getValueAsDouble();
        pInputs.iHoodReferenceValue = Rotation2d.fromRotations(mHoodMotor.getClosedLoopReference().getValueAsDouble());
    }

    private Rotation2d getPos() {
        return Rotation2d.fromRotations(mHoodPosition.getValueAsDouble());
    }

    @Override 
    public void setPDConstants(double pKP, double pKD) {
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mHoodMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = pCruiseVel;
        motionMagicConfig.MotionMagicAcceleration = pMaxAccel;
        motionMagicConfig.MotionMagicJerk = pMaxJerk;
        mHoodMotor.getConfigurator().apply(motionMagicConfig);
    }

    public void setMotorPosition(Rotation2d pSetpoint) {
        setMotorPosition(pSetpoint, 0);
    }
    @Override
    public void setMotorPosition(Rotation2d pSetpoint, double pArbFF) {
        mHoodMotor.setControl(mHoodPositionControl.withPosition(pSetpoint.getRotations()).withFeedForward(pArbFF));
    }

    /*
     * When called, it should be run periodically
     */
    @Override
    public void setMotorVolts(double pVolts) {
        mHoodMotor.setControl(mHoodVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mHoodMotor.stopMotor();
    }
}