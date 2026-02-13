package frc.robot.systems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.encoder.DetachedEncoder.Model;
import com.revrobotics.encoder.config.DetachedEncoderConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.MaxSplineEncoderHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.robot.systems.intake.IntakeConstants.PivotConstants;

public class IntakePivotIOKrakenX44 implements IntakePivotIO{
    private final TalonFX mIntakePivotMotor;
    private final DetachedEncoder mIntakePivotEncoder;
    private final VoltageOut mIntakePivotVoltageControl = new VoltageOut(0.0);
    private final MotionMagicDutyCycle mIntakePivotRotationControl = new MotionMagicDutyCycle(0.0);
    
    private final StatusSignal<Angle> mIntakePivotRotation;
    private final StatusSignal<AngularVelocity> mIntakePivotVelocityRPS;
    private final StatusSignal<AngularAcceleration> mIntakePivotAccelerationRPSS;

    private final StatusSignal<Voltage> mIntakePivotVoltage;
    private final StatusSignal<Current> mIntakePivotSupplyCurrent;
    private final StatusSignal<Current> mIntakePivotStatorCurrent;
    private final StatusSignal<Temperature> mIntakePivotTempCelsius;

    private final RotationSoftLimits mLimits;
    
    public IntakePivotIOKrakenX44(BasicMotorHardware pConfig, MaxSplineEncoderHardware pEncoderConfig, RotationSoftLimits pLimits) {
        this.mLimits = pLimits;

        // Encoder
        mIntakePivotEncoder = new DetachedEncoder(pEncoderConfig.canID(), Model.MAXSplineEncoder){};
        DetachedEncoderConfig encoderConfig = new DetachedEncoderConfig();
        encoderConfig.dutyCycleZeroCentered(pEncoderConfig.zeroCentered());
        encoderConfig.inverted(pEncoderConfig.inverted());
        encoderConfig.dutyCycleOffset((float) pEncoderConfig.offset());
        encoderConfig.angleConversionFactor(1);
        encoderConfig.averageDepth(64);
        encoderConfig.positionConversionFactor(1);
        encoderConfig.velocityConversionFactor(1);
        encoderConfig.signals.encoderAnglePeriodMs(50);
        encoderConfig.signals.encoderPositionPeriodMs(50);
        encoderConfig.signals.encoderVelocityPeriodMs(50);
        mIntakePivotEncoder.configure(encoderConfig, ResetMode.kResetSafeParameters);

        // Motor
        mIntakePivotMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var IntakeConfig = new TalonFXConfiguration();

        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        IntakeConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        IntakeConfig.MotorOutput.Inverted = pConfig.direction();

        IntakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // Okay so im like 99% sure this will not work... but unfortunately im still gonna try it.
        IntakeConfig.Feedback.FeedbackRemoteSensorID = pEncoderConfig.canID();
        IntakeConfig.Feedback.SensorToMechanismRatio = pEncoderConfig.gearRatio();
        IntakeConfig.Feedback.RotorToSensorRatio = pConfig.rotorToMechanismRatio() / pEncoderConfig.gearRatio();

        IntakeConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        IntakeConfig.Slot0.kP = PivotConstants.kPivotController.pdController().kP();
        IntakeConfig.Slot0.kD = PivotConstants.kPivotController.pdController().kD();
        IntakeConfig.Slot0.kS = PivotConstants.kPivotController.feedforward().getKs();
        IntakeConfig.Slot0.kG = PivotConstants.kPivotController.feedforward().getKg();
        IntakeConfig.Slot0.kV = PivotConstants.kPivotController.feedforward().getKv();
        IntakeConfig.Slot0.kA = PivotConstants.kPivotController.feedforward().getKa();
        IntakeConfig.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.kPivotController.motionMagicConstants().maxVelocity();
        IntakeConfig.MotionMagic.MotionMagicAcceleration = PivotConstants.kPivotController.motionMagicConstants().maxAcceleration();
        IntakeConfig.MotionMagic.MotionMagicJerk = PivotConstants.kPivotController.motionMagicConstants().maxJerk();

        mIntakePivotMotor.getConfigurator().apply(IntakeConfig);

        mIntakePivotRotation = mIntakePivotMotor.getPosition();
        mIntakePivotVelocityRPS = mIntakePivotMotor.getVelocity();
        mIntakePivotAccelerationRPSS = mIntakePivotMotor.getAcceleration();
        mIntakePivotVoltage = mIntakePivotMotor.getMotorVoltage();
        mIntakePivotSupplyCurrent = mIntakePivotMotor.getSupplyCurrent();
        mIntakePivotStatorCurrent = mIntakePivotMotor.getStatorCurrent();
        mIntakePivotTempCelsius = mIntakePivotMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mIntakePivotVelocityRPS,
            mIntakePivotAccelerationRPSS, 
            mIntakePivotVoltage,
            mIntakePivotSupplyCurrent,
            mIntakePivotStatorCurrent,
            mIntakePivotTempCelsius
        );

        mIntakePivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakePivotInputs pInputs) {
        pInputs.iIsIntakePivotConnected = BaseStatusSignal.refreshAll(
            mIntakePivotRotation,
            mIntakePivotVelocityRPS,
            mIntakePivotAccelerationRPSS,
            mIntakePivotVoltage,
            mIntakePivotSupplyCurrent,
            mIntakePivotStatorCurrent,
            mIntakePivotTempCelsius
        ).isOK();
        pInputs.iIntakePivotRotation = getPos();
        pInputs.iIntakePivotVelocityRPS = mIntakePivotVelocityRPS.getValueAsDouble();
        pInputs.iIntakePivotAccelerationRPSS = mIntakePivotAccelerationRPSS.getValueAsDouble();
        pInputs.iIntakePivotMotorVolts = mIntakePivotVoltage.getValueAsDouble();
        pInputs.iIntakePivotSupplyCurrentAmps = mIntakePivotSupplyCurrent.getValueAsDouble();
        pInputs.iIntakePivotStatorCurrentAmps = mIntakePivotStatorCurrent.getValueAsDouble();
        pInputs.iIntakePivotTempCelsius = mIntakePivotTempCelsius.getValueAsDouble();
    }

    private Rotation2d getPos() {
        return Rotation2d.fromRotations(mIntakePivotRotation.getValueAsDouble());
    }

    @Override 
    public void setPDConstants(double pKP, double pKD) {
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mIntakePivotMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = pCruiseVel;
        motionMagicConfig.MotionMagicAcceleration = pMaxAccel;
        motionMagicConfig.MotionMagicJerk = pMaxJerk;
        mIntakePivotMotor.getConfigurator().apply(motionMagicConfig);
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mIntakePivotMotor.setControl(mIntakePivotVoltageControl.withOutput(pVolts));
        enforceSoftLimits();
    }

    @Override
    public void setMotorRot(Rotation2d pRot, double feedforward) {
        mIntakePivotMotor.setControl(mIntakePivotRotationControl.withPosition(pRot.getRotations()).withFeedForward(0));
        enforceSoftLimits();
    }

    @Override
    public void enforceSoftLimits() {
        double currentRotation = getPos().getRotations();
        if((currentRotation > mLimits.forwardLimit().getRotations() && mIntakePivotVoltage.getValueAsDouble() > 0) || 
           (currentRotation < mLimits.backwardLimit().getRotations() && mIntakePivotVoltage.getValueAsDouble() < 0)) stopMotor();
    }

    @Override
    public void stopMotor() {
        mIntakePivotMotor.stopMotor();
    }

}