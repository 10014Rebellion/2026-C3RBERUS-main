package frc.robot.systems.intake.pivot;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CANdiEncoder;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.robot.Constants;
import frc.robot.systems.intake.IntakeConstants.PivotConstants;

public class IntakePivotIOKrakenX44 implements IntakePivotIO{
    private final TalonFX mIntakePivotMotor;

    private final CANdi mEncoder;
    private final StatusSignal<Angle> mIntakeEncoderAbsolutePosition;

    // private final DetachedEncoder mIntakePivotEncoder;
    private final VoltageOut mIntakePivotVoltageControl = new VoltageOut(0.0);
    private final TorqueCurrentFOC mIntakePivotAmpsControl = new TorqueCurrentFOC(0.0);
    private final PositionVoltage mIntakePivotRotationControl = new PositionVoltage(0.0);
    
    private final StatusSignal<Angle> mIntakePivotRotation;
    private final StatusSignal<AngularVelocity> mIntakePivotVelocityRPS;
    private final StatusSignal<AngularAcceleration> mIntakePivotAccelerationRPSS;

    private final StatusSignal<Voltage> mIntakePivotVoltage;
    private final StatusSignal<Current> mIntakePivotSupplyCurrent;
    private final StatusSignal<Current> mIntakePivotStatorCurrent;
    private final StatusSignal<Temperature> mIntakePivotTempCelsius;
    private final StatusSignal<Double> mIntakePivotReferencePosition;
    private final StatusSignal<Double> mIntakePivotReferencePositionSlope;

    private final RotationSoftLimits mLimits;
    private Rotation2d mPivotPos = Rotation2d.kZero;
    
    public IntakePivotIOKrakenX44(BasicMotorHardware pConfig, CANdiEncoder pEncoderConfig, RotationSoftLimits pLimits) {
        mEncoder = new CANdi(pEncoderConfig.canID(), Constants.kSubsystemsCANBus);
        CANdiConfiguration mEncoderConfiguration = new CANdiConfiguration();
        mEncoderConfiguration.PWM1.AbsoluteSensorOffset = -pEncoderConfig.offset().getRotations();
        mEncoder.getConfigurator().apply(mEncoderConfiguration);

        mIntakeEncoderAbsolutePosition = mEncoder.getPWM1Position();

        this.mLimits = pLimits;

        // Motor
        mIntakePivotMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var IntakeConfig = new TalonFXConfiguration();

        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        IntakeConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        IntakeConfig.MotorOutput.Inverted = pConfig.direction();

        IntakeConfig.Feedback.FeedbackSensorSource = pEncoderConfig.feedbackPort();
        IntakeConfig.Feedback.FeedbackRemoteSensorID = pEncoderConfig.canID();
        IntakeConfig.Feedback.SensorToMechanismRatio = pEncoderConfig.sensorToMechanismRatio();
        IntakeConfig.Feedback.RotorToSensorRatio = pConfig.rotorToMechanismRatio();

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
        mIntakePivotReferencePosition = mIntakePivotMotor.getClosedLoopReference();
        mIntakePivotReferencePositionSlope = mIntakePivotMotor.getClosedLoopReferenceSlope();

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
        pInputs.iIsEncoderConnected = BaseStatusSignal.refreshAll(mIntakeEncoderAbsolutePosition).isOK();
        pInputs.iEncoderPosition = Rotation2d.fromRadians(mIntakeEncoderAbsolutePosition.getValue().in(Radian));

        pInputs.iIsIntakePivotConnected = BaseStatusSignal.refreshAll(
            mIntakePivotRotation,
            mIntakePivotVelocityRPS,
            mIntakePivotAccelerationRPSS,
            mIntakePivotVoltage,
            mIntakePivotSupplyCurrent,
            mIntakePivotStatorCurrent,
            mIntakePivotTempCelsius,
            mIntakePivotReferencePosition,
            mIntakePivotReferencePositionSlope
        ).isOK();
        pInputs.iIntakePivotRotation = 
            Rotation2d.fromRotations(mIntakePivotRotation.getValueAsDouble());
            // .minus(mOffset);
        mPivotPos = Rotation2d.fromRotations(mIntakePivotRotation.getValueAsDouble());
        pInputs.iIntakePivotVelocityRPS = Rotation2d.fromRotations(mIntakePivotVelocityRPS.getValueAsDouble());
        pInputs.iIntakePivotAccelerationRPSS = Rotation2d.fromRotations(mIntakePivotAccelerationRPSS.getValueAsDouble());
        pInputs.iIntakePivotMotorVolts = mIntakePivotVoltage.getValueAsDouble();
        pInputs.iIntakePivotSupplyCurrentAmps = mIntakePivotSupplyCurrent.getValueAsDouble();
        pInputs.iIntakePivotStatorCurrentAmps = mIntakePivotStatorCurrent.getValueAsDouble();
        pInputs.iIntakePivotTempCelsius = mIntakePivotTempCelsius.getValueAsDouble();
        pInputs.iIntakeClosedLoopReference = 
            Rotation2d.fromRotations(mIntakePivotReferencePosition.getValueAsDouble());
            // .minus(mOffset);
        pInputs.iIntakeClosedLoopReferenceSlope = 
            Rotation2d.fromRotations(mIntakePivotReferencePositionSlope.getValueAsDouble());
            // .minus(mOffset);
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
    public void setMotorAmps(double pAmps){
        mIntakePivotMotor.setControl(mIntakePivotAmpsControl.withOutput(pAmps));
    }

    @Override
    public void setMotorRot(Rotation2d pRot, double feedforward) {
        mIntakePivotMotor.setControl(mIntakePivotRotationControl.withPosition(pRot.getRotations()).withFeedForward(feedforward));
        enforceSoftLimits();
    }
    

    @Override
    public void enforceSoftLimits() {
        double currentRotation = mPivotPos.getRotations();
        if((currentRotation > mLimits.forwardLimit().getRotations() && mIntakePivotVoltage.getValueAsDouble() > 0) || 
           (currentRotation < mLimits.backwardLimit().getRotations() && mIntakePivotVoltage.getValueAsDouble() < 0)) stopMotor();
    }

    @Override
    public void stopMotor() {
        mIntakePivotMotor.stopMotor();
    }

}