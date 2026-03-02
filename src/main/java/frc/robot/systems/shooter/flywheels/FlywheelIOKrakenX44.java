package frc.robot.systems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.telemetry.Telemetry;
import frc.robot.logging.MotorErrors;
import frc.robot.systems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOKrakenX44 implements FlywheelIO{
    private final TalonFX mFlywheelMotor;
    private final VoltageOut mFlywheelVoltageControl = new VoltageOut(0.0);
    private final VelocityTorqueCurrentFOC mFlywheelVelocityControl = new VelocityTorqueCurrentFOC(0.0);
    private final StatusSignal<ControlModeValue> mFlywheelControlMode;
    private final StatusSignal<AngularVelocity> mFlywheelVelocityRPS;
    private final StatusSignal<Voltage> mFlywheelVoltage;
    private final StatusSignal<Current> mFlywheelSupplyCurrent;
    private final StatusSignal<Current> mFlywheelStatorCurrent;
    private final StatusSignal<Temperature> mFlywheelTempCelsius;
    private final StatusSignal<AngularAcceleration> mFlywheelAccelerationRPSS;
    private final StatusSignal<Double> mFlywheelClosedLoopReference;
    private Follower mFollowerController = null;


    // FOLLOWER CONSTRUCTOR
    public FlywheelIOKrakenX44(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        this.mFollowerController = new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue()); 
        enforceFollower();
    }
    
    // LEADER CONSTRUCTOR
    public FlywheelIOKrakenX44(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
    }

    private FlywheelIOKrakenX44(int pMotorID, BasicMotorHardware pHardware) {
        this.mFlywheelMotor = new TalonFX(pMotorID, pHardware.canBus());

        TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();

        FlywheelConfig.Voltage.PeakForwardVoltage = 12;
        FlywheelConfig.Voltage.PeakReverseVoltage = -12;

        FlywheelConfig.Slot0.kP = FlywheelConstants.kFlywheelControlConfig.pdController().kP();
        FlywheelConfig.Slot0.kD = FlywheelConstants.kFlywheelControlConfig.pdController().kD();

        FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.SupplyCurrentLimit = pHardware.currentLimit().supplyCurrentLimit();
        FlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.StatorCurrentLimit = pHardware.currentLimit().statorCurrentLimit();

        FlywheelConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
        FlywheelConfig.MotorOutput.Inverted = pHardware.direction();
        
        FlywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        FlywheelConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();
        

        FlywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        FlywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        FlywheelConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
        FlywheelConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

        mFlywheelMotor.getConfigurator().apply(FlywheelConfig);

        mFlywheelControlMode = mFlywheelMotor.getControlMode();
        mFlywheelVelocityRPS = mFlywheelMotor.getVelocity();
        mFlywheelAccelerationRPSS = mFlywheelMotor.getAcceleration();
        mFlywheelVoltage = mFlywheelMotor.getMotorVoltage();
        mFlywheelSupplyCurrent = mFlywheelMotor.getSupplyCurrent();
        mFlywheelStatorCurrent = mFlywheelMotor.getStatorCurrent();
        mFlywheelTempCelsius = mFlywheelMotor.getDeviceTemp();
        mFlywheelClosedLoopReference = mFlywheelMotor.getClosedLoopReference();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mFlywheelControlMode,
            mFlywheelVelocityRPS, 
            mFlywheelAccelerationRPSS,
            mFlywheelVoltage,
            mFlywheelSupplyCurrent,
            mFlywheelStatorCurrent,
            mFlywheelTempCelsius,
            mFlywheelClosedLoopReference
        );

        mFlywheelMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelInputs pInputs) {
        pInputs.iIsFlywheelConnected = BaseStatusSignal.refreshAll(
            mFlywheelControlMode,
            mFlywheelVelocityRPS,
            mFlywheelAccelerationRPSS,
            mFlywheelVoltage,
            mFlywheelSupplyCurrent,
            mFlywheelStatorCurrent,
            mFlywheelTempCelsius,
            mFlywheelClosedLoopReference
        ).isOK();
        pInputs.iIsLeader = isLeader();
        pInputs.iFlywheelControlMode = mFlywheelControlMode.getValue().toString();
        pInputs.iFlywheelRotorVelocityRPS = Rotation2d.fromRotations(mFlywheelVelocityRPS.getValueAsDouble());
        pInputs.iFlywheelRotorAccelerationRPSS = mFlywheelAccelerationRPSS.getValueAsDouble();
        pInputs.iFlywheelMotorVolts = mFlywheelVoltage.getValueAsDouble();
        pInputs.iFlywheelSupplyCurrentAmps = mFlywheelSupplyCurrent.getValueAsDouble();
        pInputs.iFlywheelStatorCurrentAmps = mFlywheelStatorCurrent.getValueAsDouble();
        pInputs.iFlywheelTempCelsius = mFlywheelTempCelsius.getValueAsDouble();
        pInputs.iFlywheelClosedLoopReference = Rotation2d.fromRotations(mFlywheelClosedLoopReference.getValueAsDouble());
    }

    public boolean isLeader() {
        return mFollowerController == null;
    }

    @Override 
    public void setPDConstants(double pKP, double pKD) {
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mFlywheelMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void enforceFollower() {
        if(!isLeader()) mFlywheelMotor.setControl(mFollowerController);
        else Telemetry.reportIssue(new MotorErrors.EnforcingLeaderAsFollower(this));
    }

    @Override
    public void setMotorVel(double pVelocityRPS, double pFeedforward) {
        if(isLeader()) mFlywheelMotor.setControl(mFlywheelVelocityControl.withVelocity(pVelocityRPS).withFeedForward(pFeedforward));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        if(isLeader()) mFlywheelMotor.setControl(mFlywheelVoltageControl.withOutput(pVolts));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void stopMotor() {
        if(isLeader()) mFlywheelMotor.stopMotor(); 
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }
}