package frc.robot.systems.shooter.indexers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class IndexerIOKrakenx44 implements IndexerIO{
    private final TalonFX mIndexerMotor;
    private final VoltageOut mIndexerVoltageControl = new VoltageOut(0.0);
    private final VelocityDutyCycle mIndexerVelocityControl = new VelocityDutyCycle(0.0);
    private final StatusSignal<ControlModeValue> mIndexerControlMode;
    private final StatusSignal<AngularVelocity> mIndexerVelocityRPS;
    private final StatusSignal<Voltage> mIndexerVoltage;
    private final StatusSignal<Current> mIndexerSupplyCurrent;
    private final StatusSignal<Current> mIndexerStatorCurrent;
    private final StatusSignal<Temperature> mIndexerTempCelsius;
    private final StatusSignal<AngularAcceleration> mIndexerAccelerationRPSS;
    private final boolean mIsLeader;

    // FOLLOWER CONSTRUCTOR
    public IndexerIOKrakenx44(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig(), false);
        mIndexerMotor.setControl(new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue()));
    }
    
    // LEADER CONSTRUCTOR
    public IndexerIOKrakenx44(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig, true);
    }

    private IndexerIOKrakenx44(int pMotorID, BasicMotorHardware pHardware, boolean pIsLeader) {
        this.mIndexerMotor = new TalonFX(pMotorID, pHardware.canBus());
        this.mIsLeader = pIsLeader;

        TalonFXConfiguration IndexerConfig = new TalonFXConfiguration();

        IndexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IndexerConfig.CurrentLimits.SupplyCurrentLimit = pHardware.currentLimit().supplyCurrentLimit();
        IndexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IndexerConfig.CurrentLimits.StatorCurrentLimit = pHardware.currentLimit().statorCurrentLimit();

        IndexerConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
        IndexerConfig.MotorOutput.Inverted = pHardware.direction();

        IndexerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        IndexerConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();

        mIndexerMotor.getConfigurator().apply(IndexerConfig);

        mIndexerControlMode = mIndexerMotor.getControlMode();
        mIndexerVelocityRPS = mIndexerMotor.getVelocity();
        mIndexerAccelerationRPSS = mIndexerMotor.getAcceleration();
        mIndexerVoltage = mIndexerMotor.getMotorVoltage();
        mIndexerSupplyCurrent = mIndexerMotor.getSupplyCurrent();
        mIndexerStatorCurrent = mIndexerMotor.getStatorCurrent();
        mIndexerTempCelsius = mIndexerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mIndexerControlMode,
            mIndexerVelocityRPS, 
            mIndexerAccelerationRPSS,
            mIndexerVoltage,
            mIndexerSupplyCurrent,
            mIndexerStatorCurrent,
            mIndexerTempCelsius
        );

        mIndexerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerInputs pInputs) {
        pInputs.iIsIndexerConnected = BaseStatusSignal.refreshAll(
            mIndexerControlMode,
            mIndexerVelocityRPS,
            mIndexerAccelerationRPSS,
            mIndexerVoltage,
            mIndexerSupplyCurrent,
            mIndexerStatorCurrent,
            mIndexerTempCelsius
        ).isOK();
        pInputs.iIndexerControlMode = mIndexerControlMode.getValue().toString();
        pInputs.iIndexerVelocityRPS = mIndexerVelocityRPS.getValueAsDouble();
        pInputs.iIndexerAccelerationRPSS = mIndexerAccelerationRPSS.getValueAsDouble();
        pInputs.iIndexerMotorVolts = mIndexerVoltage.getValueAsDouble();
        pInputs.iIndexerSupplyCurrentAmps = mIndexerSupplyCurrent.getValueAsDouble();
        pInputs.iIndexerStatorCurrentAmps = mIndexerStatorCurrent.getValueAsDouble();
        pInputs.iIndexerTempCelsius = mIndexerTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorVelocity(double pVelocityRPS, double pFeedforward) {
        if(mIsLeader) mIndexerMotor.setControl(mIndexerVelocityControl.withVelocity(pVelocityRPS).withFeedForward(pFeedforward));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        if(mIsLeader) mIndexerMotor.setControl(mIndexerVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        if(mIsLeader) mIndexerMotor.stopMotor();
    }
}