package frc.robot.systems.shooter.fuelpump;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
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
import frc.lib.telemetry.Telemetry;
import frc.robot.errors.MotorErrors;
import frc.robot.systems.shooter.ShooterConstants.FuelPumpConstants;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class FuelPumpIOKrakenX44 implements FuelPumpIO{
    private final TalonFX mFuelPumpMotor;
    
    private final VoltageOut mFuelPumpVoltageControl = new VoltageOut(0.0);
    private final VelocityDutyCycle mFuelPumpVelocityControl = new VelocityDutyCycle(0.0);

    private double mFuelPumpVelocityGoal = 0.0;

    private final StatusSignal<ControlModeValue> mFuelPumpControlMode;
    private final StatusSignal<AngularVelocity> mFuelPumpVelocityRPS;
    private final StatusSignal<Voltage> mFuelPumpVoltage;
    private final StatusSignal<Current> mFuelPumpSupplyCurrent;
    private final StatusSignal<Current> mFuelPumpStatorCurrent;
    private final StatusSignal<Temperature> mFuelPumpTempCelsius;
    private final StatusSignal<AngularAcceleration> mFuelPumpAccelerationRPSS;
    private Follower mFollowerController = null;

    // FOLLOWER CONSTRUCTOR
    public FuelPumpIOKrakenX44(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        this.mFollowerController = new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue());
    }
    
    // LEADER CONSTRUCTOR
    public FuelPumpIOKrakenX44(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
    }

    private FuelPumpIOKrakenX44(int pMotorID, BasicMotorHardware pHardware) {
        this.mFuelPumpMotor = new TalonFX(pMotorID, pHardware.canBus());

        TalonFXConfiguration FuelPumpConfig = new TalonFXConfiguration();

        FuelPumpConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        FuelPumpConfig.CurrentLimits.SupplyCurrentLimit = pHardware.currentLimit().supplyCurrentLimit();
        FuelPumpConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        FuelPumpConfig.CurrentLimits.StatorCurrentLimit = pHardware.currentLimit().statorCurrentLimit();

        FuelPumpConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
        FuelPumpConfig.MotorOutput.Inverted = pHardware.direction();

        FuelPumpConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        FuelPumpConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();

        mFuelPumpMotor.getConfigurator().apply(FuelPumpConfig);

        setPDConstants(FuelPumpConstants.kFuelPumpControlConfig.slot(), FuelPumpConstants.kFuelPumpControlConfig.pdController().kP(), FuelPumpConstants.kFuelPumpControlConfig.pdController().kD());

        mFuelPumpControlMode = mFuelPumpMotor.getControlMode();
        mFuelPumpVelocityRPS = mFuelPumpMotor.getVelocity();
        mFuelPumpAccelerationRPSS = mFuelPumpMotor.getAcceleration();
        mFuelPumpVoltage = mFuelPumpMotor.getMotorVoltage();
        mFuelPumpSupplyCurrent = mFuelPumpMotor.getSupplyCurrent();
        mFuelPumpStatorCurrent = mFuelPumpMotor.getStatorCurrent();
        mFuelPumpTempCelsius = mFuelPumpMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mFuelPumpControlMode,
            mFuelPumpVelocityRPS, 
            mFuelPumpAccelerationRPSS,
            mFuelPumpVoltage,
            mFuelPumpSupplyCurrent,
            mFuelPumpStatorCurrent,
            mFuelPumpTempCelsius
        );

        mFuelPumpMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FuelPumpInputs pInputs) {
        pInputs.iIsFuelPumpConnected = BaseStatusSignal.refreshAll(
            mFuelPumpControlMode,
            mFuelPumpVelocityRPS,
            mFuelPumpAccelerationRPSS,
            mFuelPumpVoltage,
            mFuelPumpSupplyCurrent,
            mFuelPumpStatorCurrent,
            mFuelPumpTempCelsius
        ).isOK();
        pInputs.iFuelPumpControlMode = mFuelPumpControlMode.getValue().toString();
        pInputs.iFuelPumpVelocityRPS = mFuelPumpVelocityRPS.getValueAsDouble();
        pInputs.iFuelPumpAccelerationRPSS = mFuelPumpAccelerationRPSS.getValueAsDouble();
        pInputs.iFuelPumpMotorVolts = mFuelPumpVoltage.getValueAsDouble();
        pInputs.iFuelPumpSupplyCurrentAmps = mFuelPumpSupplyCurrent.getValueAsDouble();
        pInputs.iFuelPumpStatorCurrentAmps = mFuelPumpStatorCurrent.getValueAsDouble();
        pInputs.iFuelPumpTempCelsius = mFuelPumpTempCelsius.getValueAsDouble();
        pInputs.iFuelPumpVelocityGoal = getVelocityGoal();

    }

    private boolean isLeader() {
        return mFollowerController == null;
    }

    @Override 
    public void setPDConstants(int pSlot, double pKP, double pKD) {
        SlotConfigs slotConfig = new SlotConfigs();
        slotConfig.SlotNumber = pSlot;
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mFuelPumpMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void enforceFollower() {
        if(!isLeader()) mFuelPumpMotor.setControl(mFollowerController);
        else Telemetry.reportIssue(new MotorErrors.EnforcingLeaderAsFollower(this));
    }

    @Override
    public void setMotorVelocity(double pVelocityRPS, double pFeedforward) {
        if(isLeader()) mFuelPumpMotor.setControl(mFuelPumpVelocityControl.withVelocity(pVelocityRPS).withFeedForward(pFeedforward));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));

        mFuelPumpVelocityGoal = pVelocityRPS;
        Logger.recordOutput("FuelPump/Goal", pVelocityRPS);
    }

    @Override
    public void setMotorVolts(double pVolts) {
        if(isLeader()) mFuelPumpMotor.setControl(mFuelPumpVoltageControl.withOutput(pVolts));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void stopMotor() {
        if(isLeader()) mFuelPumpMotor.stopMotor();
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    /**
     * @return double value for velocity setpoint. Will be zero if voltage request is sent.
     */
    @SuppressWarnings("unlikely-arg-type")
    private double getVelocityGoal(){
        if(mFuelPumpControlMode.equals(ControlModeValue.VoltageOut)){
            return 0.0;
        }

        else{
            return mFuelPumpVelocityGoal;
        }
    }
}