package frc.robot.systems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;

public class FlywheelIOKrakenx44 implements FlywheelIO{
    private final TalonFX mFlywheelMotor;
    private final VoltageOut mFlywheelVoltageControl = new VoltageOut(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC mFlywheelVelocityControl = new MotionMagicVelocityTorqueCurrentFOC(0.0);
    private final StatusSignal<ControlModeValue> mFlywheelControlMode;
    private final StatusSignal<AngularVelocity> mFlywheelVelocityRPS;
    private final StatusSignal<Voltage> mFlywheelVoltage;
    private final StatusSignal<Current> mFlywheelSupplyCurrent;
    private final StatusSignal<Current> mFlywheelStatorCurrent;
    private final StatusSignal<Temperature> mFlywheelTempCelsius;
    private final StatusSignal<AngularAcceleration> mFlywheelAccelerationRPSS;
    private Follower mFollowerController = null;

    // FOLLOWER CONSTRUCTOR
    public FlywheelIOKrakenx44(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        this.mFollowerController = new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue()); 
    }
    
    // LEADER CONSTRUCTOR
    public FlywheelIOKrakenx44(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
    }

    private FlywheelIOKrakenx44(int pMotorID, BasicMotorHardware pHardware) {
        this.mFlywheelMotor = new TalonFX(pMotorID, pHardware.canBus());

        TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();

        FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.SupplyCurrentLimit = pHardware.currentLimit().supplyCurrentLimit();
        FlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.StatorCurrentLimit = pHardware.currentLimit().statorCurrentLimit();

        FlywheelConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
        FlywheelConfig.MotorOutput.Inverted = pHardware.direction();

        FlywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        FlywheelConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();

        mFlywheelMotor.getConfigurator().apply(FlywheelConfig);

        mFlywheelControlMode = mFlywheelMotor.getControlMode();
        mFlywheelVelocityRPS = mFlywheelMotor.getVelocity();
        mFlywheelAccelerationRPSS = mFlywheelMotor.getAcceleration();
        mFlywheelVoltage = mFlywheelMotor.getMotorVoltage();
        mFlywheelSupplyCurrent = mFlywheelMotor.getSupplyCurrent();
        mFlywheelStatorCurrent = mFlywheelMotor.getStatorCurrent();
        mFlywheelTempCelsius = mFlywheelMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mFlywheelControlMode,
            mFlywheelVelocityRPS, 
            mFlywheelAccelerationRPSS,
            mFlywheelVoltage,
            mFlywheelSupplyCurrent,
            mFlywheelStatorCurrent,
            mFlywheelTempCelsius);

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
            mFlywheelTempCelsius
        ).isOK();
        pInputs.iIsLeader = isLeader();
        pInputs.iFlywheelControlMode = mFlywheelControlMode.getValue().toString();
        pInputs.iFlywheelVelocityRPS = mFlywheelVelocityRPS.getValueAsDouble();
        pInputs.iFlywheelAccelerationRPSS = mFlywheelAccelerationRPSS.getValueAsDouble();
        pInputs.iFlywheelMotorVolts = mFlywheelVoltage.getValueAsDouble();
        pInputs.iFlywheelSupplyCurrentAmps = mFlywheelSupplyCurrent.getValueAsDouble();
        pInputs.iFlywheelStatorCurrentAmps = mFlywheelStatorCurrent.getValueAsDouble();
        pInputs.iFlywheelTempCelsius = mFlywheelTempCelsius.getValueAsDouble();
    }

    public boolean isLeader() {
        return (mFollowerController == null);
    }

    
    /**
     * Sets the motor velocity and acceleration using Motion Magic control.
     * If this motor is configured as a follower, it will ignore the velocity/acceleration
     * parameters and instead follow its leader motor. Control commands should be sent
     * to the leader motor instance, not to followers.
     * 
     * @param pVelocityRPS Target velocity in rotations per second
     * @param pAccelerationRPSS Target acceleration in rotations per second squared
     * @param pFeedforward Feedforward voltage to apply
     */
    @Override
    public void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {
        if(isLeader()) mFlywheelMotor.setControl(mFlywheelVelocityControl.withVelocity(pVelocityRPS).withAcceleration(pAccelerationRPSS).withFeedForward(pFeedforward));
        else mFlywheelMotor.setControl(mFollowerController);
    }

    /**
     * Sets the motor voltage output.
     * If this motor is configured as a follower, it will ignore the voltage parameter
     * and instead follow its leader motor. Control commands should be sent to the
     * leader motor instance, not to followers.
     * 
     * @param pVolts Target voltage to apply
     */
    @Override
    public void setMotorVolts(double pVolts) {
        if(isLeader()) mFlywheelMotor.setControl(mFlywheelVoltageControl.withOutput(pVolts));
        else mFlywheelMotor.setControl(mFollowerController);
    }

     /**
     * Stops the motor.
     * If this motor is configured as a follower, it will continue following its leader
     * motor rather than stopping independently. To stop a follower motor, send the stop
     * command to the leader motor instance.
     */
    @Override
    public void stopMotor() {
        if(isLeader()) mFlywheelMotor.stopMotor();
        else mFlywheelMotor.setControl(mFollowerController); 
    }
}