package frc.robot.systems.shooter.fuelpump;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.robot.systems.shooter.ShooterConstants.FlywheelConstants;

public class FuelPumpIOSim implements FuelPumpIO {

    private DCMotorSim mFuelPumpMotor;
    private boolean mIsFollower;
    private double mAppliedVoltage;
    private final PIDController mFuelPumpController;

    public FuelPumpIOSim(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        mIsFollower = true;
    }

    public FuelPumpIOSim(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
        mIsFollower = false;
    }

    
    private FuelPumpIOSim(int pMotorID, BasicMotorHardware pHardware){
        mFuelPumpMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, pHardware.rotorToMechanismRatio()),
            DCMotor.getKrakenX60Foc(1).withReduction(pHardware.rotorToMechanismRatio()),
            0.0,
            0.0
        ); 

        mFuelPumpController = new PIDController(FlywheelConstants.kFlywheelControlConfig.pdController().kP(), 0.0, FlywheelConstants.kFlywheelControlConfig.pdController().kD());
    }
    
    public void updateInputs(FuelPumpInputs pInputs) {
        mFuelPumpMotor.update(0.02);
        pInputs.iFuelPumpAccelerationRPSS = (Math.PI * 2) / mFuelPumpMotor.getAngularAccelerationRadPerSecSq();
        pInputs.iFuelPumpMotorVolts = mAppliedVoltage;
        pInputs.iFuelPumpStatorCurrentAmps = Math.abs(mFuelPumpMotor.getCurrentDrawAmps());
        pInputs.iFuelPumpSupplyCurrentAmps = 0.0;
        pInputs.iFuelPumpTempCelsius = 0.0;
        pInputs.iFuelPumpVelocityGoal = mFuelPumpMotor.getAngularVelocityRPM() * 60;
        pInputs.iIsFuelPumpConnected = true;
    } 
    
    public void setMotorVolts(double pVolts) {
        mAppliedVoltage = MathUtil.clamp(pVolts, -12.0, 12.0);

        if(mIsFollower){
            mAppliedVoltage *= -1;
        }

        mFuelPumpMotor.setInputVoltage(mAppliedVoltage);
    }

    public void setPDConstants(int pSlot, double pKP, double pKD) {
        mFuelPumpController.setPID(pKP, 0.0, pKD);
    }
    
    public void setMotorVelocity(double pVelocityRPS, double pFeedforward) {
        setMotorVolts(mFuelPumpController.calculate(mFuelPumpMotor.getAngularVelocityRPM() * 60.0, pVelocityRPS) + pFeedforward);
    }

    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        return;
    }

    public void enforceFollower() {
        return;
    }

    public void stopMotor() {
        setMotorVolts(0.0);
    }
}
