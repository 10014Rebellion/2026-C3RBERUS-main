package frc.robot.systems.shooter.fuelpump;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.robot.systems.shooter.ShooterConstants.FlywheelConstants;

public class FuelPumpIOSim implements FuelPumpIO {

    private DCMotorSim mFlywheelMotor;
    private boolean mIsFollower;
    private double mAppliedVoltage;
    private final PIDController mFlywheelController;

    // FOLLOWER CONSTRUCTOR
    public FuelPumpIOSim(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        mIsFollower = true;
    }
    
    // LEADER CONSTRUCTOR
    public FuelPumpIOSim(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
        mIsFollower = false;
    }

    private FuelPumpIOSim(int pMotorID, BasicMotorHardware pHardware){
        mFlywheelMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, pHardware.rotorToMechanismRatio()),
            DCMotor.getKrakenX60Foc(1).withReduction(pHardware.rotorToMechanismRatio()),
            0.0,
            0.0
        ); 

        mFlywheelController = new PIDController(FlywheelConstants.kFlywheelControlConfig.pdController().kP(), 0.0, FlywheelConstants.kFlywheelControlConfig.pdController().kD());
    }

    public void updateInputs(FuelPumpInputs pInputs) {
        mFlywheelMotor.update(0.02);
        pInputs.iIsFuelPumpLeader = !mIsFollower;
        pInputs.iIsFuelPumpConnected = true;
        pInputs.iFuelPumpAccelerationRPSS = (2*Math.PI) / mFlywheelMotor.getAngularAccelerationRadPerSecSq();
        pInputs.iFuelPumpMotorVolts = mAppliedVoltage;
        pInputs.iFuelPumpStatorCurrentAmps = Math.abs(mFlywheelMotor.getCurrentDrawAmps());
        pInputs.iFuelPumpSupplyCurrentAmps = 0.0;
        pInputs.iFuelPumpTempCelsius = 0.0;
        pInputs.iFuelPumpVelocityRPS = Rotation2d.fromRotations(mFlywheelMotor.getAngularVelocityRPM() / 60.0);
    }

    public void setPDConstants(double pKP, double pKD) {
        mFlywheelController.setPID(pKP, 0.0, pKD);
    }

    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        return;
    }

    public void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {
        setMotorVolts(mFlywheelController.calculate(mFlywheelMotor.getAngularVelocityRPM() * 60.0, pVelocityRPS) + pFeedforward);
    }

    // Inverts voltage if follower as the rest of the close loop control runs of this same method //
    public void setMotorVolts(double pVolts) {
        mAppliedVoltage = MathUtil.clamp(pVolts, -12.0, 12.0);
        mFlywheelMotor.setInputVoltage(mAppliedVoltage);
    }

    public void stopMotor() {
        setMotorVolts(0.0);
    }

    // Will not be needed as I just used a 
    public void enforceFollower() {
        return;
    }
}
