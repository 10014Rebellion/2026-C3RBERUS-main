package frc.robot.systems.shooter.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.robot.systems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO{

    private DCMotorSim mFlywheelMotor;
    private double mAppliedVoltage;
    private final PIDController mFlywheelController;
    
    // LEADER CONSTRUCTOR
    public FlywheelIOSim(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
    }

    private FlywheelIOSim(int pMotorID, BasicMotorHardware pHardware){
        mFlywheelMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, pHardware.rotorToMechanismRatio()),
            DCMotor.getKrakenX60Foc(1).withReduction(pHardware.rotorToMechanismRatio()),
            0.0,
            0.0
        ); 

        mFlywheelController = new PIDController(FlywheelConstants.kFlywheelControlConfig.pdController().kP(), 0.0, FlywheelConstants.kFlywheelControlConfig.pdController().kD());
    }

    public void updateInputs(FlywheelInputs pInputs) {
        mFlywheelMotor.update(0.02);
        pInputs.iFlywheelAccelerationRPSS = (2*Math.PI) / mFlywheelMotor.getAngularAccelerationRadPerSecSq();
        pInputs.iFlywheelMotorVolts = mAppliedVoltage;
        pInputs.iFlywheelStatorCurrentAmps = Math.abs(mFlywheelMotor.getCurrentDrawAmps());
        pInputs.iFlywheelSupplyCurrentAmps = 0.0;
        pInputs.iFlywheelTempCelsius = 0.0;
        pInputs.iFlywheelVelocityRPS = mFlywheelMotor.getAngularVelocityRPM() * 60.0;
        pInputs.iIsFlywheelConnected = true;
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
