package frc.robot.systems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.robot.systems.shooter.ShooterConstants.HoodConstants;

public class HoodIOSim implements HoodIO {

    private DCMotorSim mHoodMotor;
    private double mAppliedVoltage;
    private final PIDController mHoodController;
    private RotationSoftLimits mSoftLimits;

    public HoodIOSim(BasicMotorHardware pHardware, RotationSoftLimits pSoftLimits) {
        mHoodMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, pHardware.rotorToMechanismRatio()),
            DCMotor.getKrakenX60Foc(1).withReduction(pHardware.rotorToMechanismRatio()),
            0.0,
            0.0
        ); 

        mHoodController = new PIDController(HoodConstants.kHoodControlConfig.pdController().kP(), 0.0, HoodConstants.kHoodControlConfig.pdController().kD());
        mSoftLimits = pSoftLimits;
    }
    
    public void updateInputs(HoodInputs pInputs) {
        mHoodMotor.update(0.02);
        pInputs.iHoodAccelerationRPSS = (2 * Math.PI) / mHoodMotor.getAngularAccelerationRadPerSecSq();
        pInputs.iHoodAngle = getHoodAngle();
        pInputs.iHoodMotorVolts = mAppliedVoltage;
        pInputs.iHoodStatorCurrentAmps = Math.abs(mHoodMotor.getCurrentDrawAmps());
        pInputs.iHoodSupplyCurrentAmps = 0.0;
        pInputs.iHoodTempCelsius = 0.0;
        pInputs.iHoodVelocityRPS = mHoodMotor.getAngularVelocityRPM() * 60.0;
        pInputs.iIsHoodConnected = true;
    }

    private Rotation2d getHoodAngle(){        
        return Rotation2d.fromRotations(mHoodMotor.getAngularPositionRotations());
    }
    
    public void setMotorPosition(Rotation2d pRotationSP, double pFeedforward) {
        setMotorVolts(mHoodController.calculate(getHoodAngle().getRotations(), pRotationSP.getRotations()) + pFeedforward);
    }

    public void setPDConstants(double pKP, double pKD) {
        mHoodController.setPID(pKP, 0.0, pKD);
    }

    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        return;
    }

    public void enforceSoftLimits() {
        double currentRotation = getHoodAngle().getRotations();
        if((currentRotation > mSoftLimits.forwardLimit().getRotations() && mAppliedVoltage > 0) || 
           (currentRotation < mSoftLimits.backwardLimit().getRotations() && mAppliedVoltage < 0)) stopMotor();
    }

    public void setMotorVolts(double pVolts) {
        mAppliedVoltage = MathUtil.clamp(pVolts, -12.0, 12.0);
        mHoodMotor.setInputVoltage(mAppliedVoltage);
    }

    public void stopMotor() {
        setMotorVolts(0.0);
    }
}
