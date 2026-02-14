package frc.robot.systems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.MaxSplineEncoderHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;

import frc.robot.systems.intake.IntakeConstants.PivotConstants;

public class IntakePivotIOSim implements IntakePivotIO {

    private final DCMotorSim mIntakePivotMotor;
    private final PIDController mIntakePivotController;
    private double mAppliedVolts = 0.0;
    private final RotationSoftLimits mLimits;

    public IntakePivotIOSim(BasicMotorHardware pConfig, MaxSplineEncoderHardware pEncoderConfig, RotationSoftLimits pLimits) {
        mIntakePivotMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, pConfig.rotorToMechanismRatio()),
            DCMotor.getKrakenX60Foc(1).withReduction(pConfig.rotorToMechanismRatio()),
            0.0,
            0.0
        );

        mIntakePivotController = new PIDController(PivotConstants.kPivotController.pdController().kP(), 0.0, PivotConstants.kPivotController.pdController().kD());
        mLimits = pLimits;
    }

    public void updateInputs(IntakePivotInputs pInputs) {
        mIntakePivotMotor.update(0.02);
        pInputs.iIsIntakePivotConnected = true;
        pInputs.iIntakePivotRotation = getPos();
        pInputs.iIntakePivotVelocityRPS = mIntakePivotMotor.getAngularVelocityRPM() / 60.0;
        pInputs.iIntakePivotAccelerationRPSS = Math.PI *2 / (mIntakePivotMotor.getAngularAccelerationRadPerSecSq());
        pInputs.iIntakePivotMotorVolts = mAppliedVolts;
        pInputs.iIntakePivotSupplyCurrentAmps = 0.0;
        pInputs.iIntakePivotStatorCurrentAmps = 0.0;
        pInputs.iIntakePivotTempCelsius = 0.0;
    }

    public Rotation2d getPos(){
        return Rotation2d.fromRotations(mIntakePivotMotor.getAngularPositionRotations());
    }

    public void setMotorVolts(double pVolts) {
        mAppliedVolts = MathUtil.clamp(pVolts, -12.0, 12.0);
        mIntakePivotMotor.setInputVoltage(pVolts);
    }   

    public void setMotorRot(Rotation2d pRot, double feedforward) {
        setMotorVolts(mIntakePivotController.calculate(getPos().getRotations(), pRot.getRotations()) + feedforward);
    }

    public void setPDConstants(double pKP, double pKD) {
        mIntakePivotController.setPID(pKP, 0.0, pKD);
    }

    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        return;
    }

    public void enforceSoftLimits() {
        double currentRotation = getPos().getRotations();
        if((currentRotation > mLimits.forwardLimit().getRotations() && mAppliedVolts > 0) || 
           (currentRotation < mLimits.backwardLimit().getRotations() && mAppliedVolts < 0)) stopMotor();
    }

    public void stopMotor() {
        setMotorVolts(0.0);
    }
}
