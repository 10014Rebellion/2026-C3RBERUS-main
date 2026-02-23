package frc.robot.systems.intake.pivot;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CANdiEncoder;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.IntakeConstants.PivotConstants;

public class IntakePivotIOSim implements IntakePivotIO {

    private final SingleJointedArmSim mIntakePivotMotor;
    private final PIDController mIntakePivotController;
    private double mAppliedVolts = 0.0;
    private final RotationSoftLimits mLimits;

    public IntakePivotIOSim(BasicMotorHardware pConfig, CANdiEncoder pEncoderConfig, RotationSoftLimits pLimits) {
        mIntakePivotMotor = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            pConfig.rotorToMechanismRatio(), 
            0.05, 
            Units.inchesToMeters(18), 
            IntakeConstants.PivotConstants.kPivotLimits.forwardLimit().getRadians(), 
            IntakeConstants.PivotConstants.kPivotLimits.backwardLimit().getRadians(), 
            true,
            IntakeConstants.PivotConstants.kPivotLimits.backwardLimit().getRadians());

        mIntakePivotController = new PIDController(PivotConstants.kPivotController.pdController().kP(), 0.0, PivotConstants.kPivotController.pdController().kD());
        mLimits = pLimits;
    }

    public void updateInputs(IntakePivotInputs pInputs) {
        mIntakePivotMotor.update(0.02);
        pInputs.iIsIntakePivotConnected = true;
        pInputs.iIntakePivotRotation = getPos();
        pInputs.iIntakePivotVelocityRPS = Rotation2d.fromRotations(mIntakePivotMotor.getVelocityRadPerSec() / (2 * Math.PI));
        pInputs.iIntakePivotAccelerationRPSS = Rotation2d.kZero;
        pInputs.iIntakePivotMotorVolts = mAppliedVolts;
        pInputs.iIntakePivotSupplyCurrentAmps = 0.0;
        pInputs.iIntakePivotStatorCurrentAmps = mIntakePivotMotor.getCurrentDrawAmps();
        pInputs.iIntakePivotTempCelsius = 0.0;
    }

    public Rotation2d getPos(){
        return Rotation2d.fromRadians(mIntakePivotMotor.getAngleRads());
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
