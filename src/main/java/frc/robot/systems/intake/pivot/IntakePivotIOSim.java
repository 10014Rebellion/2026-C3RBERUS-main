package frc.robot.systems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CANdiEncoder;
import frc.robot.systems.intake.IntakeConstants.PivotConstants;

public class IntakePivotIOSim implements IntakePivotIO {
    private final SingleJointedArmSim mArmSim;
    private final ProfiledPIDController mIntakePivotController;
    private double mAppliedVolts = 0.0;

    public IntakePivotIOSim(BasicMotorHardware pConfig, CANdiEncoder pEncoderConfig) {
        mArmSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            pConfig.rotorToMechanismRatio(), 
            SingleJointedArmSim.estimateMOI(PivotConstants.kArmLengthMeters, PivotConstants.kArmMass), 
            PivotConstants.kArmLengthMeters, 
            PivotConstants.kPivotLimits.backwardLimit().getRadians(), 
            PivotConstants.kPivotLimits.backwardLimit().getRadians(), 
            true, 
            0.0, 
            0.0001, 0.0001);

        mIntakePivotController = 
            new ProfiledPIDController(
                PivotConstants.kPivotController.pdController().kP(), 
                0.0, 
                PivotConstants.kPivotController.pdController().kD(),
                new TrapezoidProfile.Constraints(
                    PivotConstants.kPivotController.motionMagicConstants().maxVelocity(), 
                    PivotConstants.kPivotController.motionMagicConstants().maxAcceleration()));
    }

    public void updateInputs(IntakePivotInputs pInputs) {
        mArmSim.update(0.02);
        pInputs.iIsIntakePivotConnected = true;
        pInputs.iIntakePivotRotation = getPos();
        pInputs.iIntakePivotVelocityRPS = Rotation2d.fromRotations(mArmSim.getVelocityRadPerSec() / (2 * Math.PI));
        pInputs.iIntakePivotAccelerationRPSS = Rotation2d.kZero;
        pInputs.iIntakePivotMotorVolts = mAppliedVolts;
        pInputs.iIntakePivotSupplyCurrentAmps = 0.0;
        pInputs.iIntakePivotStatorCurrentAmps = mArmSim.getCurrentDrawAmps();
        pInputs.iIntakePivotTempCelsius = 0.0;
        pInputs.iIntakeClosedLoopReference = Rotation2d.fromRotations(mIntakePivotController.getSetpoint().position);
        pInputs.iIntakeClosedLoopReferenceSlope = Rotation2d.fromRotations(mIntakePivotController.getSetpoint().velocity);

        pInputs.iIsEncoderConnected = false;
        pInputs.iEncoderPosition = Rotation2d.kZero;
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mAppliedVolts = MathUtil.clamp(pVolts, -12.0, 12.0);
        mArmSim.setInputVoltage(pVolts);
    }

    @Override
    public void resetPPID() {
        mIntakePivotController.reset(getPos().getRotations());
    }

    @Override
    public void setMotorPosition(Rotation2d pRot, double feedforward) {
        setMotorVolts(mIntakePivotController.calculate(getPos().getRotations(), pRot.getRotations()) + feedforward);
    }

    @Override
    public void setPDConstants(double pKP, double pKD) {
        mIntakePivotController.setPID(pKP, 0.0, pKD);
    }

    @Override
    public void setMotionMagicConstants(double pCruiseVelRPS, double pMaxAccelRPSS, double pMaxJerkRPSSS) {
        mIntakePivotController.setConstraints(new TrapezoidProfile.Constraints(pCruiseVelRPS, pMaxAccelRPSS));
    }

    @Override
    public void stopMotor() {
        setMotorVolts(0.0);
    }

    public Rotation2d getPos(){
        return Rotation2d.fromRadians(mArmSim.getAngleRads());
    }
}
