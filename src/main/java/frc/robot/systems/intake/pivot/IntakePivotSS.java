// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake.pivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.IntakeConstants.PivotConstants;

import static frc.robot.systems.intake.IntakeConstants.PivotConstants.kPivotController;

import java.util.function.Supplier;

public class IntakePivotSS extends SubsystemBase {
  public static final LoggedTunableNumber tPivotCustomSetpointRot = 
    new LoggedTunableNumber("Intake/Pivot/Control/CustomSetpointRot", 0);

  public static enum IntakePivotState {
    INTAKE(() -> PivotConstants.kPivotLimits.backwardLimit()),
    STOWED(() -> PivotConstants.kPivotLimits.forwardLimit()),
    TUNING(() -> Rotation2d.fromRotations(tPivotCustomSetpointRot.get()));

    private Supplier<Rotation2d> mRotSupplier;

    private IntakePivotState(Supplier<Rotation2d> pRotSupplier) {
      mRotSupplier = pRotSupplier;
    }

    public Rotation2d getDesiredRotation() {
      return mRotSupplier.get();
    } 
  }


  private final IntakePivotIO mIntakePivotIO;
  private final IntakePivotInputsAutoLogged mIntakePivotInputs = new IntakePivotInputsAutoLogged();

  private final ArmFeedforward mPivotFF;

  private final LoggedTunableNumber tPivotKP = new LoggedTunableNumber("Intake/Pivot/Control/kP", kPivotController.pdController().kP());
  private final LoggedTunableNumber tPivotKD = new LoggedTunableNumber("Intake/Pivot/Control/kD", kPivotController.pdController().kD());

  private final LoggedTunableNumber tPivotKS = new LoggedTunableNumber("Intake/Pivot/Control/kS", kPivotController.feedforward().getKs());
  private final LoggedTunableNumber tPivotKG = new LoggedTunableNumber("Intake/Pivot/Control/kG", kPivotController.feedforward().getKg());
  private final LoggedTunableNumber tPivotKV = new LoggedTunableNumber("Intake/Pivot/Control/kV", kPivotController.feedforward().getKv());
  private final LoggedTunableNumber tPivotKA = new LoggedTunableNumber("Intake/Pivot/Control/kA", kPivotController.feedforward().getKa());

  private final LoggedTunableNumber tPivotCruiseVel = new LoggedTunableNumber("Intake/Pivot/Control/CruiseVel", kPivotController.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tPivotMaxAccel = new LoggedTunableNumber("Intake/Pivot/Control/MaxAcceleration", kPivotController.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tPivotMaxJerk = new LoggedTunableNumber("Intake/Pivot/Control/MaxJerk", kPivotController.motionMagicConstants().maxJerk());

  public static final LoggedTunableNumber tCustomAmps = new LoggedTunableNumber("Intake/Pivot/Custom/Amps", 0.0);
  public static final LoggedTunableNumber tPivotPositionTolerance = new LoggedTunableNumber("Intake/Pivot/Control/Tolerance", IntakeConstants.PivotConstants.kPivotMotorToleranceRotations);

  private IntakePivotState mIntakePivotState = IntakePivotState.STOWED;
  private Rotation2d mCurrentRotationalGoal = Rotation2d.kZero;
  private Double mAppliedVolts = null;
  private Double mAppliedAmps = null;

  
  public IntakePivotSS(IntakePivotIO pIntakePivotIO) {
    this.mIntakePivotIO = pIntakePivotIO;
    this.mPivotFF = kPivotController.feedforward();
  }

  @Override
  public void periodic() {
    mIntakePivotIO.updateInputs(mIntakePivotInputs);

    refreshTuneables();
    enforceSoftLimits();

    Logger.processInputs("Intake/Pivot", mIntakePivotInputs);

    if(mIntakePivotState != null) {
      mCurrentRotationalGoal = mIntakePivotState.getDesiredRotation();
      setPivotRot(mCurrentRotationalGoal);
    }
  }

  public void enforceSoftLimits() {
    boolean positive;
    if(mAppliedVolts != null) {
      positive = mAppliedVolts > 0;
    } else if(mAppliedAmps != null) {
      positive = mAppliedAmps > 0;
    } else {
      positive = getErrorRotations().getRotations() > 0;
    }

    if((getIntakePivotRotations().getRotations() > IntakeConstants.PivotConstants.kPivotLimits.forwardLimit().getRotations() && positive) || 
       (getIntakePivotRotations().getRotations() < IntakeConstants.PivotConstants.kPivotLimits.backwardLimit().getRotations() && !positive)) {
      mIntakePivotIO.stopMotor();
    }
  }

  public Command setIntakePivotStateCmd(IntakePivotState pIntakePivotState){
    return Commands.run(() -> {
      setPivotState(pIntakePivotState);
    }, this);
  }

  public Command setIntakePivotVoltsCmd(double pVolts){
    return Commands.run(() -> {
      setPivotVolts(pVolts);
    }, this);
  }

  public Command setIntakePivotAmpsCmd(double pAmps){
    return Commands.run(() -> {
      setPivotAmps(pAmps);
    }, this);
  }

  public Command setIntakePivotAmpsCmd(){
    return Commands.run(() -> {
      setCustomPivotAmps();
    }, this);
  }

  public Command setIntakePivotManualCmd(Rotation2d pRot){
    return Commands.run(() -> {
      setPivotRotManual(pRot);
    }, this);
  }

  public Command setIntakePivotManualCmd(){
    return Commands.run(() -> {
      setPivotRotManual();
    }, this);
  }

  public Command stopIntakePivotCmd(){
    return Commands.run(() -> {
      stopPivotMotor();
    }, this);
  }
  
  public void setPivotState(IntakePivotState pIntakePivotState) {
    mAppliedAmps = null;
    mAppliedVolts = null;
    mIntakePivotState = pIntakePivotState;
  }

  public void setPivotVolts(double pVolts) {
    mCurrentRotationalGoal = null;
    mIntakePivotState = null;
    mAppliedAmps = null;
    mAppliedVolts = pVolts;
    mIntakePivotIO.setMotorVolts(pVolts);
  }

  public void setPivotAmps(double pAmps) {
    mCurrentRotationalGoal = null;
    mIntakePivotState = null;
    mAppliedVolts = null;
    mAppliedAmps = pAmps;
    mIntakePivotIO.setMotorVolts(pAmps);
  }
  
  public void stopPivotMotor() {
    mCurrentRotationalGoal = null;
    mIntakePivotState = null;
    mAppliedAmps = null;
    mAppliedVolts = 0.0;
    mIntakePivotIO.stopMotor();
  }

  public void setPivotRotManual(Rotation2d pRot) {
    mAppliedAmps = null;
    mAppliedVolts = null;
    mIntakePivotState = null;
    setPivotRot(pRot);
  }

  public void setPivotRot(Rotation2d pRot) {
    pRot = Rotation2d.fromRotations(MathUtil.clamp(pRot.getRotations(), PivotConstants.kPivotLimits.backwardLimit().getRotations(), PivotConstants.kPivotLimits.forwardLimit().getRotations()));
    mCurrentRotationalGoal = pRot;
    double ffOutput = mPivotFF.calculate(
      mIntakePivotInputs.iIntakePivotRotation.getRadians(), 
      mIntakePivotInputs.iIntakeClosedLoopReferenceSlope.getRadians()
    );

    double cos = mIntakePivotInputs.iIntakeClosedLoopReference.getCos();

    Logger.recordOutput("Intake/Pivot/ffOutput", ffOutput);
    Logger.recordOutput("Intake/Pivot/cos", cos);

    mIntakePivotIO.setMotorRot(pRot, ffOutput);
  }

  public void setPivotRotManual() {
    setPivotRotManual(Rotation2d.fromRotations(tPivotCustomSetpointRot.getAsDouble()));
  }

  public void setCustomPivotAmps() {
    setPivotAmps(tCustomAmps.get());;
  }


  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mIntakePivotIO.setPDConstants(tPivotKP.get(), tPivotKD.get()), 
      tPivotKP, tPivotKD
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setFF(tPivotKS.get(), tPivotKG.get(), tPivotKV.get(), tPivotKA.get()), 
      tPivotKS, tPivotKG, tPivotKV, tPivotKA
    );
  
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mIntakePivotIO.setMotionMagicConstants(tPivotCruiseVel.get(), tPivotMaxAccel.get(), tPivotMaxJerk.get()), 
      tPivotCruiseVel, tPivotMaxAccel, tPivotMaxJerk
    );
  }

  private void setFF(double kS, double kG, double kV, double kA) {
    mPivotFF.setKs(kS);
    mPivotFF.setKg(kG);
    mPivotFF.setKv(kV);
    mPivotFF.setKa(kA);
  }

  private Rotation2d getIntakePivotRotations(){
    return mIntakePivotInputs.iIntakePivotRotation;
  }

  @AutoLogOutput(key = "Intake/Feedback/ErrorRotations")
  public Rotation2d getErrorRotations() {
    return Rotation2d.fromRotations(mCurrentRotationalGoal.getRotations() - getIntakePivotRotations().getRotations());
  }
  
  @AutoLogOutput(key = "Intake/Feedback/CurrentGoal")
  public double getCurrentGoal() {
    return mCurrentRotationalGoal.getRotations();
  }
  
  @AutoLogOutput(key = "Intake/Feedback/AtGoal")
  public boolean atGoal() {
    return Math.abs(getErrorRotations().getRotations()) < tPivotPositionTolerance.get();
  }
}
