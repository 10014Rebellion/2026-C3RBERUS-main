// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

import static frc.robot.systems.intake.IntakeConstants.PivotConstants.kPivotController;

public class IntakePivotSS extends SubsystemBase {
  private final IntakePivotIO mIntakePivotIO;
  private final IntakePivotInputsAutoLogged mIntakePivotInputs = new IntakePivotInputsAutoLogged();

  private final ArmFeedforward mPivotFF;

  private final LoggedTunableNumber tPivotKP = new LoggedTunableNumber("Intake/Pivot/Control/kP", kPivotController.pdController().kP());
  private final LoggedTunableNumber tPivotKD = new LoggedTunableNumber("Intake/Pivot/Control/kD", kPivotController.pdController().kD());

  private final LoggedTunableNumber tPivotKS = new LoggedTunableNumber("Intake/Pivot//Control/kS", kPivotController.feedforward().getKs());
  private final LoggedTunableNumber tPivotKG = new LoggedTunableNumber("Intake/Pivot/Control/kG", kPivotController.feedforward().getKg());
  private final LoggedTunableNumber tPivotKV = new LoggedTunableNumber("Intake/Pivot/Control/kV", kPivotController.feedforward().getKv());
  private final LoggedTunableNumber tPivotKA = new LoggedTunableNumber("Intake/Pivot/Control/kA", kPivotController.feedforward().getKa());

  private final LoggedTunableNumber tPivotCruiseVel = new LoggedTunableNumber("Intake/Pivot/Control/CruiseVel", kPivotController.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tPivotMaxAccel = new LoggedTunableNumber("Intake/Pivot/Control/MaxAcceleration", kPivotController.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tPivotMaxJerk = new LoggedTunableNumber("Intake/Pivot/Control/MaxJerk", kPivotController.motionMagicConstants().maxJerk());
  
  public IntakePivotSS(IntakePivotIO pIntakePivotIO) {
    this.mIntakePivotIO = pIntakePivotIO;
    this.mPivotFF = kPivotController.feedforward();
  }

  public void setPivotRot(Rotation2d pRot) {
    mIntakePivotIO.setMotorRot(pRot, 0);
  }

  public void setPivotVolts(double pVolts) {
    mIntakePivotIO.setMotorVolts(pVolts);
  }

  public void stopPivotMotor() {
    mIntakePivotIO.stopMotor();
  }

  private void setFF(double kS, double kG, double kV, double kA) {
    mPivotFF.setKs(kS);
    mPivotFF.setKg(kG);
    mPivotFF.setKv(kV);
    mPivotFF.setKa(kA);
  }

  @Override
  public void periodic() {
    mIntakePivotIO.updateInputs(mIntakePivotInputs);

    refreshTuneables();
    mIntakePivotIO.enforceSoftLimits();

    Logger.processInputs("Intake", mIntakePivotInputs);
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
}
