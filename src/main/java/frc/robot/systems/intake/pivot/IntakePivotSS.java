// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSS extends SubsystemBase {
  private final IntakePivotIO mIntakePivotIO;
  private final IntakePivotInputsAutoLogged mIntakePivotInputs = new IntakePivotInputsAutoLogged();
  
  public IntakePivotSS(IntakePivotIO pIntakePivotIO) {
    this.mIntakePivotIO = pIntakePivotIO;
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
  @Override
  public void periodic() {
    mIntakePivotIO.updateInputs(mIntakePivotInputs);

    mIntakePivotIO.enforceSoftLimits();

    Logger.processInputs("Intake", mIntakePivotInputs);
  }
}
