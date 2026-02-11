// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.systems.intake.pivot.IntakePivotSS;
import frc.robot.systems.intake.roller.IntakeRollerSS;

public class Intake {
  private final IntakeRollerSS mIntakeRollerSS;
  private final IntakePivotSS mIntakePivotSS;

  public Intake(IntakeRollerSS pIntakeRollerSS, IntakePivotSS pIntakePivotSS) {
    this.mIntakeRollerSS = pIntakeRollerSS;
    this.mIntakePivotSS = pIntakePivotSS;
  }

  public void setRollerVolts(double pVolts) {
    mIntakeRollerSS.setVolts(pVolts);
  }

  public void stopRollerMotor() {
    mIntakeRollerSS.stopMotor();
  }

  public void setPivotRotCmd(Rotation2d pRotSP) {
    mIntakePivotSS.setPivotVolts(0);
  }
}
