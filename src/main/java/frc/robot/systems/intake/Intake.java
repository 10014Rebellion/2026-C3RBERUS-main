// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.intake.pivot.IntakePivotSS;
import frc.robot.systems.intake.roller.IntakeRollerSS;

public class Intake {
  private final IntakePivotSS mIntakePivotSS;
  private final IntakeRollerSS mIntakeRollerSS;

  public Intake(IntakePivotSS pIntakePivotSS, IntakeRollerSS pIntakeRollerSS) {
    this.mIntakePivotSS = pIntakePivotSS;
    this.mIntakeRollerSS = pIntakeRollerSS;
  }

  public Command setRollerVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mIntakeRollerSS.setVolts(pVolts));
  }

  public Command stopRollerMotorCmd() {
    return new InstantCommand(() -> mIntakeRollerSS.stopMotor());
  }

  public Command setPivotRotCmd(Rotation2d pRotSP) {
    return new InstantCommand(() -> mIntakePivotSS.setPivotRot(pRotSP));
  }
}
