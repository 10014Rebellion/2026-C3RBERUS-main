// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return new InstantCommand(() -> mIntakeRollerSS.setVolts(pVolts), mIntakeRollerSS);
  }

  public Command stopPivotMotorCmd() {
    return new InstantCommand(() -> mIntakePivotSS.stopPivotMotor(), mIntakePivotSS);
  }

  public Command setPivotTuneableAmps() {
    return new InstantCommand(() -> mIntakePivotSS.setCustomPivotAmps(), mIntakePivotSS);
  }

  public Command stopRollerMotorCmd() {
    return new InstantCommand(() -> mIntakeRollerSS.stopMotor(), mIntakeRollerSS);
  }

  public Command setPivotVolts(double pVolts) {
    return new InstantCommand(() -> mIntakePivotSS.setPivotVolts(pVolts));
  }

  public Command setPivotRotCmd(Rotation2d pRotSP) {
    return Commands.run(() -> mIntakePivotSS.setPivotRot(pRotSP));
  }

  public Command setPivotRotCmd() {
    return Commands.run(() -> mIntakePivotSS.setPivotRot());
  }
}
