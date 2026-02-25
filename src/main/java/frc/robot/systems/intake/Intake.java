// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.intake.pivot.IntakePivotSS;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;

public class Intake {
  private final IntakePivotSS mIntakePivotSS;
  private final IntakeRollerSS mIntakeRollerSS;

  public Intake(IntakePivotSS pIntakePivotSS, IntakeRollerSS pIntakeRollerSS) {
    this.mIntakePivotSS = pIntakePivotSS;
    this.mIntakeRollerSS = pIntakeRollerSS;
  }

  // ROLLER COMMANDS //
  public Command setRollerStateCmd(IntakeRollerState rollerState) {
    return mIntakeRollerSS.setIntakeRollerState(rollerState);
  }

  public Command setRollerVoltsManualCmd(double pVolts) {
    return mIntakeRollerSS.setIntakeVoltsManual(pVolts);
  }
  
  public Command stopRollerCmd() {
    return mIntakeRollerSS.stopIntakeVoltsManual();
  }

  // PIVOT COMMANDS //
  public Command setPivotState(IntakePivotState pIntakePivotState) {
    return mIntakePivotSS.setIntakePivotState(pIntakePivotState);
  }
  
  public Command setPivotAmps(double pAmps){
    return mIntakePivotSS.setIntakePivotAmps(pAmps);
  }

  public Command setPivotAmps() {
    return mIntakePivotSS.setIntakePivotAmps();
  }
  
  public Command setPivotRotManualCmd(Rotation2d pRot){
    return mIntakePivotSS.setIntakePivotManual(pRot);
  }
  
  public Command setPivotRotManualCmd(){
    return mIntakePivotSS.setIntakePivotManual();
  }

  public Command setPivotVoltsCmd(double pVolts){
    return mIntakePivotSS.setIntakePivotVolts(pVolts);
  }

  public Command stopPivotCmd() {
    return mIntakePivotSS.stopIntakePivot();
  }
}
