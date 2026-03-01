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
    return mIntakeRollerSS.setIntakeRollerStateCmd(rollerState);
  }

  public Command trashCompact(){
    return mIntakePivotSS.trashCompact();
  }

  public Command setRollerVoltsManualCmd(double pVolts) {
    return mIntakeRollerSS.setIntakeVoltsManualCmd(pVolts);
  }
  
  public Command stopRollerCmd() {
    return mIntakeRollerSS.stopIntakeVoltsManualCmd();
  }

  // PIVOT COMMANDS //
  public Command setPivotStateCmd(IntakePivotState pIntakePivotState) {
    return mIntakePivotSS.setIntakePivotStateCmd(pIntakePivotState);
  }
  
  public Command setPivotAmps(double pAmps){
    return mIntakePivotSS.setIntakePivotAmpsCmd(pAmps);
  }

  public Command setPivotAmps() {
    return mIntakePivotSS.setIntakePivotAmpsCmd();
  }
  
  public Command setPivotRotManualCmd(Rotation2d pRot){
    return mIntakePivotSS.setIntakePivotManualCmd(pRot);
  }
  
  public Command setPivotRotManualCmd(){
    return mIntakePivotSS.setIntakePivotManualCmd();
  }

  public Command setPivotVoltsCmd(double pVolts){
    return mIntakePivotSS.setIntakePivotVoltsCmd(pVolts);
  }

  public Command stopPivotCmd() {
    return mIntakePivotSS.stopIntakePivotCmd();
  }
}
