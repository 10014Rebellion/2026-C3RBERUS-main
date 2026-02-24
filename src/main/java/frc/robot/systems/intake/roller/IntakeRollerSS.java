// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSS extends SubsystemBase {
  private final IntakeRollerIO mIntakeRollerIO;
  private final IntakeRollerInputsAutoLogged mIntakeRollerInputs = new IntakeRollerInputsAutoLogged();

  public IntakeRollerSS(IntakeRollerIO pIntakeRollerIO) {
    this.mIntakeRollerIO = pIntakeRollerIO;
  }

  public void setVolts(double pVolts) {
    mIntakeRollerIO.setMotorVolts(pVolts);
  }

  public void stopMotor() {
    mIntakeRollerIO.stopMotor();
  }
  
  @Override
  public void periodic() {
    mIntakeRollerIO.updateInputs(mIntakeRollerInputs);
    Logger.processInputs("Intake/Roller", mIntakeRollerInputs);
  }
}
