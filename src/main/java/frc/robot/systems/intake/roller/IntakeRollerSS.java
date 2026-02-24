// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake.roller;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class IntakeRollerSS extends SubsystemBase {
  public static enum IntakeRollerState {
    IDLE(() -> 0.0),
    INTAKE(() -> 10.014),
    OUTTAKE(() -> -10.014),
    TUNING(new LoggedTunableNumber("Intake/TuneableVoltage", 0.0));

    private DoubleSupplier mVoltage;

    private IntakeRollerState(DoubleSupplier pVoltage) {
      mVoltage = pVoltage;
    }

    public double getDesiredVoltge() {
      return mVoltage.getAsDouble();
    } 
  }

  private final IntakeRollerIO mIntakeRollerIO;
  private final IntakeRollerInputsAutoLogged mIntakeRollerInputs = new IntakeRollerInputsAutoLogged();

  private IntakeRollerState mIntakeRollerState = IntakeRollerState.IDLE;

  public IntakeRollerSS(IntakeRollerIO pIntakeRollerIO) {
    this.mIntakeRollerIO = pIntakeRollerIO;
  }
  
  @Override
  public void periodic() {
    mIntakeRollerIO.updateInputs(mIntakeRollerInputs);
    Logger.processInputs("Intake/Roller", mIntakeRollerInputs);

    if(mIntakeRollerState != null) {
      Logger.recordOutput("IntakeRoller/DesiredVoltage", mIntakeRollerState.getDesiredVoltge());

      mIntakeRollerIO.setMotorVolts(mIntakeRollerState.getDesiredVoltge());
    }
  }
  
  public Command setIntakeRollerState(IntakeRollerState pIntakeRollerState) {
    return Commands.run(() -> {
      mIntakeRollerState = pIntakeRollerState;
    }, this);
  }

  public Command setIntakeVoltsManualCmd(double pVolts) {
    return Commands.run(() -> {
      mIntakeRollerState = null;
      mIntakeRollerIO.setMotorVolts(pVolts);
    }, this);
  }

  public Command stopIntakeVoltsManualCmd() {
    return Commands.run(() -> {
      mIntakeRollerState = null;
      stopIntakeMotor();
    }, this);
  }

  private void stopIntakeMotor() {
    mIntakeRollerIO.stopMotor();
  }
  
}
