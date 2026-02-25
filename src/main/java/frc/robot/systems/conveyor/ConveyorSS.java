// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.conveyor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class ConveyorSS extends SubsystemBase {
  public static enum ConveyorState {
    IDLE(() -> 0.0),
    INTAKE(() -> 10.014),
    OUTTAKE(() -> -10.014),
    CONVEY_TO_INDEX(() -> 10.014),
    TUNING(new LoggedTunableNumber("Conveyor/TuneableVoltage", 0.0));

    private DoubleSupplier mVoltage;

    private ConveyorState(DoubleSupplier pVoltage) {
      mVoltage = pVoltage;
    }

    public double getDesiredVoltge() {
      return mVoltage.getAsDouble();
    } 
  }

  private final ConveyorIO mConveyorIO;
  private final ConveyorInputsAutoLogged mConveyorInputs = new ConveyorInputsAutoLogged();

  private ConveyorState mConveyorState = ConveyorState.IDLE;

  public ConveyorSS(ConveyorIO pConveyorIO) {
    this.mConveyorIO = pConveyorIO;
  }

  public Command setConveyorStateCmd(ConveyorState pConveyorState) {
    return Commands.run(() -> {
      mConveyorState = pConveyorState;
    }, this);
  }

  public Command setConveyorVoltsManualCmd(double pVolts) {
    return Commands.run(() -> {
      mConveyorState = null;
      mConveyorIO.setMotorVolts(pVolts);
    }, this);
  }

  public Command stopConveyorMotorManuallyCmd() {
    return Commands.run(() -> {
      mConveyorState = null;
      stopConveyorMotor();
    }, this);
  }

  private void stopConveyorMotor() {
    mConveyorIO.stopMotor();
  }

  @Override
  public void periodic() {
    mConveyorIO.updateInputs(mConveyorInputs);
    Logger.processInputs("Conveyor", mConveyorInputs);

    if(mConveyorState != null) {
      Logger.recordOutput("Conveyor/DesiredVoltage", mConveyorState.getDesiredVoltge());

      mConveyorIO.setMotorVolts(mConveyorState.getDesiredVoltge());
    }
  }
}
