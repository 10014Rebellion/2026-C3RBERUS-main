// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.intake.IntakeConstants;

public class IntakeRollerSS extends SubsystemBase {
    private final IntakeRollerIO mIntakeRollerIO;
    private final IntakeRollerInputsAutoLogged mIntakeRollerInputs = new IntakeRollerInputsAutoLogged();

    public IntakeRollerSS(IntakeRollerIO pIntakeRollerIO) {
        this.mIntakeRollerIO = pIntakeRollerIO;
    }
  
    @Override
    public void periodic() {
        mIntakeRollerIO.updateInputs(mIntakeRollerInputs);
        Logger.processInputs("Intake/Roller", mIntakeRollerInputs);
    }
    // Commands that encapsulate the previous per-state output logic. Each command
    // sets the motor voltage for the corresponding behavior and holds until
    // interrupted (matching the prior setStateCmd behavior).

    public Command idleCmd() {
        return Commands.startEnd(
            () -> mIntakeRollerIO.setMotorVolts(IntakeConstants.RollerConstants.tIdleTuningVoltage.get()),
            () -> {},
            this
        );
    }

    public Command intakeCmd() {
        return Commands.startEnd(
            () -> mIntakeRollerIO.setMotorVolts(IntakeConstants.RollerConstants.tIntakeTuningVoltage.get()),
            () -> {},
            this
        );
    }

    public Command outtakeCmd() {
        return Commands.startEnd(
            () -> mIntakeRollerIO.setMotorVolts(IntakeConstants.RollerConstants.tOuttakeTuningVoltage.get()),
            () -> {},
            this
        );
    }

    public Command tuningCmd() {
        return Commands.startEnd(
            () -> mIntakeRollerIO.setMotorVolts(IntakeConstants.RollerConstants.tRollerTuningVoltage.get()),
            () -> {},
            this
        );
    }
}