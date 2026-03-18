// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.systems.intake.pivot.IntakePivotSS;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotStates;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;

public class Intake {
    public static final double tIntakeCompactTime = 0.3;

    private final IntakePivotSS mIntakePivotSS;
    private final IntakeRollerSS mIntakeRollerSS;

    public Intake(IntakePivotSS pIntakePivotSS, IntakeRollerSS pIntakeRollerSS) {
        this.mIntakePivotSS = pIntakePivotSS;
        this.mIntakeRollerSS = pIntakeRollerSS;
    }

    // ROLLER COMMANDS //
    public Command setRollerStateCmd(IntakeRollerState rollerState) {
        return mIntakeRollerSS.setStateCmd(rollerState);
    }
  
    public Command stopRollerCmd() {
        return mIntakeRollerSS.setStateCmd(IntakeRollerState.IDLE);
    }

    // PIVOT COMMANDS //
    public Command setPivotStateCmd(IntakePivotStates pIntakePivotState) {
        return mIntakePivotSS.setStateCmd(pIntakePivotState);
    }

    public Command trashCompactPivotRepeat() {
        return new RepeatCommand(
            new SequentialCommandGroup(
                mIntakePivotSS.setStateCmd(IntakePivotStates.COMPACT_HIGH).withTimeout(tIntakeCompactTime),
                mIntakePivotSS.setStateCmd(IntakePivotStates.COMPACT_LOW).withTimeout(tIntakeCompactTime)
            )
        );
    }

    public Command trashCompactPivotContinuous(){
        return mIntakePivotSS.setStateCmd(IntakePivotStates.COMPACT_AMPS);
    }
}
