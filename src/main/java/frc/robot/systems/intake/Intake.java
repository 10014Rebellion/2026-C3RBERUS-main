// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.systems.intake.pivot.IntakePivotSS;
// Intake pivot states migrated to string-based behaviors; use setPivotStateCmd("NAME") or convenience commands.
import frc.robot.systems.intake.roller.IntakeRollerSS;

public class Intake {
    public static final double tIntakeCompactTime = 0.3;

    private final IntakePivotSS mIntakePivotSS;
    private final IntakeRollerSS mIntakeRollerSS;

    public Intake(IntakePivotSS pIntakePivotSS, IntakeRollerSS pIntakeRollerSS) {
        this.mIntakePivotSS = pIntakePivotSS;
        this.mIntakeRollerSS = pIntakeRollerSS;
    }

    // ROLLER COMMANDS -- now explicit per-behavior methods
    public Command rollerIdleCmd() { return mIntakeRollerSS.idleCmd(); }
    public Command rollerIntakeCmd() { return mIntakeRollerSS.intakeCmd(); }
    public Command rollerOuttakeCmd() { return mIntakeRollerSS.outtakeCmd(); }
    public Command rollerTuningCmd() { return mIntakeRollerSS.tuningCmd(); }

    // PIVOT COMMANDS //
    public Command setPivotStateCmd(String pIntakePivotState) {
        return mIntakePivotSS.setStateCmd(pIntakePivotState);
    }

    public Command trashCompactPivotRepeat() {
        return new RepeatCommand(
            new SequentialCommandGroup(
                mIntakePivotSS.compactHighCmd().withTimeout(tIntakeCompactTime),
                mIntakePivotSS.compactLowCmd().withTimeout(tIntakeCompactTime)
            )
        );
    }

    public Command trashCompactPivotContinuous(){
        return mIntakePivotSS.compactAmpsCmd();
    }
}
