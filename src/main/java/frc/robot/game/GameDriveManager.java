// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;

public class GameDriveManager {
    public static enum GameDriveStates {
        HUB_HEADING_ALIGN,
        AUTON_HUB_HEADING_ALIGN,
        LINE_TO_TRENCH,
        LINE_TO_BUMP,
        LINE_TO_O,
        LINE_TO_D,
        DRIVE_TO_SAFE_SCORE
    }

    public Drive mDrive;

    public GameDriveManager(Drive pDrive) {
        this.mDrive = pDrive;
    }

    public Command getSetGameDriveStateCmd(GameDriveStates pGameDriveState) {
        switch (pGameDriveState) {
            case HUB_HEADING_ALIGN:
                return mDrive.setToGenericHeadingAlign(
                    () -> GameGoalPoseChooser.turnFromHub(mDrive.getPoseEstimate()));
            case AUTON_HUB_HEADING_ALIGN:
                return mDrive.setToGenericHeadingAlignAuton(
                    () -> GameGoalPoseChooser.turnFromHub(mDrive.getPoseEstimate()));
            case LINE_TO_TRENCH:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestTrench(mDrive.getPoseEstimate()),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case LINE_TO_BUMP:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestBump(mDrive.getPoseEstimate()),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case LINE_TO_O:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getO(),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case LINE_TO_D:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getD(),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case DRIVE_TO_SAFE_SCORE:
                return mDrive.setToGenericAutoAlign(
                    () -> GameGoalPoseChooser.getSafeScoringPosition(), 
                    ConstraintType.LINEAR);
            default:
                return new InstantCommand(() -> DriverStation.reportError(
                        "<<< UNACCOUNTED DRIVE STATE \"" + pGameDriveState.toString() + "\" >>>", true));
        }
    }
}
