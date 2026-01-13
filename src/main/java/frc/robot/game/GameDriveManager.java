// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.drive.Drive;

public class GameDriveManager {
    public static enum GameDriveStates {
        HUB_HEADING_ALIGN,
        DRIVE_TO_TRENCH,
        DRIVE_TO_BUMP,
        DRIVE_TO_O,
        DRIVE_TO_D
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
            case DRIVE_TO_TRENCH:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestTrench(mDrive.getPoseEstimate()));
            case DRIVE_TO_BUMP:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestBump(mDrive.getPoseEstimate()));
            case DRIVE_TO_O:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getO());
            case DRIVE_TO_D:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getD());
            default:
                return new InstantCommand(() -> DriverStation.reportError(
                        "<<< UNACCOUNTED DRIVE STATE \"" + pGameDriveState.toString() + "\" >>>", true));
        }
    }
}
