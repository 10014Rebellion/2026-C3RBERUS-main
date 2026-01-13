// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import java.util.function.Supplier;

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
                    () -> GameGoalPoseChooser.getClosestBump(mDrive.getPoseEstimate()), ConstraintType.AXIS);
            case DRIVE_TO_BUMP:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestBump(mDrive.getPoseEstimate()), ConstraintType.AXIS);
            case DRIVE_TO_O:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getO(), ConstraintType.AXIS);
            case DRIVE_TO_D:
                return mDrive.setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getD(), ConstraintType.AXIS);
            default:
                return new InstantCommand(() -> DriverStation.reportError(
                        "<<< UNACCOUNTED DRIVE STATE \"" + pGameDriveState.toString() + "\" >>>", true));
        }
    }

    public Supplier<Pose2d> helperGetPose(CHOOSER_STRATEGY pChooserStrategy) {
        return () -> GameGoalPoseChooser.getGoalPose(pChooserStrategy, mDrive.getPoseEstimate());
    }
}
