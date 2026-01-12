// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.game.GameGoalPoseChooser.CHOOSER_STRATEGY;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import java.util.function.Supplier;

public class GameDriveManager {
    public static enum GameDriveStates {
        PROCESSOR_HEADING_ALIGN,
        INTAKE_HEADING_ALIGN,
        REEF_HEADING_ALIGN,
        DRIVE_TO_CORAL,
        DRIVE_TO_ALGAE,
        DRIVE_TO_INTAKE,
        DRIVE_TO_BARGE,
        DRIFT_TEST,
        LINEAR_TEST,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION
    }

    public Drive mDrive;

    public GameDriveManager(Drive pDrive) {
        this.mDrive = pDrive;
    }

    public Command getSetGameDriveStateCmd(GameDriveStates pGameDriveState) {
        switch (pGameDriveState) {
            case PROCESSOR_HEADING_ALIGN:
                return mDrive.setToGenericHeadingAlign(() -> AllianceFlipUtil.apply(Rotation2d.fromDegrees(90.0)));
            case INTAKE_HEADING_ALIGN:
                return mDrive.setToGenericHeadingAlign(
                        () -> AllianceFlipUtil.apply(GameGoalPoseChooser.getIntakePose(mDrive.getPoseEstimate())
                                .getRotation()));
            case REEF_HEADING_ALIGN:
                return mDrive.setToGenericHeadingAlign(
                        () -> AllianceFlipUtil.apply(GameGoalPoseChooser.turnFromReefOrigin(mDrive.getPoseEstimate())));
            case DRIVE_TO_CORAL:
                return mDrive.setToGenericAutoAlign(
                        helperGetPose(CHOOSER_STRATEGY.kReefHexagonal), ConstraintType.LINEAR);
            case DRIVE_TO_INTAKE:
                return mDrive.setToGenericAutoAlign(helperGetPose(CHOOSER_STRATEGY.kIntake), ConstraintType.LINEAR);
            case DRIVE_TO_ALGAE:
                return mDrive.setToGenericAutoAlign(
                        helperGetPose(CHOOSER_STRATEGY.kReefHexagonal), ConstraintType.AXIS);
            case DRIVE_TO_BARGE:
                return mDrive.setToGenericAutoAlign(helperGetPose(CHOOSER_STRATEGY.kNet), ConstraintType.AXIS);
            default:
                return new InstantCommand(() -> DriverStation.reportError(
                        "<<< UNACCOUNTED DRIVE STATE \"" + pGameDriveState.toString() + "\" >>>", true));
        }
    }

    public Supplier<Pose2d> helperGetPose(CHOOSER_STRATEGY pChooserStrategy) {
        return () -> GameGoalPoseChooser.getGoalPose(pChooserStrategy, mDrive.getPoseEstimate());
    }
}
