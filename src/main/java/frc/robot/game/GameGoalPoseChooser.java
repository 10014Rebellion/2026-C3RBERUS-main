// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/* Chooses pose based of strategy and psoe */
public class GameGoalPoseChooser {
    public static enum CHOOSER_STRATEGY {
        kTest
    }

    public static Pose2d getClosestTrench(Pose2d robotPose) {
        return (robotPose.getY() > 8.0) ? getTL() : getTR(); 
    }

    public static Pose2d getTL() {
        return new Pose2d();
    }

    public static Pose2d getTR() {
        return new Pose2d();
    }

    public static Pose2d getClosestBump(Pose2d robotPose) {
        return (robotPose.getY() > 8.0) ? getBL() : getBR(); 
    }

    public static Pose2d getBL() {
        return new Pose2d();
    }

    public static Pose2d getBR() {
        return new Pose2d();
    }

    public static Pose2d getO() {
        return new Pose2d();
    }

    public static Pose2d getD() {
        return new Pose2d();
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromHub(Pose2d robotPose) {
        Pose2d reefCenter = AllianceFlipUtil.apply(FieldConstants.kField.hubPose().toPose2d());
        Rotation2d angleFromReefCenter = Rotation2d.fromRadians(
                Math.atan2(robotPose.getY() - reefCenter.getY(), robotPose.getX() - reefCenter.getX()));

        Rotation2d finalAngle = angleFromReefCenter.times(-1.0);
        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
            finalAngle = angleFromReefCenter.plus(Rotation2d.k180deg).times(-1.0);
        Logger.recordOutput("Drive/GoalPoseAngle", finalAngle);
        return finalAngle;
    }
}