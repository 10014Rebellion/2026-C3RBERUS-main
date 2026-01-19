// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.math.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/* Chooses pose based of strategy and psoe */
// TODO: UPDATE ALL POSES FOR GAME
public class GameGoalPoseChooser {
    public static enum CHOOSER_STRATEGY {
        kTest
    }

    /* Static positions */
    public static Pose2d getSafeScoringPosition() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    public static Pose2d getO() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    public static Pose2d getD() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromHub(Pose2d robotPose) {
        Pose2d hubCenter = AllianceFlipUtil.apply(FieldConstants.kHubPose);
        Rotation2d angleFromhubCenter = Rotation2d.fromRadians(
                Math.atan2(hubCenter.getY() - robotPose.getY(), hubCenter.getX() - robotPose.getX()));

        Rotation2d finalAngle = angleFromhubCenter;
        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
            finalAngle = angleFromhubCenter.plus(Rotation2d.k180deg).times(-1.0);
        Logger.recordOutput("Drive/GoalPoseAngle", finalAngle);
        return finalAngle;
    }

    /* Non-static Trench */
    public static Pose2d getClosestTrench(Pose2d robotPose) {
        return onLeftOrRightY(robotPose) ? getTL() : getTR(); 
    }

    public static Pose2d getTL() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    public static Pose2d getTR() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    /* Nonstatic bump */
    public static Pose2d getClosestBump(Pose2d robotPose) {
        return onLeftOrRightY(robotPose) ? getBL() : getBR(); 
    }

    public static Pose2d getBL() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    public static Pose2d getBR() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    /* Utils function */
    public static boolean onLeftOrRightY(Pose2d robotPose) {
        return 
            (robotPose.getY() > FieldConstants.kFieldYM / 2.0
                &&
            DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) 
                ||
            (!(robotPose.getY() > FieldConstants.kFieldYM / 2.0)
                &&
            !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue));
    }
}