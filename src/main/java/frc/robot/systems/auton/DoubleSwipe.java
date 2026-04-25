package frc.robot.systems.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.game.GameGoalPoseChooser;
import frc.robot.commands.FollowPathCommand;

public class DoubleSwipe extends Auton {
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeSwitchToAlignTime;
    private final String mSecondSwipePathName;
    private final double mSecondSwipeSwitchToAlignTime;

    private final double kShotTime1Seconds = 3.0;
    private final double kShotTime2Seconds = 6.5;

    public DoubleSwipe(
        AutonCommands pAutos, 
        String pAutoName, 
        String pFirstSwipePathName,
        double pFirstSwipeSwitchToAlignTime,
        String pSecondSwipePathName, 
        double pSecondSwipeSwitchToAlignTime) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeSwitchToAlignTime = pFirstSwipeSwitchToAlignTime;
        mSecondSwipePathName = pSecondSwipePathName;
        mSecondSwipeSwitchToAlignTime = pSecondSwipeSwitchToAlignTime;
    }

    @Override
    protected AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);
        Trigger autoActivated = auto.getIsRunningTrigger();

        mAutos.runIntake(
            auto.loggedCondition(
                auto.getName()+"/IntakeWhileInCenter", 
                () -> GameGoalPoseChooser.inCenter(mDriveSS.getPoseEstimate()),
                true), 
            auto.getName(), 
            auto);

        FollowPathCommand firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true, auto);
        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        Trigger firstPathEnded = mAutos.traversePathWithIntakeOutOnly(
            0.1, 
            firstSwipePath, 
            autoActivated, 
            mFirstSwipePathName, 
            auto);

        /* Takes over mid path */
        Trigger autoAlignShotReadySwipe1 = mAutos.transitionFromPathTraversingToAutoAlignHubShoot(
            mDriveSS.getDriveManager().runAutoAlignThroughTrench(getSwipeEndPose(lastPoseOfFirstSwipe)), 
            firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime), 
            mFirstSwipePathName, 
            auto);

        Trigger fuelToHubHasEndedSwipe1 = mAutos.shootFuelToHub(
            kShotTime1Seconds, 
            autoAlignShotReadySwipe1, 
            mFirstSwipePathName, 
            auto);

        FollowPathCommand secondSwipePath = 
            followChoreoPath(mSecondSwipePathName, false, auto);
        Pose2d lastPoseOfSecondSwipe = mAutos.getTraj(mSecondSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mSecondSwipePathName).get().getPathPoses().size() - 1);

        Trigger secondPathHasEnded = mAutos.traversePathWithIntakeOutOnly(
            secondSwipePath, 
            fuelToHubHasEndedSwipe1, 
            mSecondSwipePathName, 
            auto);

        /* Takes over mid path */
        Trigger autoAlignShotReadySwipe2 = mAutos.transitionFromPathTraversingToAutoAlignHubShoot(
            mDriveSS.getDriveManager().runAutoAlignThroughTrench(getSwipeEndPose(lastPoseOfSecondSwipe)), 
            secondSwipePath.atTime(mSecondSwipeSwitchToAlignTime), 
            mSecondSwipePathName, 
            auto);

        Trigger fuelToHubHasEndedSwipe2 = mAutos.shootFuelToHub(
            kShotTime2Seconds, 
            autoAlignShotReadySwipe2, 
            mSecondSwipePathName, 
            auto);

        fuelToHubHasEndedSwipe2.onTrue(mAutos.endAuto(auto));

        return auto;
    }

    private Pose2d getSwipeEndPose(Pose2d pose) {
        return AllianceFlipUtil.apply(
            new Pose2d(
                pose.getX(), pose.getY(),
                GameGoalPoseChooser.turnFromHub(AllianceFlipUtil.apply(pose))
                    .plus((AllianceFlipUtil.shouldFlip()) 
                        ? Rotation2d.k180deg 
                        : Rotation2d.kZero)
        ));
    }
}