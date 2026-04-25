package frc.robot.systems.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.game.GameGoalPoseChooser;
import frc.robot.commands.FollowPathCommand;

public class SingleSwipe extends Auton {
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeSwitchToAlignTime;

    private final double kShotTimeSeconds = 6.5;

    public SingleSwipe(
        AutonCommands pAutos, 
        String pAutoName, 
        String pFirstSwipePathName, 
        double pFirstSwipeAlignTime) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeSwitchToAlignTime = pFirstSwipeAlignTime;
    }

    @Override
    protected AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);
        Trigger autoActivted = auto.getIsRunningTrigger();

        mAutos.runIntake(
            auto.loggedCondition(
                auto.getName() + "/IntakeWhileInCenter", 
                () -> GameGoalPoseChooser.inCenter(mDriveSS.getPoseEstimate()), 
                false), 
            auto.getName(), 
            auto);

        FollowPathCommand firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true, auto);

        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        Trigger firstPathEnded = mAutos.traversePathWithIntakeOutOnly(
            0.1,
            firstSwipePath, 
            autoActivted, 
            mFirstSwipePathName, 
            auto);

        Trigger autoAlignShotReadySwipe1 = mAutos.transitionFromPathTraversingToAutoAlignHubShoot(
            mDriveSS.getDriveManager().runAutoAlignThroughTrench(getSwipeEndPose(lastPoseOfFirstSwipe)), 
            firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime), 
            mFirstSwipePathName, 
            auto);

        Trigger fuelToHubHasEndedSwipe1 = mAutos.shootFuelToHub(
            kShotTimeSeconds, 
            autoAlignShotReadySwipe1, 
            mFirstSwipePathName, 
            auto);

        // Trigger intakingRange = mAutos.inIntakeRange(auto);
        // Trigger shootingRange = auto.loggedCondition(
        //     auto.getName()+"/WantToShoot", 
        //     () -> mWantToShoot, 
        //     true);

        // Trigger inShootingTolerance = auto.loggedCondition(
        //     auto.getName()+"/ShootingTolerance", 
        //     () -> 
        //         mFlywheelsSS.atLatestClosedLoopGoal() && 
        //         mHoodSS.atGoal() &&
        //         mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
        //     true);

        // Trigger inShootingToleranceDebounced = inShootingTolerance
        //     .debounce(0.05, DebounceType.kRising)
        //     .debounce(5.0, DebounceType.kFalling);

        // FollowPathCommand firstSwipePath = 
        //     followChoreoPath(mFirstSwipeSwitchToAlignTime, true, auto);

        // Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipeSwitchToAlignTime).get().getPathPoses().get(
        //     mAutos.getTraj(mFirstSwipeSwitchToAlignTime).get().getPathPoses().size() - 1);

        // SequentialEndingCommandGroup firstSwipeIntakeShot = 
        //     mAutos.timedIntakeShot(kShotTimeSeconds, kShotEndTimeSeconds);

        // SequentialEndingCommandGroup firstSwipeInjectorShot = 
        //     mAutos.timedInjectorShot(kShotTimeSeconds, kShotEndTimeSeconds);

        // Trigger hasFirstShotEnded = auto.loggedCondition(
        //     mFirstSwipeSwitchToAlignTime+"/FirstShotEnded", 
        //     () -> (firstSwipeIntakeShot.hasEnded() && firstSwipeInjectorShot.hasEnded()),
        //     true);

        // intakingRange
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        // shootingRange
        //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
        //     .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION))
        //     .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
        //     .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
        //     .onFalse(mHoodSS.setStateCmd(HoodStates.MIN))
        //     .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));

        // autoActivted
        //     .onTrue(Commands.waitSeconds(0.5).andThen(firstSwipePath))
        //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
        //     .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
        //     .onTrue(Commands.runOnce(() -> mWantToShoot = false));

        // firstSwipePath.atTime(mFirstSwipeAlignTime)
        //     .onTrue(Commands.runOnce(() -> mWantToShoot = true))
        //     .onTrue(new SequentialEndingCommandGroup(
        //             transitionPose(mTrenchApproachPose),
        //             transitionPose(mTrenchExitPose),
        //             mDriveSS.getDriveManager().setToGenericAutoAlignWithGeneratorReset(
        //                 () -> getSwipeEndPose(lastPoseOfFirstSwipe),
        //                 ConstraintType.LINEAR)));

        // firstSwipePath.hasEnded().and(() -> mWantToShoot).and(hasFirstShotEnded.negate()).and(inShootingToleranceDebounced)
        //     .onTrue(firstSwipeIntakeShot)
        //     .onTrue(firstSwipeInjectorShot)
        //     .onTrue(mIntakeSS.trashCompact());

        // firstSwipePath.hasEnded().and(() -> firstSwipeInjectorShot.hasEnded() && firstSwipeIntakeShot.hasEnded())
        //     .onTrue(Commands.runOnce(() -> mWantToShoot = false))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //     .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
        //     .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
        //     .onTrue(mAutos.endAuto(auto));

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

    // public Command transitionPose(Supplier<Pose2d> poseSup) {
    //     return mDriveSS.getDriveManager().setToGenericAutoAlignWithGeneratorReset(
    //         poseSup,
    //         () -> new ChassisSpeeds(AllianceFlipUtil.shouldFlip() ? -0.5 : 0.5, 0.0, 0.0),
    //         ConstraintType.LINEAR)
    //             .onlyIf(
    //                 () -> (
    //                     !AllianceFlipUtil.shouldFlip()
    //                         &&
    //                     poseSup.get().getX() < mDriveSS.getPoseEstimate().getX()
    //                 ) || (
    //                     AllianceFlipUtil.shouldFlip()
    //                         &&
    //                     poseSup.get().getX() > mDriveSS.getPoseEstimate().getX()
    //                 )
    //             )
    //             .onlyWhile(() -> (mDriveSS.getDriveManager().getAutoAlignController().inTolerance(new Transform2d(0.1, 0.1, Rotation2d.fromDegrees(5.0)), mDriveSS.getPoseEstimate())));
    // }
}