package frc.robot.systems.auton;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.game.GameGoalPoseChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FollowPathCommand;

import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotStates;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;

public class DoubleSwipeClimb extends Auton {
    private boolean mWantToShoot = false;

    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeSwitchToAlignTime;
    private final String mSecondSwipePathName;
    private final double mSecondSwipeSwitchToAlignTime;

    private final double kShotTime1Seconds = 3.0;
    private final double kShotTime2Seconds = 6.5;
    private final double kShotEndTimeSeconds = 0.02; 

    private final Pose2d mClimbPose;

    public DoubleSwipeClimb(
        AutonCommands pAutos, 
        String pAutoName, 
        String pFirstSwipePathName,
        double pFirstSwipeSwitchToAlignTime,
        String pSecondSwipePathName, 
        double pSecondSwipeSwitchToAlignTime,
        Pose2d pClimbPose) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeSwitchToAlignTime = pFirstSwipeSwitchToAlignTime;
        mSecondSwipePathName = pSecondSwipePathName;
        mSecondSwipeSwitchToAlignTime = pSecondSwipeSwitchToAlignTime;
        mClimbPose = pClimbPose;
    }

    @Override
    protected AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);
        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = mAutos.inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(
            auto.getName()+"/WantToShoot", 
            () -> mWantToShoot, 
            true);

        Trigger inShootingTolerance = auto.loggedCondition(
            auto.getName()+"/ShootingTolerance", 
            () -> 
                mFlywheelsSS.atLatestClosedLoopGoal() && 
                mHoodSS.atGoal() &&
                mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        Trigger inShootingToleranceDebounced = inShootingTolerance
            .debounce(0.1, DebounceType.kRising)
            .debounce(5.0, DebounceType.kFalling);

        FollowPathCommand firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true, auto);

        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        FollowPathCommand secondSwipePath = 
            followChoreoPath(mSecondSwipePathName, false, auto);

        Pose2d lastPoseOfSecondSwipe = mAutos.getTraj(mSecondSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mSecondSwipePathName).get().getPathPoses().size() - 1);

        intakingRange
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        SequentialEndingCommandGroup firstSwipeIntakeShot = 
            mAutos.timedIntakeShot(kShotTime1Seconds, kShotEndTimeSeconds);

        SequentialEndingCommandGroup firstSwipeIndexShot = 
            mAutos.timedIndexShot(kShotTime1Seconds, kShotEndTimeSeconds);

        Trigger hasFirstShotEnded = auto.loggedCondition(
            mFirstSwipePathName+"/FirstShotEnded", 
            () -> (firstSwipeIntakeShot.hasEnded() && firstSwipeIndexShot.hasEnded()),
            true);

        SequentialEndingCommandGroup secondSwipeIntakeShot = 
            mAutos.timedIntakeShot(kShotTime2Seconds, kShotEndTimeSeconds);

        SequentialEndingCommandGroup secondSwipeIndexShot = 
            mAutos.timedIndexShot(kShotTime2Seconds, kShotEndTimeSeconds);

        Trigger hasSecondShotEnded = auto.loggedCondition(
            mSecondSwipePathName+"/SecondShotEnded", 
            () -> (secondSwipeIntakeShot.hasEnded() && secondSwipeIndexShot.hasEnded()),
            true);

        SequentialEndingCommandGroup goToPreClimbPose = new SequentialEndingCommandGroup(
            mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getClimbEndPose(mClimbPose.transformBy(new Transform2d(0.0, -0.05, Rotation2d.kZero))), 
                ConstraintType.LINEAR));

        Trigger atPreClimbPose = auto.loggedCondition(
            "ClimbEnd/AtPreClimbPose", 
            () -> 
                goToPreClimbPose.isRunning()
                    &&
                mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        SequentialEndingCommandGroup goToClimbPose = new SequentialEndingCommandGroup(
            mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getClimbEndPose(mClimbPose), 
                ConstraintType.LINEAR));

        Trigger atClimbPose = auto.loggedCondition(
            "ClimbEnd/ReadyToPreClimb", 
            () -> 
                goToPreClimbPose.isRunning()
                    &&
                mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        SequentialEndingCommandGroup prepareForClimb = 
            new SequentialEndingCommandGroup(mClimbSS.goUpTillClimbHeightThenStay());

        Trigger preparedForClimb = auto.loggedCondition(
            "ClimbEnd/PreparedForClimb", 
            () -> prepareForClimb.hasEnded(), 
            true);

        SequentialEndingCommandGroup climb = 
            new SequentialEndingCommandGroup(mClimbSS.goUpTillClimbHeightThenStay());

        Trigger hasClimbed = auto.loggedCondition(
            "ClimbEnd/HasClimbed", 
            () -> climb.hasEnded(), 
            true);

        //////////////////// FIRST SWIPE \\\\\\\\\\\\\\\\\\\\\\\\\\\
        autoActivted
            .onTrue(Commands.waitSeconds(0.5).andThen(firstSwipePath))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(Commands.runOnce(() -> mWantToShoot = false));

        firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getSwipeEndPose(lastPoseOfFirstSwipe),
                ConstraintType.LINEAR));

        firstSwipePath.hasEnded().and(() -> mWantToShoot).and(hasFirstShotEnded.negate()).and(inShootingToleranceDebounced)
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        firstSwipePath.hasEnded().and(hasFirstShotEnded)
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(secondSwipePath);

        //////////////////// SECOND SWIPE \\\\\\\\\\\\\\\\\\\\\\\\\\\
        secondSwipePath.atTime(mSecondSwipeSwitchToAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getSwipeEndPose(lastPoseOfSecondSwipe),
                ConstraintType.LINEAR));

        secondSwipePath.hasEnded().and(() -> mWantToShoot).and(hasSecondShotEnded.negate()).and(inShootingToleranceDebounced)
            .onTrue(secondSwipeIntakeShot)
            .onTrue(secondSwipeIndexShot)
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        secondSwipePath.hasEnded().and(hasSecondShotEnded)
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(goToPreClimbPose)
            .onTrue(prepareForClimb);

        /* GO CLIMB */
        atPreClimbPose.and(preparedForClimb)
            .onTrue(goToClimbPose);

        atClimbPose.and(preparedForClimb)
            .onTrue(climb);

        hasClimbed
            .onTrue(mAutos.endAuto(auto));

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

    private Pose2d getClimbEndPose(Pose2d pose) {
        return AllianceFlipUtil.apply(pose);
    }
}