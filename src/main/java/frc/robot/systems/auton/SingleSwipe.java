package frc.robot.systems.auton;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class SingleSwipe extends Auton {
    private boolean mWantToShoot = false;
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeAlignTime;

    private final double kShotTimeSeconds = 6.5;
    private final double kShotEndTimeSeconds = 0.02; 

    public SingleSwipe(AutonCommands pAutos, String pAutoName, String pFirstSwipePathName, double pFirstSwipeAlignTime) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeAlignTime = pFirstSwipeAlignTime;
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
            .debounce(0.05, DebounceType.kRising)
            .debounce(5.0, DebounceType.kFalling);

        FollowPathCommand firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true, auto);

        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        SequentialEndingCommandGroup firstSwipeIntakeShot = 
            mAutos.timedIntakeShot(kShotTimeSeconds, kShotEndTimeSeconds);

        SequentialEndingCommandGroup firstSwipeIndexShot = 
            mAutos.timedIndexShot(kShotTimeSeconds, kShotEndTimeSeconds);

        Trigger hasFirstShotEnded = auto.loggedCondition(
            mFirstSwipePathName+"/FirstShotEnded", 
            () -> (firstSwipeIntakeShot.hasEnded() && firstSwipeIndexShot.hasEnded()),
            true);

        intakingRange
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        autoActivted
            .onTrue(Commands.waitSeconds(0.5).andThen(firstSwipePath))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(Commands.runOnce(() -> mWantToShoot = false));

        firstSwipePath.atTime(mFirstSwipeAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getSwipeEndPose(lastPoseOfFirstSwipe),
                ConstraintType.LINEAR));

        firstSwipePath.hasEnded().and(() -> mWantToShoot).and(hasFirstShotEnded.negate()).and(inShootingToleranceDebounced)
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        firstSwipePath.hasEnded().and(() -> firstSwipeIndexShot.hasEnded() && firstSwipeIntakeShot.hasEnded())
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
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
}
