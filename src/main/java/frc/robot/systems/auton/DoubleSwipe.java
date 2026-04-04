package frc.robot.systems.auton;

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

public class DoubleSwipe extends Auton {
    private boolean mWantToShoot = false;

    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeSwitchToAlignTime;
    private final String mSecondSwipePathName;
    private final double mSecondSwipeSwitchToAlignTime;

    private final double kShotTime1Seconds = 3.0;
    private final double kShotTime2Seconds = 6.5;
    private final double kShotEndTimeSeconds = 0.02; 

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
        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = mAutos.inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(
            auto.getName()+"/WantToShoot", 
            () -> mWantToShoot, 
            true);

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

        //////////////////// FIRST SWIPE \\\\\\\\\\\\\\\\\\\\\\\\\\\
        autoActivted
            .onTrue(Commands.waitSeconds(0.5).andThen(firstSwipePath))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(Commands.runOnce(() -> mWantToShoot = false));

        firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> AllianceFlipUtil.apply(
                    new Pose2d(
                        lastPoseOfFirstSwipe.getX(),
                        lastPoseOfFirstSwipe.getY(),
                        GameGoalPoseChooser.turnFromHub(AllianceFlipUtil.apply(lastPoseOfFirstSwipe))
                            .plus((AllianceFlipUtil.shouldFlip()) ? Rotation2d.k180deg : Rotation2d.kZero)
                    )
                ),
                ConstraintType.LINEAR));

        SequentialEndingCommandGroup firstSwipeIntakeShot = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(kShotTime1Seconds),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(kShotEndTimeSeconds));

        SequentialEndingCommandGroup firstSwipeIndexShot = 
            new SequentialEndingCommandGroup(
                mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(kShotTime1Seconds),
                mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(kShotEndTimeSeconds));

        Trigger hasFirstShotEnded = auto.loggedCondition(
            mFirstSwipePathName+"/FirstShotEnded", 
            () -> (firstSwipeIntakeShot.hasEnded() && firstSwipeIndexShot.hasEnded()),
            true);

        firstSwipePath.hasEnded().and(() -> mWantToShoot).and(hasFirstShotEnded.negate()).and(() -> 
            mFlywheelsSS.atLatestClosedLoopGoal() && 
            mHoodSS.atGoal() &&
            mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean())
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        firstSwipePath.hasEnded().and(() -> firstSwipeIndexShot.hasEnded() && firstSwipeIntakeShot.hasEnded())
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(secondSwipePath);

        //////////////////// SECOND SWIPE \\\\\\\\\\\\\\\\\\\\\\\\\\\
        secondSwipePath.atTime(mSecondSwipeSwitchToAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> AllianceFlipUtil.apply(
                    new Pose2d(
                        lastPoseOfSecondSwipe.getX(),
                        lastPoseOfSecondSwipe.getY(),
                        GameGoalPoseChooser.turnFromHub(AllianceFlipUtil.apply(lastPoseOfSecondSwipe))
                            .plus((AllianceFlipUtil.shouldFlip()) ? Rotation2d.k180deg : Rotation2d.kZero)
                    )
                ),
                ConstraintType.LINEAR));

        SequentialEndingCommandGroup secondSwipeIntakeShot = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(kShotTime2Seconds),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(kShotEndTimeSeconds));

        SequentialEndingCommandGroup secondSwipeIndexShot = 
            new SequentialEndingCommandGroup(
                mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(kShotTime2Seconds),
                mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(kShotEndTimeSeconds));

        Trigger hasSecondShotEnded = auto.loggedCondition(
            mFirstSwipePathName+"/FirstShotEnded", 
            () -> (firstSwipeIntakeShot.hasEnded() && firstSwipeIndexShot.hasEnded()),
            true);

        secondSwipePath.hasEnded().and(() -> mWantToShoot).and(hasSecondShotEnded.negate()).and(() -> 
            mFlywheelsSS.atLatestClosedLoopGoal() && 
            mHoodSS.atGoal() &&
            mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean())
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        secondSwipePath.hasEnded().and(() -> secondSwipeIndexShot.hasEnded() && secondSwipeIntakeShot.hasEnded())
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(secondSwipePath);

        return auto;
    }
}
