package frc.robot.systems.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotStates;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.game.GameGoalPoseChooser;

public class SingleSwipe extends Auton {
    private boolean mWantToShoot = false;
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final boolean mUseChoreoLib;

    private final double kShotTimeSeconds = 6.5;
    private final double kShotEndTimeSeconds = 0.02; 

    public SingleSwipe(AutonCommands pAutos, String pAutoName, String pFirstSwipePathName, boolean pUseChoreoLib) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mUseChoreoLib = pUseChoreoLib;
    }

    @Override
    protected AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = mAutos.inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(auto.getName()+"/WantToShoot", () -> mWantToShoot, true);

        SequentialEndingCommandGroup firstSwipePath = (mUseChoreoLib) ?
            followChorePathUsingCL(mFirstSwipePathName, true)
                : 
            followChoreoPath(mFirstSwipePathName, true);
        Trigger isFirstSwipeRunning = auto.loggedCondition(mFirstSwipePathName+"/isRunning", () -> firstSwipePath.isRunning(), true);
        Trigger hasFirstSwipeEnded = auto.loggedCondition(mFirstSwipePathName+"/hasEnded", () -> firstSwipePath.hasEnded(), true);

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

        hasFirstSwipeEnded
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()));

        SequentialEndingCommandGroup firstSwipeIntakeShot = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(kShotTimeSeconds),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(kShotEndTimeSeconds));

        SequentialEndingCommandGroup firstSwipeIndexShot = 
            new SequentialEndingCommandGroup(
                mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(kShotTimeSeconds),
                mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(kShotEndTimeSeconds));

        Trigger hasFirstShotEnded = auto.loggedCondition(
            mFirstSwipePathName+"/FirstShotEnded", 
            () -> (firstSwipeIntakeShot.hasEnded() && firstSwipeIndexShot.hasEnded()),
            true);

        hasFirstSwipeEnded.and(() -> mWantToShoot).and(hasFirstShotEnded.negate()).and(() -> 
            mFlywheelsSS.atLatestClosedLoopGoal() && 
            mHoodSS.atGoal() &&
            mDriveSS.getDriveManager().inHeadingTolerance())
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        hasFirstSwipeEnded.and(() -> firstSwipeIndexShot.hasEnded() && firstSwipeIntakeShot.hasEnded())
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(mAutos.endAuto(auto));

        return auto;
    }
}
