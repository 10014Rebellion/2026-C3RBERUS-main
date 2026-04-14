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
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.efi.FuelInjectorSS.FuelInjectorState;
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
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
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

        //////////////////// FIRST SWIPE \\\\\\\\\\\\\\\\\\\\\\\\\\\
        autoActivted
            .onTrue(firstSwipePath)
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(Commands.runOnce(() -> mWantToShoot = false));

        // firstSwipePath.hasEnded().negate().and(firstSwipePath.isRunning().negate()).and(new Trigger(() -> mIntakeSS.getRackState().equals(IntakeRackState.INTAKE) && mIntakeSS.getRackAtGoal()))
        //     .onTrue(firstSwipePath);

        firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlignWithGeneratorReset(
                () -> getSwipeEndPose(lastPoseOfFirstSwipe),
                ConstraintType.LINEAR));

        firstSwipePath.hasEnded().and(() -> mWantToShoot).and(hasFirstShotEnded.negate()).and(inShootingToleranceDebounced)
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
            .onTrue(mIntakeSS.trashCompact());

        firstSwipePath.hasEnded().and(hasFirstShotEnded)
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(mInjectorSS.setStateCmd(FuelInjectorState.IDLE))
            .onTrue(secondSwipePath);

        //////////////////// SECOND SWIPE \\\\\\\\\\\\\\\\\\\\\\\\\\\
        secondSwipePath.atTime(mSecondSwipeSwitchToAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlignWithGeneratorReset(
                () -> getSwipeEndPose(lastPoseOfSecondSwipe),
                ConstraintType.LINEAR));

        secondSwipePath.hasEnded().and(() -> mWantToShoot).and(hasSecondShotEnded.negate()).and(inShootingToleranceDebounced)
            .onTrue(secondSwipeIntakeShot)
            .onTrue(secondSwipeIndexShot)
            .onTrue(mInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
            .onTrue(mIntakeSS.trashCompact());

        secondSwipePath.hasEnded().and(hasSecondShotEnded)
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mInjectorSS.setStateCmd(FuelInjectorState.IDLE))
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