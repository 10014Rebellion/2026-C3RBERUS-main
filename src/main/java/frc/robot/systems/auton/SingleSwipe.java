package frc.robot.systems.auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.game.GameGoalPoseChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;

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
        AutoEvent auto = new AutoEvent(mAutoName, mAutos, mAutos.getAutoFactory());

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = mAutos.inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(auto.getName()+"/WantToShoot", () -> mWantToShoot, true);

        SequentialEndingCommandGroup firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true);
        Trigger isFirstSwipeRunning = auto.loggedCondition(mFirstSwipePathName+"/isRunning", () -> firstSwipePath.isRunning(), true);
        Trigger hasFirstSwipeEnded = auto.loggedCondition(mFirstSwipePathName+"/hasEnded", () -> firstSwipePath.hasEnded(), true);

        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        intakingRange
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        auto.loggedCondition(
            mFirstSwipePathName+"/InShotRadius", 
            () -> mDriveSS.getPoseEstimate().minus(lastPoseOfFirstSwipe).getTranslation().getNorm() < 0.5, 
            true)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true));

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        autoActivted
            .onTrue(Commands.waitSeconds(0.5).andThen(firstSwipePath))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(Commands.runOnce(() -> mWantToShoot = false));

        hasFirstSwipeEnded
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
            mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean())
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        hasFirstSwipeEnded.and(() -> firstSwipeIndexShot.hasEnded() && firstSwipeIntakeShot.hasEnded())
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(mAutos.endAuto(auto));

        return auto;
    }
}
