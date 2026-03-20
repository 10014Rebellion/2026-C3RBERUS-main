package frc.robot.bindings;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.lib.controls.TurnPointFeedforward;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.game.FieldConstants;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.game.GameDriveManager.GameDriveStates;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.climb.ClimbSS.ClimbState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.DriveManager.DriveState;
import frc.robot.systems.drive.controllers.ManualTeleopController;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotStates;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;;

public class ButtonBindings {
    public static enum HeadingTraversalState {
        ALLIANCE, CENTER, NONE
    }

    private final Drive mDriveSS;
    private final FuelPumpSS mFuelPumpSS;
    private final HoodSS mHoodSS;
    private final FlywheelsSS mFlywheelsSS;
    private final Intake mIntakeSS;
    private final ClimbSS mClimbSS;
    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final FlydigiApex4 mGunnerController = new FlydigiApex4(BindingsConstants.kGunnerControllerPort);

    private final LoggedNetworkBoolean kUsingPilotGunner = new LoggedNetworkBoolean("DriverOperator/UsePilotGunner", true);

    private final CommandGenericHID mHID = new CommandGenericHID(2);

    HoodStates prevHoodState = HoodStates.STOPPED;
    DriveState prevDriveState = DriveState.TELEOP;
    HeadingTraversalState mHeadingTraversalState = HeadingTraversalState.NONE;

    private final HardwareButtonsSS mHBSS = new HardwareButtonsSS();

    private boolean inCenterFlag = false;

    public ButtonBindings(Drive pDriveSS, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS, Intake pIntake, ClimbSS pClimbSS) {
        this.mDriveSS = pDriveSS;
        this.mFuelPumpSS = pFuelPumpSS;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mIntakeSS = pIntake;
        this.mClimbSS = pClimbSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.getDriveManager().setToTeleop());
    }


    public void initBindings() {
        initTriggers();

        mDriveSS.getDriveManager().acceptJoystickInputs(
            () -> -mPilotController.getLeftY(),
            () -> -mPilotController.getLeftX(),
            () -> -mPilotController.getRightX(),
            () -> mPilotController.getPOVAngle());

        initCompBindings();
        testBindings();
        initButtonBoard();
        // initPilotBindings();
        // initGunnerBindings();
    }

    public void initCompBindings() {
        boolean closedLoopFuelPump = true;
        boolean isEli = false;
        Trigger wantToOuttake = 
            mGunnerController.leftTrigger().and(kUsingPilotGunner);
        Trigger wantToDynamicShoot = 
            mGunnerController.rightTrigger().and(kUsingPilotGunner);
        Trigger wantToIntake = 
            (isEli ? 
                mPilotController.leftBumper() : 
                mPilotController.rightBumper()).or(
                    mGunnerController.rightBumper()).and(kUsingPilotGunner);
        Trigger wantToTraverse = mPilotController.rightTrigger().and(kUsingPilotGunner);
        Trigger wantToSafeStow = (isEli ? mPilotController.rightBumper() : mPilotController.leftBumper()).and(kUsingPilotGunner);
        Trigger wantToLineAlignToBump = mPilotController.b();
        Trigger wantToLineAlignToTrench = mPilotController.y();
        Trigger wantToOpponentFeed = mGunnerController.a();
        Trigger wantToFeedWithNoCompact = mGunnerController.b().and(kUsingPilotGunner);
        Trigger wantToFeed = mGunnerController.y().and(kUsingPilotGunner);
        Trigger wantToInitiateClimb = mGunnerController.povUp().and(kUsingPilotGunner);
        Trigger wantToEndClimb = mGunnerController.povDown().and(kUsingPilotGunner);
        Trigger wantToStow = mGunnerController.leftBumper().and(kUsingPilotGunner);

        Trigger autonomousWorking = new Trigger(() -> true);
        Trigger doesRobotWantToMove = new Trigger(() -> 
            (Math.abs(mPilotController.getLeftX()) > 0.1)
                ||
            (Math.abs(mPilotController.getLeftY()) > 0.1)
        );
        Trigger inCenter = new Trigger(() -> GameGoalPoseChooser.inCenter(mDriveSS.getPoseEstimate()));
        Trigger inCenterTraversalHeading = new Trigger(() -> mHeadingTraversalState.equals(HeadingTraversalState.ALLIANCE));
        Trigger inAllianceTraversalHeading = new Trigger(() -> mHeadingTraversalState.equals(HeadingTraversalState.CENTER));
        Trigger isRobotMoving = new Trigger(() -> !mDriveSS.isRobotStationary());
        Trigger driveIsHeadingXLocked = new Trigger(() -> mDriveSS.getDriveManager().getDriveState().equals(DriveState.HEADING_X_LOCK));
        
        Trigger inNoHoodZone = new Trigger(() -> 
            (GameGoalPoseChooser.inLeftTrenchYRange(mDriveSS.getPoseEstimate()) ||
                GameGoalPoseChooser.inEitherTrenchXRange(mDriveSS.getPoseEstimate()))
            ||
            (GameGoalPoseChooser.inRightTrenchYRange(mDriveSS.getPoseEstimate()) ||
                GameGoalPoseChooser.inEitherTrenchXRange(mDriveSS.getPoseEstimate()))
        );
        Trigger inSuperNoHoodZone = new Trigger(() -> 
            (GameGoalPoseChooser.inLeftTrenchYRange(mDriveSS.getPoseEstimate()) ||
                GameGoalPoseChooser.inEitherSuperTrenchXRange(mDriveSS.getPoseEstimate()))
            ||
            (GameGoalPoseChooser.inRightTrenchYRange(mDriveSS.getPoseEstimate()) ||
                GameGoalPoseChooser.inEitherSuperTrenchXRange(mDriveSS.getPoseEstimate()))
        );
        prevHoodState = mHoodSS.getHoodState();
        Trigger flywheelAtGoal = new Trigger(() -> mFlywheelsSS.atLatestClosedLoopGoal());
        Trigger hoodAtGoal = new Trigger(() -> mHoodSS.atGoal());
        Trigger headingAlignAtGoal = new Trigger(mDriveSS.getDriveManager().waitUntilHeadingAlignFinishes());
        Trigger autoAlignAtGoal = new Trigger(mDriveSS.getDriveManager().waitUntilAutoAlignFinishes());

        Trigger shooterAtGoal = hoodAtGoal.and(flywheelAtGoal);
        Trigger atPositionalGoal = autonomousWorking.negate().or(autoAlignAtGoal);
        Trigger atHeadingGoal = (headingAlignAtGoal.or(driveIsHeadingXLocked));

        Trigger wantsToHeadingXLock = mPilotController.x();

        Trigger wantToShoot = new Trigger(() -> {
            return 
                wantToFeed.getAsBoolean()
                    ||
                wantToFeedWithNoCompact.getAsBoolean()
                    ||
                wantToOpponentFeed.getAsBoolean()
                    ||
                wantToDynamicShoot.getAsBoolean();
        });

        final double kShootingReadyDebounceSeconds = 0.25;

        mPilotController.startButton()
            .onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        wantsToHeadingXLock
            .onTrue(mDriveSS.getDriveManager().setToHeadingXLock());

        // wantsToHeadingXLock
        //     .onFalse(new ConditionalCommand(
        //         new InstantCommand(), 
        //         mDriveSS.getDriveManager().setToTeleop(), 
        //         wantToShoot));

        wantsToHeadingXLock.negate().and(wantToShoot.negate())
            .onTrue(mDriveSS.getDriveManager().setToTeleop());

        /* AUTO ALIGNS TO HUB AND SHOOTS */
        // wantToCloseShoot
        //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY))
        //     .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT));

        // wantToCloseShoot.and(autonomousWorking)
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //         () -> GameGoalPoseChooser.getCloseShotPose(), 
        //         ConstraintType.LINEAR))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToLineAlignToTrench.and(autonomousWorking)
            .onTrue(
                mDriveSS.getDriveManager().setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestTrench(mDriveSS.getPoseEstimate()), 
                    () -> Rotation2d.kZero, 
                    () -> 1, 
                    () -> true
                )
            )
        .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToLineAlignToBump.and(autonomousWorking)
            .onTrue(
                mDriveSS.getDriveManager().setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestBump(mDriveSS.getPoseEstimate()), 
                    () -> Rotation2d.k180deg, 
                    () -> 1, 
                    () -> false
                )
            )
        .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // wantToCloseShoot.and(autonomousWorking).and(wantsToHeadingXLock.negate()).and(driveIsHeadingXLocked)
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //         () -> GameGoalPoseChooser.getCloseShotPose(), 
        //         ConstraintType.LINEAR));

        // wantToCloseShoot.and(shooterAtGoal.and(atPositionalGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))
        //     .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onTrue(mIntakeSS.trashCompactPivotRepeat());

        // wantToCloseShoot
        //     .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
        //     .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        /* SHOOTS FROM ANYWHERE IN OUR FLYWHEEL RANGE */
        wantToDynamicShoot
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION));

        wantToDynamicShoot.and(autonomousWorking)
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToDynamicShoot.and(autonomousWorking).and(wantsToHeadingXLock.negate()).and(driveIsHeadingXLocked)
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()));

        // If at goal, shoot it in
        wantToDynamicShoot.and(shooterAtGoal.and(headingAlignAtGoal.or(atHeadingGoal)).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VELOCITY))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        wantToDynamicShoot
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        /* Feeding Logic */
        wantToFeed.or(wantToFeedWithNoCompact)
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.FEED_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MAX));

        wantToOpponentFeed
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.OPPONENT_FEED_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MAX));

        wantToFeed.or(wantToOpponentFeed).or(wantToFeedWithNoCompact).and(autonomousWorking)
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> AllianceFlipUtil.apply(Rotation2d.k180deg), 
                TurnPointFeedforward.zeroTurnPointFF()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToFeed.or(wantToOpponentFeed).and(shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))
            .onTrue(mFuelPumpSS.setStateCmd(closedLoopFuelPump ? FuelPumpState.INTAKE_VELOCITY : FuelPumpState.INTAKE_VOLT))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        wantToFeedWithNoCompact.and(shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))
            .onTrue(mFuelPumpSS.setStateCmd(closedLoopFuelPump ? FuelPumpState.INTAKE_VELOCITY : FuelPumpState.INTAKE_VOLT))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        wantToFeed.or(wantToOpponentFeed).or(wantToFeedWithNoCompact).and(autonomousWorking).and(wantsToHeadingXLock.negate()).and(driveIsHeadingXLocked)
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()));

        wantToFeed.or(wantToOpponentFeed).or(wantToFeedWithNoCompact)
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        /* Makes flywheel stand by */
        wantToShoot.negate()
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN));

        /* HOOD PROTECTION LOGIC */
        inSuperNoHoodZone.or(inNoHoodZone.and(isRobotMoving))
            .onTrue(Commands.runOnce(() -> prevHoodState = mHoodSS.getHoodState())
                .andThen(mHoodSS.setStateCmd(HoodStates.MIN)))
            .onFalse(mHoodSS.setStateCmd(prevHoodState));

        /* INTAKE LOGIC */
        wantToIntake
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            // .onTrue(mDriveSS.getDriveManager().setToTeleopSniper())
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
            // .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToOuttake
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        wantToSafeStow
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.SAFESTOW));

        wantToStow
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW));

        wantToTraverse
            .onTrue(Commands.runOnce(() -> inCenterFlag = inCenter.getAsBoolean()));

        /* TRAVERSAL Logic */
        wantToTraverse.and(() -> inCenterFlag)
            .onTrue(centerTraversalHeadingState().andThen(
                mDriveSS.getDriveManager().setToGenericHeadingAlign(
                    () -> AllianceFlipUtil.apply(Rotation2d.kZero), 
                    TurnPointFeedforward.zeroTurnPointFF())));

        wantToTraverse.and(() -> !inCenterFlag)
            .onTrue(allianceTraversalHeadingState().andThen(
                mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> AllianceFlipUtil.apply(Rotation2d.k180deg), 
                TurnPointFeedforward.zeroTurnPointFF())));

        wantToTraverse
            .onFalse(noneTraversalHeadingState().andThen(mDriveSS.getDriveManager().setToTeleop()));

        wantToInitiateClimb
            .onTrue(mClimbSS.goUpTillClimbHeightThenStay())
            .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));

        wantToEndClimb
            .onTrue(mClimbSS.goDownTillClimbedThenStayClimbed())
            .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));
    }

    public void testBindings() {
        mPilotController.startButton().and(isTesting())
            .onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        mPilotController.a().and(isTesting())
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.TUNING_VELOCITY))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED));

        mPilotController.b().and(isTesting())
            .onTrue(mHoodSS.setStateCmd(HoodStates.TUNING_SETPOINT))
            .onFalse(mHoodSS.setStateCmd(HoodStates.STOPPED));
        
        mPilotController.x().and(isTesting())
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW));

        mPilotController.y().and(isTesting())
            .onTrue(mIntakeSS.trashCompactPivotRepeat())
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW));

        // mPilotController.y().and(isTesting())
        //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.TUNING_VELOCITY))
        //     .onTrue(mHoodSS.setStateCmd(HoodStates.TUNING_SETPOINT));

        // mPilotController.y().and(isTesting()).and(new Trigger(() -> mFlywheelsSS.atLatestClosedLoopGoal() && mHoodSS.atGoal()))
        //     .onTrue(mIntakeSS.trashCompactPivotRepeat())
        //     .onTrue(null);

            // TO DO: Tune this
        mPilotController.leftTrigger().and(isTesting())
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> AllianceFlipUtil.apply(Rotation2d.kZero), 
                () -> GameGoalPoseChooser.getHub()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.leftTrigger().and(isTesting())
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mPilotController.rightTrigger().and(isTesting())
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.x()
        //     .onTrue(mHoodSS.setStateCmd(HoodStates.TUNING_VOLTAGE))
        //     .onFalse(mHoodSS.setStateCmd(HoodStates.STOPPED));

        // mPilotController.rightTrigger().and(isTesting())
        //     .onTrue(DriveCharacterizationCommands.testAzimuthsVoltage(mDriveSS, 0, 1, 2, 3))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.leftTrigger().and(isTesting())
        //     .onTrue(DriveCharacterizationCommands.testDriveAmpCharacterization(mDriveSS))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.rightBumper().and(isTesting())
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mPilotController.leftBumper().and(isTesting())
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));

        mGunnerController.leftTrigger().and(isTesting())
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mGunnerController.rightTrigger().and(isTesting())
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION));
        
        mGunnerController.rightTrigger().and(new Trigger(() -> mFlywheelsSS.atLatestClosedLoopGoal() && mHoodSS.atGoal()).debounce(0.08)).and(isTesting())
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        mGunnerController.rightTrigger().and(isTesting())
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onFalse(mHoodSS.setStateCmd(HoodStates.STOPPED))
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE));

        mGunnerController.povUp().and(isTesting())
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> AllianceFlipUtil.apply(Rotation2d.kZero), 
                () -> GameGoalPoseChooser.getHub()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mGunnerController.povDown().and(isTesting())
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> AllianceFlipUtil.apply(Rotation2d.k180deg), 
                TurnPointFeedforward.zeroTurnPointFF()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

         
    }

    public void initTriggers() {
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.getDriveManager().setToTeleop())
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));
        
        new Trigger(() -> DriverStation.isFMSAttached()).and(() -> DriverStation.isTeleopEnabled())
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE));

        Trigger climbButton = new Trigger(() -> mHBSS.getClimbButtonUpdateInputs().iPressed);
        climbButton.and(() -> DriverStation.isDisabled() && !DriverStation.isFMSAttached())
            .onFalse(new InstantCommand(() -> mClimbSS.changeClimbNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true))
            .onTrue(new InstantCommand(() -> mClimbSS.changeClimbNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
    }

    public Command rumbleDriverController(){
        return Commands.startEnd(
            () -> mPilotController.setRumble(RumbleType.kBothRumble, 1.0), 
            () -> mPilotController.setRumble(RumbleType.kBothRumble, 0.5));
    }

    public BooleanSupplier isTesting() {
        return () -> !kUsingPilotGunner.get();
    }

    public Command centerTraversalHeadingState() {
        return Commands.runOnce(() -> mHeadingTraversalState = HeadingTraversalState.CENTER);
    }

    public Command allianceTraversalHeadingState() {
        return Commands.runOnce(() -> mHeadingTraversalState = HeadingTraversalState.ALLIANCE);
    }

    public Command noneTraversalHeadingState() {
        return Commands.runOnce(() -> mHeadingTraversalState = HeadingTraversalState.ALLIANCE);
    }

    public void initButtonBoard() {
       
        mPilotController.b()
            .onTrue(mDriveSS.getDriveManager().setToGenericLineAlign(
                () -> GameGoalPoseChooser.closestClimbPose(mDriveSS.getPoseEstimate()),
                () -> GameGoalPoseChooser.closestClimbPose(mDriveSS.getPoseEstimate()).getRotation(), 
                () -> 0.5, 
                () -> false))
            .onFalse(mDriveSS.getDriveManager().setDriveStateCommand(DriveState.TELEOP));

        // mHID.button(4)
        //     .whileTrue(mHoodSS.setStateCmd(HoodStates.TUNING_SETPOINT))
        //     .whileFalse(mHoodSS.setStateCmd(HoodStates.HOLD_POSITION));

        // // mHID.button(9)
        // //     .onTrue(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.INTAKE)
        // //         .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.CONVEY_TO_INDEX))
        // //         .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        // //         .alongWith(mIntakeSS.trashCompact()))
                
        // //     .onalongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))))
        // //     .onFalse(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED)
        // //         .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE)
        // //         .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        // //         .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE))));
        // mDriveSS.getDriveManager().acceptJoystickInputs(
        //     () -> -mPilotController.getLeftY(),
        //     () -> -mPilotController.getLeftX(),
        //     () -> -mPilotController.getRightX(),
        //     () -> mPilotController.getPOVAngle());
            
        // mPilotController.a()
        //     .onTrue(mIntakeSS.trashCompactPivotRepeat())
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
            
        // mPilotController.b()
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
        //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        // mPilotController.x()
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.TUNING_VOLTAGE))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.TUNING))
        //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INVALID))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.INVALID));

        // mPilotController.y()
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.TUNING_AMPS))
        //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INVALID));

        // mPilotController.a()
        //     .onTrue(mClimbSS.goUpTillClimbHeightThenStay())
        //     .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));;
        
        // mPilotController.b()
        //     .onTrue(mClimbSS.goDownTillClimbedThenStayClimbed())
        //     .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));

        // mPilotController.a()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //         () -> GameGoalPoseChooser.getHubPresetPose(
        //             mDriveSS.getPoseEstimate(), 
        //             Units.inchesToMeters(113.0)), 
        //         ConstraintType.LINEAR))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.b()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //         () -> AllianceFlipUtil.apply(new Pose2d(
        //             3.351194381713867 - 0.1, 
        //             4.036095142364502, Rotation2d.kZero)), 
        //         ConstraintType.LINEAR))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.x()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericLineAlign(
        //         () -> new Pose2d(3.3, 3.3, Rotation2d.fromDegrees(0)),
        //         () -> Rotation2d.fromDegrees(0.0), 
        //         () -> 1.0, 
        //         () -> false))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());
    }

        // public BooleanSupplier shootingReady(){
    //     return () -> mShooter.getIsFlywheelAtGoal() && mShooter.getIsHoodAtGoal() && mDriveSS.getDriveManager().getAutoAlignController().atGoal();
    // }
    
    public void initPilotBindings() {
    //     mPilotController.startButton().and(kUsingPilotGunner)
    //         .onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

    //     mPilotController.leftTrigger().and(kUsingPilotGunner)
    //         .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
    //         .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
    //         .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

    //     mPilotController.rightTrigger().and(kUsingPilotGunner)
    //         .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW));

    //     // mPilotController.a()
    //     //     .onTrue(DriveCharacterizationCommands.runDriveAmpCharacterization(80, mDriveSS))
    //     //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

    //     // mPilotController.b()
    //     //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
    //     //         () -> GameGoalPoseChooser.getHubPresetPose(
    //     //             mDriveSS.getPoseEstimate(), 
    //     //             2.25), 
    //     //         ConstraintType.LINEAR))
    //     //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

    //     mPilotController.a().and(kUsingPilotGunner)
    //         .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
    //             () -> GameGoalPoseChooser.getHubPresetPose(
    //                 mDriveSS.getPoseEstimate(), 
    //                 Units.inchesToMeters(113.0)), 
    //             ConstraintType.LINEAR))
    //         .onFalse(mDriveSS.getDriveManager().setToTeleop());

    //     mPilotController.b().and(kUsingPilotGunner)
    //         .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
    //             () -> AllianceFlipUtil.apply(new Pose2d(
    //                 3.351194381713867 - 0.1, 
    //                 4.036095142364502, Rotation2d.kZero)), 
    //             ConstraintType.LINEAR))
    //         .onFalse(mDriveSS.getDriveManager().setToTeleop());

    //     mDriveSS.getDriveManager().acceptJoystickInputs(
    //             () -> -mPilotController.getLeftY(),
    //             () -> -mPilotController.getLeftX(),
    //             () -> -mPilotController.getRightX(),
    //             () -> mPilotController.getPOVAngle());

    //     // mPilotController.a()
    //     //     .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(() -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), mDriveSS.getDriveManager().getDefaultTurnPointFF()))
    //     //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

    //     // mPilotController.y()
    //     //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(() -> PoseConstants.kClimbPose, ConstraintType.LINEAR)) // TODO: TUNE ME
    //     //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

    //     // mPilotController.povUp().and(kUsingPilotGunner)
    //     //     .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT))
    //     //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY));
        
    //     // mPilotController.povDown().and(kUsingPilotGunner)
    //     //     .onTrue(mHoodSS.setStateCmd(HoodStates.TOWER_SHOT))
    //     //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.TOWER_VELOCITY));

    //     // mPilotController.povRight().and(kUsingPilotGunner)
    //     //     .onTrue(mHoodSS.setStateCmd(HoodStates.MAX))
    //     //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.FEED_VELOCITY));
        
    //     // mPilotController.povLeft().and(kUsingPilotGunner)
    //     //     .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
    //     //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED));

    //     // mPilotController.leftBumper().and(kUsingPilotGunner)
    //     //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
    //     //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

    //     // mPilotController.rightBumper().and(kUsingPilotGunner)
    //     //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE)))
    //     //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE)));
          
    //     // mPilotController.leftTrigger().and(kUsingPilotGunner)
    //     //     .whileTrue(mFlywheelsSS.setStateCmd(FlywheelStates.FEED_VELOCITY))
    //     //     .whileFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED));

    //     // /* TODO: TUNE */
    //     // mPilotController.y().and(kUsingPilotGunner)
    //     //     .whileTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY))
    //     //     .whileFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE));

    //     // mPilotController.rightTrigger().and(kUsingPilotGunner)
    //     //     .whileTrue(
    //     //         new SequentialCommandGroup(
    //     //             mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT)
    //     //         )
    //     //     )
    //     //     .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
    //     //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
    //     //     .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE))
    //     //     .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));
    }

    public void initGunnerBindings() {
    //     mGunnerController.povUp().and(kUsingPilotGunner)
    //         .onTrue(mHoodSS.setStateCmd(HoodStates.MAX))
    //         .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));
        
    //     // mGunnerController.povDown()
    //     //     .whileTrue(mHoodSS.setStateCmd(HoodStates.MIN));

    //     mGunnerController.povRight().and(kUsingPilotGunner)
    //         .whileTrue(mHoodSS.setStateCmd(HoodStates.INCREMENTING))
    //         .whileFalse(mHoodSS.setStateCmd(HoodStates.HOLD_POSITION));
        
    //     mGunnerController.povLeft().and(kUsingPilotGunner)
    //         .whileTrue(mHoodSS.setStateCmd(HoodStates.DECREMENTING))
    //         .whileFalse(mHoodSS.setStateCmd(HoodStates.HOLD_POSITION));

    //     // mGunnerController.a().whileTrue(mClimbSS.unHookClawsCmd());
    //     // mGunnerController.b().whileTrue(mClimbSS.hookClawsCmd());

    //     mGunnerController.leftBumper().and(kUsingPilotGunner)
    //         .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW));
    //     mGunnerController.rightBumper().and(kUsingPilotGunner)
    //         .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE));

    //     mGunnerController.leftTrigger().and(kUsingPilotGunner)
    //         .whileTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
    //         .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

    //     mGunnerController.rightTrigger().and(kUsingPilotGunner)
    //         .whileTrue((mIntakeSS.trashCompactPivotRepeat()))
    //         .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE));

    //     // new Trigger(() -> (mGunnerController.getLeftY() < -0.25))
    //     //     .onTrue(mClimbSS.setClimbPositionManualCmd(2.47))
    //     //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW))
    //     //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
    //     //     .onFalse(mClimbSS.setClimbVoltsCmd(0));
        
    //     // new Trigger(() -> (mGunnerController.getLeftY() > 0.25))
    //     //     .onTrue(mClimbSS.setClimbVoltsCmd(-10))
    //     //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW))
    //     //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
    //     //     .onFalse(mClimbSS.setClimbVoltsCmd(0));
        
    //     new Trigger(
    //         () -> mGunnerController.getRightY() < -0.25).and(kUsingPilotGunner)
    //             .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
    //             .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

    //     new Trigger(
    //         () -> mGunnerController.getRightY() > 0.25).and(kUsingPilotGunner)
    //             .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
    //             .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.OUTTAKE_VOLT))

    //             .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
    //             .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));

    }
}
