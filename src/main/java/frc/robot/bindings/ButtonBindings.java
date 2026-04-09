package frc.robot.bindings;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.lib.controls.TurnPointFeedforward;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.bindings.BindingsConstants.ButtonBoardPorts;
import frc.robot.commands.DriveCharacterizationCommands;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.DriveManager.DriveState;
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
    // private final ClimbSS mClimbSS;
    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final FlydigiApex4 mGunnerController = new FlydigiApex4(BindingsConstants.kGunnerControllerPort);

    private final LoggedNetworkBoolean kUsingPilotGunner = new LoggedNetworkBoolean("DriverOperator/UsePilotGunner", true);

    private final CommandGenericHID mButtonBoard = new CommandGenericHID(2);

    HoodStates prevHoodState = HoodStates.STOPPED;
    DriveState prevDriveState = DriveState.TELEOP;
    HeadingTraversalState mHeadingTraversalState = HeadingTraversalState.NONE;

    private boolean inCenterFlag = false; 

    public ButtonBindings(Drive pDriveSS, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS, Intake pIntake) {
        this.mDriveSS = pDriveSS;
        this.mFuelPumpSS = pFuelPumpSS;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mIntakeSS = pIntake;
        // this.mClimbSS = pClimbSS;
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
    }

    public void initButtonBoardBindings(){
        final GenericHID mButtonBoard1 = new GenericHID(1);
        final GenericHID mButtonBoard2 = new GenericHID(2); 

        Trigger wantToClimbUp = new Trigger(() -> mButtonBoard1.getRawButton(1));
        Trigger wantToClimbDescend = new Trigger(() -> mButtonBoard1.getRawButton(2));
        Trigger wantToTrashCompact = new Trigger(() -> mButtonBoard1.getRawButton(3));

        Trigger wantToSlowIntake = new Trigger(() -> mButtonBoard2.getRawButton(6));
        Trigger wantToIntakeOut = new Trigger(() -> mButtonBoard1.getRawButton(4));

        Trigger wantToOutakeRoller = new Trigger(() -> mButtonBoard2.getRawButton(5));
        Trigger wantToIntakeRoller = new Trigger(() -> mButtonBoard2.getRawButton(4));

        Trigger wantToRevFlywheels = new Trigger(() -> mButtonBoard2.getRawAxis(0) < 0.5);
        Trigger wantToStopFlywheels = new Trigger(() -> mButtonBoard2.getRawAxis(0) > 0.5);


        Trigger wantToBumpShot = new Trigger(() -> mButtonBoard1.getRawButton(5));
        Trigger wantToClimbShot = new Trigger(() -> mButtonBoard1.getRawButton(6));
        Trigger wantToTrenchShot = new Trigger(() -> mButtonBoard2.getRawButton(9));
        Trigger wantToCornerShot = new Trigger(() -> mButtonBoard2.getRawAxis(1) > 0.5);

        Trigger wantToHailstorm = new Trigger(() -> mButtonBoard2.getRawButton(10));
        Trigger wantToSnowPlow = new Trigger(() -> mButtonBoard2.getRawButton(7));
        Trigger wantToShoot = new Trigger((() -> mButtonBoard2.getRawAxis(1) < 0.5));

        Trigger wantToDisableCams = new Trigger(() -> mButtonBoard2.getRawButton(1));
        Trigger wantToDisableCANRange = new Trigger(() -> mButtonBoard2.getRawButton(2));
        Trigger wantToAfterBurn = new Trigger(() -> mButtonBoard2.getRawButton(3));


    }

    public void initCompBindings() {
        boolean closedLoopFuelPump = true;
        
        // PILOT CONTROLS
        Trigger wantToAutoAlignToHub = mPilotController.a();
        Trigger wantToTraverse = mPilotController.rightTrigger().and(kUsingPilotGunner);
        Trigger wantToSafeStow = mPilotController.leftBumper().and(kUsingPilotGunner);
        Trigger wantToLineAlignToBump = mPilotController.b();
        Trigger wantToLineAlignToTrench = mPilotController.y();
        Trigger wantsToHeadingXLock = mPilotController.x();
          
        // PILOT AND GUNNER CONTROLS
        Trigger wantToIntake = 
            mPilotController.rightBumper().or(
            mGunnerController.rightBumper().or(
                mButtonBoard.button(ButtonBoardPorts.kWhiteSquare))).and(kUsingPilotGunner);

        // GUNNER CONTROLS
        Trigger wantToDynamicShoot = mGunnerController.rightTrigger().or(mButtonBoard.button(ButtonBoardPorts.kGreenTriangle)).and(kUsingPilotGunner);
        Trigger wantToOuttake = mGunnerController.leftTrigger().or(mButtonBoard.button(ButtonBoardPorts.kBlueSquare)).and(kUsingPilotGunner);
        Trigger wantToOpponentFeed = mGunnerController.a().or(mButtonBoard.button(ButtonBoardPorts.kBlackRectangle)).and(kUsingPilotGunner);
        Trigger wantToFeedWithNoCompact = mGunnerController.b().and(kUsingPilotGunner);
        Trigger wantToFeed = mGunnerController.y().or(mButtonBoard.button(ButtonBoardPorts.kBlueRectangle)).and(kUsingPilotGunner);
        Trigger wantToStow = mGunnerController.leftBumper().or(mButtonBoard.button(ButtonBoardPorts.kRedSquare)).and(kUsingPilotGunner);

        // OTHER CONDITIONAL TRIGGERS
        Trigger autonomousWorking = new Trigger(() -> true);
        Trigger inCenter = new Trigger(() -> GameGoalPoseChooser.inCenter(mDriveSS.getPoseEstimate()));
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
        Trigger shooterAtGoal = hoodAtGoal.and(flywheelAtGoal);
        Trigger atHeadingGoal = (headingAlignAtGoal.or(driveIsHeadingXLocked));


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

        wantToAutoAlignToHub.and(autonomousWorking)
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> GameGoalPoseChooser.getCloseShotPose(), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

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

        wantToFeed.and(shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))
            .onTrue(mFuelPumpSS.setStateCmd(closedLoopFuelPump ? FuelPumpState.INTAKE_VELOCITY : FuelPumpState.INTAKE_VOLT))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.trashCompactPivotRepeat());

        wantToFeedWithNoCompact.or(wantToOpponentFeed).and(shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))
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

        // wantToInitiateClimb
        //     .onTrue(mClimbSS.goUpTillClimbHeightThenStay())
        //     .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));

        // wantToEndClimb
        //     .onTrue(mClimbSS.goDownTillClimbedThenStayClimbed())
        //     .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));
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

        mPilotController.leftTrigger().and(isTesting())
            .onTrue(DriveCharacterizationCommands.testDriveAmpCharacterization(mDriveSS))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

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

        // Trigger climbButton = new Trigger(() -> mHBSS.getClimbButtonUpdateInputs().iPressed);
        // climbButton.and(() -> DriverStation.isDisabled() && !DriverStation.isFMSAttached())
        //     .onFalse(new InstantCommand(() -> mClimbSS.changeClimbNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true))
            // .onTrue(new InstantCommand(() -> mClimbSS.changeClimbNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
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
}
