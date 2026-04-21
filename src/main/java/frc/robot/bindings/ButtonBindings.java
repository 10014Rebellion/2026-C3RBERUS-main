package frc.robot.bindings;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.lib.controllers.RebelButtonBoardRebuilt;
import frc.lib.controls.TurnPointFeedforward;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.DriveCharacterizationCommands;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.LEDs.ledConstants.LEDColor;
import frc.robot.systems.LEDs.ledSS;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.DriveManager.DriveState;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.efi.FuelInjectorSS.FuelInjectorState;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
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
    private final FuelInjectorSS mFuelInjectorSS;
    private final ledSS mLEDSS;
    // private final ClimbSS mClimbSS;
    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final RebelButtonBoardRebuilt mGunnerButtonboard = new RebelButtonBoardRebuilt(1, 2);

    private final LoggedNetworkBoolean kUsingPilotGunner = new LoggedNetworkBoolean("DriverOperator/UsePilotGunner", true);


    HoodStates prevHoodState = HoodStates.STOPPED;
    DriveState prevDriveState = DriveState.TELEOP;
    HeadingTraversalState mHeadingTraversalState = HeadingTraversalState.NONE;

    private boolean inCenterFlag = false; 

    public ButtonBindings(Drive pDriveSS, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS, Intake pIntake, FuelInjectorSS pInjectorSS, ledSS pLEDSS) {
        this.mDriveSS = pDriveSS;
        this.mFuelPumpSS = pFuelPumpSS;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mIntakeSS = pIntake;
        this.mFuelInjectorSS = pInjectorSS;
        this.mLEDSS = pLEDSS;
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
        // testBindings();
    }

    public void initCompBindings() {
        boolean closedLoopFuelPump = true;
        
        // PILOT CONTROLS
        Trigger wantToAutoAlignToHub = mPilotController.a().and(kUsingPilotGunner);
        Trigger wantToSafeStow = mPilotController.leftBumper().and(kUsingPilotGunner);
        Trigger wantToLineAlignToBump = mPilotController.b().and(kUsingPilotGunner);
        // Trigger wantToYawToBump = mPilotController.rightTrigger().and(kUsingPilotGunner);
        Trigger wantToLineAlignToTrench = mPilotController.y().and(kUsingPilotGunner);
        Trigger wantsToHeadingXLock = mPilotController.x().and(kUsingPilotGunner);
        Trigger wantToIntake = mPilotController.rightBumper().and(kUsingPilotGunner);

        // GUNNER CONTROLS
        Trigger wantToDynamicShoot = mGunnerButtonboard.blueSquareRight().and(kUsingPilotGunner);
        Trigger wantToDeployClimb = mGunnerButtonboard.whiteUpwardTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToClimbAscend = mGunnerButtonboard.whiteDownwardTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToTrashCompact = mGunnerButtonboard.greenDiamondLeft().and(kUsingPilotGunner);
        Trigger wantToStowIntake = mGunnerButtonboard.yellowTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToIntakeOut = mGunnerButtonboard.redTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToOutakeRoller = mGunnerButtonboard.redCircleBottom().and(kUsingPilotGunner);
        Trigger wantToIntakeRoller = mGunnerButtonboard.blueCircleBottom().and(kUsingPilotGunner);
        Trigger wantToRevFlywheels = mGunnerButtonboard.yellowTriangleRight().and(kUsingPilotGunner);
        Trigger wantToStopFlywheels = mGunnerButtonboard.redTriangleRight().and(kUsingPilotGunner);
        Trigger wantToBumpShot = mGunnerButtonboard.redSquareCenter().and(kUsingPilotGunner);
        Trigger wantToClimbShot = mGunnerButtonboard.blueSquareCenter().and(kUsingPilotGunner);
        Trigger wantToTrenchShot = mGunnerButtonboard.greenSquareCenter().and(kUsingPilotGunner);
        Trigger wantToCornerShot = mGunnerButtonboard.yellowSquareCenter().and(kUsingPilotGunner);
        Trigger wantToHailstorm = mGunnerButtonboard.whiteSquareRight().and(kUsingPilotGunner);
        Trigger wantToSnowPlow = mGunnerButtonboard.yellowRectangleRight().and(kUsingPilotGunner);
        Trigger wantToDisableCams = mGunnerButtonboard.orangePilotTop().and(kUsingPilotGunner);
        Trigger wantToDisableCANRange = mGunnerButtonboard.purplePilotTop().and(kUsingPilotGunner);
        Trigger wantToAfterBurn = mGunnerButtonboard.bluePilotTop().and(kUsingPilotGunner);

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
        Trigger fuelPumpAtGoal = new Trigger(() -> mFuelPumpSS.atGoal());
        Trigger atHeadingGoal = (headingAlignAtGoal.or(driveIsHeadingXLocked));


        Trigger wantToShoot = new Trigger(() -> {
            return 
                wantToSnowPlow.getAsBoolean()
                    ||
                wantToHailstorm.getAsBoolean()
                    ||
                wantToDynamicShoot.getAsBoolean();
        });

        final double kShootingReadyDebounceSeconds = 0.25;


        new Trigger(() -> mIntakeSS.safeToRunRollers())
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
        
        wantToTrashCompact
            .onTrue(mIntakeSS.trashCompact())
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        wantToStowIntake
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.STOW))
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        wantToIntakeOut
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        wantToOutakeRoller
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        wantToIntakeRoller
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        wantToRevFlywheels
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE));
        
        wantToStopFlywheels
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED));

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
            .onTrue(Commands.runOnce(() -> inCenterFlag = inCenter.getAsBoolean()));

        wantToLineAlignToBump
            .onFalse(noneTraversalHeadingState().andThen(mDriveSS.getDriveManager().setToTeleop()));

        /* TRAVERSAL Logic */
        wantToLineAlignToBump.and(() -> inCenterFlag)
            .onTrue(centerTraversalHeadingState()
                .andThen(
                    mDriveSS.getDriveManager().setToGenericLineAlign(
                        () -> GameGoalPoseChooser.getClosestBump(mDriveSS.getPoseEstimate()), 
                        () -> Rotation2d.k180deg, 
                        () -> 1, 
                        () -> true
                    )
                )
            );

        wantToLineAlignToBump.and(() -> !inCenterFlag)
            .onTrue(allianceTraversalHeadingState().andThen(
                    mDriveSS.getDriveManager().setToGenericLineAlign(
                        () -> GameGoalPoseChooser.getClosestBump(mDriveSS.getPoseEstimate())
                            .transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)), 
                        () -> Rotation2d.k180deg, 
                        () -> 1, 
                        () -> false
                    )
                )
            );


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
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION))
            .onTrue(new InstantCommand(() -> mLEDSS.setSolidStripColor(LEDColor.CYAN.getLEDColor())));
;

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
        wantToDynamicShoot.and(fuelPumpAtGoal).and(shooterAtGoal.and(headingAlignAtGoal.or(atHeadingGoal)).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VELOCITY))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE));

        wantToDynamicShoot
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN))
            .onFalse(new InstantCommand(() -> mLEDSS.setSolidStripColorToAllianceColor()));

        /* Feeding Logic */
        wantToSnowPlow
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.FEED_VELOCITY))
            .onTrue(mFuelPumpSS.setStateCmd(closedLoopFuelPump ? FuelPumpState.INTAKE_VELOCITY : FuelPumpState.INTAKE_VOLT))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MAX))
            .onTrue(new InstantCommand(() -> mLEDSS.setSolidStripColor(LEDColor.YELLOW.getLEDColor())));

        wantToHailstorm
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.OPPONENT_FEED_VELOCITY))
            .onTrue(mFuelPumpSS.setStateCmd(closedLoopFuelPump ? FuelPumpState.INTAKE_VELOCITY : FuelPumpState.INTAKE_VOLT))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MAX))
            .onTrue(new InstantCommand(() -> mLEDSS.setSolidStripColor(LEDColor.WHITE.getLEDColor())));


        wantToSnowPlow.or(wantToHailstorm).and(fuelPumpAtGoal).and((shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth)))
            .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        wantToSnowPlow.debounce(0.75, DebounceType.kRising).or(wantToHailstorm.debounce(0.75, DebounceType.kRising)).and(((shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth))).negate())
            .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        wantToHailstorm.or(wantToSnowPlow)
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN))
            .onFalse(new InstantCommand(() -> mLEDSS.setSolidStripColorToAllianceColor()));
;

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
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            // .onTrue(mDriveSS.getDriveManager().setToTeleopSniper())
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
            // .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToSafeStow
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.SAFESTOW));

   
        // wantToInitiateClimb
        //     .onTrue(mClimbSS.goUpTillClimbHeightThenStay())
        //     .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));

        // wantToEndClimb
        //     .onTrue(mClimbSS.goDownTillClimbedThenStayClimbed())
        //     .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));

        /* LEDs DURING ALIGNMENT LOGIC */ 
            /* ALIGNMENT TO HUB LOGIC */
                if(atHeadingGoal.getAsBoolean() && wantToAutoAlignToHub.getAsBoolean()) {
                    new InstantCommand(() -> mLEDSS.setSolidStripColor(LEDColor.GREEN.getLEDColor()));
                }
                else if(!atHeadingGoal.getAsBoolean() && wantToAutoAlignToHub.getAsBoolean()) {
                    new InstantCommand(() -> mLEDSS.setBreatheStripColor(LEDColor.RED.getLEDColor()));
                }
                else {
                    new InstantCommand(() -> mLEDSS.setSolidStripColorToAllianceColor());
                }
            /* ALIGNMENT TO CLIMB LOGIC */
                //waiting for captains to commit code for climb auto align in main.



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
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOW));

        mPilotController.y().and(isTesting())
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOW));

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
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
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

        // mGunnerController.leftTrigger().and(isTesting())
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        // mGunnerController.rightTrigger().and(isTesting())
        //     .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
        //     .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION));
        
        // mGunnerController.rightTrigger().and(new Trigger(() -> mFlywheelsSS.atLatestClosedLoopGoal() && mHoodSS.atGoal()).debounce(0.08)).and(isTesting())
        //     .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onTrue(mIntakeSS.trashCompactPivotRepeat());

        // mGunnerController.rightTrigger().and(isTesting())
        //     .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
        //     .onFalse(mHoodSS.setStateCmd(HoodStates.STOPPED))
        //     .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE));

        // mGunnerController.povUp().and(isTesting())
        //     .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
        //         () -> AllianceFlipUtil.apply(Rotation2d.kZero), 
        //         () -> GameGoalPoseChooser.getHub()))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mGunnerController.povDown().and(isTesting())
        //     .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
        //         () -> AllianceFlipUtil.apply(Rotation2d.k180deg), 
        //         TurnPointFeedforward.zeroTurnPointFF()))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());
    }

    public void initTriggers() {
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.getDriveManager().setToTeleop())
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));
        
        new Trigger(() -> DriverStation.isFMSAttached()).and(() -> DriverStation.isTeleopEnabled())
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE));

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
