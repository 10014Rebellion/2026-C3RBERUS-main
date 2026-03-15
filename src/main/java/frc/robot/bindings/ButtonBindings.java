package frc.robot.bindings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.climb.ClimbSS.ClimbState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotStates;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelState;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodSS.HoodClosedSetpoints;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final FuelPumpSS mFuelPumpSS;
    private final HoodSS mHoodSS;
    private final FlywheelsSS mFlywheelsSS;
    private final Intake mIntakeSS;
    private final ClimbSS mClimbSS;
    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final FlydigiApex4 mGunnerController = new FlydigiApex4(BindingsConstants.kGunnerControllerPort);

    private final CommandGenericHID mHID = new CommandGenericHID(2);

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
        // initPilotBindings();
        // initGunnerBindings();
        initButtonBoard();
        initTriggers();
    }

    public void initTriggers() {
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.getDriveManager().setToTeleop()
                .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .alongWith(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE))
                .alongWith(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED))
            );
        
        new Trigger(() -> DriverStation.isFMSAttached()).and(() -> DriverStation.isTeleopEnabled())
            .onTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN)
                .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE)));
    }

    public void initButtonBoard() {
        // mHID.button(3)
        //     .whileTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.TUNING))
        //     .whileFalse(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE));

        // mHID.button(4)
        //     .whileTrue(mHoodSS.setHoodTuneableCmdNoEnd())
        //     .onFalse(Commands.runOnce(() -> mHoodSS.clearGoalWithoutStateChange(), mHoodSS));

        // mHID.button(9)
        //     .onTrue(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.INTAKE)
        //         .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.CONVEY_TO_INDEX))
        //         .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //         .alongWith(mIntakeSS.trashCompact()))
                
        //     .onalongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))))
        //     .onFalse(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED)
        //         .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE)
        //         .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //         .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE))));
        mDriveSS.getDriveManager().acceptJoystickInputs(
            () -> -mPilotController.getLeftY(),
            () -> -mPilotController.getLeftX(),
            () -> -mPilotController.getRightX(),
            () -> mPilotController.getPOVAngle());
            
        mPilotController.a()
            .onTrue(mIntakeSS.trashCompactPivotContinuous())
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
            
        mPilotController.b()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mPilotController.x()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.TUNING_VOLTAGE))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.TUNING))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INVALID))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.INVALID));

        mPilotController.y()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.TUNING_AMPS))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INVALID));

        mPilotController.a()
            .onTrue(mClimbSS.goUpTillClimbHeightThenStay())
            .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));;
        
        mPilotController.b()
            .onTrue(mClimbSS.goDownTillClimbedThenStayClimbed())
            .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));

        mPilotController.a()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> GameGoalPoseChooser.getHubPresetPose(
                    mDriveSS.getPoseEstimate(), 
                    Units.inchesToMeters(113.0)), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.b()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> AllianceFlipUtil.apply(new Pose2d(
                    3.351194381713867 - 0.1, 
                    4.036095142364502, Rotation2d.kZero)), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.x()
            .onTrue(mDriveSS.getDriveManager().setToGenericLineAlign(
                () -> new Pose2d(3.3, 3.3, Rotation2d.fromDegrees(0)),
                () -> Rotation2d.fromDegrees(0.0), 
                () -> 1.0, 
                () -> false))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.y()
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()));
    }

    public Command rumbleDriverController(){
        return Commands.startEnd(
            () -> mPilotController.setRumble(RumbleType.kBothRumble, 1.0), 
            () -> mPilotController.setRumble(RumbleType.kBothRumble, 0.5));
    }

    // public BooleanSupplier shootingReady(){
    //     return () -> mShooter.getIsFlywheelAtGoal() && mShooter.getIsHoodAtGoal() && mDriveSS.getDriveManager().getAutoAlignController().atGoal();
    // }
    
    public void initPilotBindings() {
        mPilotController.startButton().onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        // mPilotController.a()
        //     .onTrue(DriveCharacterizationCommands.runDriveAmpCharacterization(80, mDriveSS))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.b()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //         () -> GameGoalPoseChooser.getHubPresetPose(
        //             mDriveSS.getPoseEstimate(), 
        //             2.25), 
        //         ConstraintType.LINEAR))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.a()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> GameGoalPoseChooser.getHubPresetPose(
                    mDriveSS.getPoseEstimate(), 
                    Units.inchesToMeters(113.0)), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.b()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> AllianceFlipUtil.apply(new Pose2d(
                    3.351194381713867 - 0.1, 
                    4.036095142364502, Rotation2d.kZero)), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mDriveSS.getDriveManager().acceptJoystickInputs(
                () -> -mPilotController.getLeftY(),
                () -> -mPilotController.getLeftX(),
                () -> -mPilotController.getRightX(),
                () -> mPilotController.getPOVAngle());

        mPilotController.startButton().onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        // mPilotController.a()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(() -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), mDriveSS.getDriveManager().getDefaultTurnPointFF()))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.y()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(() -> PoseConstants.kClimbPose, ConstraintType.LINEAR)) // TODO: TUNE ME
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.povUp()
            .onTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.CLOSE_SHOT))
            .onTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.SHOOT_CLOSE_VELOCITY));
        
        mPilotController.povDown()
            .onTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.TOWER_SHOT))
            .onTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.SHOOT_MID_VELOCITY));

        mPilotController.povRight()
            .onTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MAX))
            .onTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.SNOW_BLOW));
        
        mPilotController.povLeft()
            .onTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN))
            .onTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE));

        mPilotController.leftBumper()
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mPilotController.rightBumper()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE)))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE)));
          
        mPilotController.leftTrigger()
            .whileTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.SNOW_BLOW))
            .whileFalse(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE));

        /* TODO: TUNE */
        mPilotController.y()
            .whileTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.SHOOT_CLOSE_VELOCITY))
            // .whileTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.CLOSE_SHOT))
            .whileFalse(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.STANDBY));
            // .whileFalse(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN));

        mPilotController.rightTrigger().whileTrue(
            new SequentialCommandGroup(
                // mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.UNJAM).withTimeout(0.4),
                // mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.INTAKE).alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.UNJAM)).until(()->mFuelPumpSS.isReadyToShoot()),
                mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.INTAKE)
                .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            )
        )
                .onFalse(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED))
                // .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                // .alongWith(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN))
                // .alongWith(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE))
                .onFalse(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.STANDBY))
                .onFalse(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN));
    }

    public void initGunnerBindings() {

        mGunnerController.povUp()
            .onTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MAX))
            .onFalse(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN));
        
        // mGunnerController.povDown()
        //     .whileTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN));

        mGunnerController.povRight()
            .whileTrue(mHoodSS.incrementAngleCmd());
        
        mGunnerController.povLeft()
            .whileTrue(mHoodSS.decrementAngleCmd());

        // mGunnerController.a().whileTrue(mClimbSS.unHookClawsCmd());
        // mGunnerController.b().whileTrue(mClimbSS.hookClawsCmd());

        mGunnerController.leftBumper().onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW));
        mGunnerController.rightBumper().onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE));

        mGunnerController.leftTrigger()
            .whileTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mGunnerController.rightTrigger()
            .whileTrue((mIntakeSS.trashCompactPivotContinuous()))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE));

        // new Trigger(() -> (mGunnerController.getLeftY() < -0.25))
        //     .onTrue(mClimbSS.setClimbPositionManualCmd(2.47))
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //     .onFalse(mClimbSS.setClimbVoltsCmd(0));
        
        // new Trigger(() -> (mGunnerController.getLeftY() > 0.25))
        //     .onTrue(mClimbSS.setClimbVoltsCmd(-10))
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotStates.STOW))
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //     .onFalse(mClimbSS.setClimbVoltsCmd(0));
        
        new Trigger(
            () -> mGunnerController.getRightY() < -0.25)
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        new Trigger(
            () -> mGunnerController.getRightY() > 0.25)
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
                .onTrue(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.OUTTAKE))

                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onFalse(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED));

    }

    public void testBindings() {
        // if(!DriverStation.isFMSAttached()) {
        //     mPilotController.x()
        //         .onTrue(Commands.runOnce(() -> mDriveSS.setPose(AllianceFlipUtil.apply(new Pose2d(
        //             3.351194381713867, 
        //             4.036095142364502, Rotation2d.kZero)))));
        // }
    }
}
