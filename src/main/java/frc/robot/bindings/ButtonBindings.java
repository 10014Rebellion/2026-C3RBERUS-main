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
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.conveyor.ConveyorSS.ConveyorState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final FuelPumpSS mFuelPumpSS;
    private final HoodSS mHoodSS;
    private final FlywheelsSS mFlywheelsSS;
    private final Intake mIntakeSS;
    private final ClimbSS mClimbSS;
    private final ConveyorSS mConveyorSS;
    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final FlydigiApex4 mGunnerController = new FlydigiApex4(BindingsConstants.kGunnerControllerPort);

    private final CommandGenericHID mHID = new CommandGenericHID(2);

    public ButtonBindings(Drive pDriveSS, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS, Intake pIntake, ConveyorSS pConveyorSS, ClimbSS pClimbSS) {
        this.mDriveSS = pDriveSS;
        this.mFuelPumpSS = pFuelPumpSS;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mIntakeSS = pIntake;
        this.mConveyorSS = pConveyorSS;
        this.mClimbSS = pClimbSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.getDriveManager().setToTeleop());
    }


    public void initBindings() {

        initPilotBindings();
        initGunnerBindings();
        initButtonBoard();
        initTriggers();
    }

    public void initTriggers() {
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.getDriveManager().setToTeleop()
                .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE))
                .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .alongWith(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
                .alongWith(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            );
        
        new Trigger(() -> DriverStation.isFMSAttached()).and(() -> DriverStation.isTeleopEnabled())
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN)
                .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE)));
    }

    public void initButtonBoard() {
        mHID.button(3)
            .whileTrue(mFlywheelsSS.setStateCmd(FlywheelStates.TUNING_VELOCITY))
            .whileFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED));

        mHID.button(4)
            .whileTrue(mHoodSS.setStateCmd(HoodStates.TUNING_SETPOINT))
            .whileFalse(mHoodSS.setStateCmd(HoodStates.HOLD_POSITION));

        mHID.button(9)
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT)
                .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.CONVEY_TO_INDEX))
                .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .alongWith(mIntakeSS.trashCompact()))
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED)
                .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE)
                .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE))));
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
            .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY));
        
        mPilotController.povDown()
            .onTrue(mHoodSS.setStateCmd(HoodStates.TOWER_SHOT))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.TOWER_VELOCITY));

        mPilotController.povRight()
            .onTrue(mHoodSS.setStateCmd(HoodStates.MAX))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.FEED_VELOCITY));
        
        mPilotController.povLeft()
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED));

        mPilotController.leftBumper()
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mPilotController.rightBumper()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE)))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE)));
          
        mPilotController.leftTrigger()
            .whileTrue(mFlywheelsSS.setStateCmd(FlywheelStates.FEED_VELOCITY))
            .whileFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED));

        /* TODO: TUNE */
        mPilotController.y()
            .whileTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY))
            .whileFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE));

        mPilotController.rightTrigger().whileTrue(
            new SequentialCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).alongWith(
                mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE)).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            )
        )
        .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE))
                .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE))
                .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));
    }

    public void initGunnerBindings() {

        mGunnerController.povUp()
            .onTrue(mHoodSS.setStateCmd(HoodStates.MAX))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));
        
        // mGunnerController.povDown()
        //     .whileTrue(mHoodSS.setStateCmd(HoodStates.MIN));

        mGunnerController.povRight()
            .whileTrue(mHoodSS.setStateCmd(HoodStates.INCREMENTING))
            .whileFalse(mHoodSS.setStateCmd(HoodStates.HOLD_POSITION));
        
        mGunnerController.povLeft()
            .whileTrue(mHoodSS.setStateCmd(HoodStates.DECREMENTING))
            .whileFalse(mHoodSS.setStateCmd(HoodStates.HOLD_POSITION));

        mGunnerController.a().whileTrue(mClimbSS.unHookClawsCmd());
        mGunnerController.b().whileTrue(mClimbSS.hookClawsCmd());

        mGunnerController.leftBumper().onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.STOWED));
        mGunnerController.rightBumper().onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));

        mGunnerController.leftTrigger()
            .whileTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mGunnerController.rightTrigger()
            .whileTrue((mIntakeSS.trashCompact()))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));

        new Trigger(() -> (mGunnerController.getLeftY() < -0.25))
            .onTrue(mClimbSS.setClimbPositionManualCmd(2.47))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.STOWED))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onFalse(mClimbSS.setClimbVoltsCmd(0));
        
        new Trigger(() -> (mGunnerController.getLeftY() > 0.25))
            .onTrue(mClimbSS.setClimbVoltsCmd(-10))
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.STOWED))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onFalse(mClimbSS.setClimbVoltsCmd(0));
        
        new Trigger(
            () -> mGunnerController.getRightY() < -0.25)
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onTrue(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE))
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE));

        new Trigger(
            () -> mGunnerController.getRightY() > 0.25)
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
                .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.OUTTAKE_VOLT))
                .onTrue(mConveyorSS.setConveyorStateCmd(ConveyorState.OUTTAKE))

                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
                .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE));

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
