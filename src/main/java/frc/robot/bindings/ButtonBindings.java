package frc.robot.bindings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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

        // new Trigger(() -> DriverStation.isTeleopEnabled())
        //     .onTrue(mDriveSS.getDriveManager().setToTeleop()
        //         .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE))
        //         .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //         .alongWith(mShooter.setFlywheelStateCmd(FlywheelState.IDLE))
        //         .alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE))
        //     );
        
        // new Trigger(() -> DriverStation.isFMSAttached()).and(() -> DriverStation.isTeleopEnabled())
        //     .onTrue(mShooter.setHoodStateCmd(HoodState.MIN)
        //         .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE)));
    }

    public void initButtonBoard() {
        mHID.button(3)
            .whileTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.TUNING_VELOCITY))
            .whileFalse(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE));

        mHID.button(4)
            .whileTrue(mHoodSS.setHoodTuneableCmdNoEnd())
            .onFalse(Commands.runOnce(() -> mHoodSS.clearGoalWithoutStateChange(), mHoodSS));

        mHID.button(9)
            .onTrue(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.INTAKE)
                .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.CONVEY_TO_INDEX))
                .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .alongWith(mIntakeSS.trashCompact()))
            .onFalse(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED)
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
        //     .onTrue(mDriveSS.getDriveManager().setToLinearTest())
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.b()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //         () -> GameGoalPoseChooser.getHubPresetPose(
        //             mDriveSS.getPoseEstimate(), 
        //             2.25), 
        //         ConstraintType.LINEAR))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.x()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //         () -> GameGoalPoseChooser.getHubPresetPose(
        //             mDriveSS.getPoseEstimate(), 
        //             2.9), 
        //         ConstraintType.LINEAR))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.b()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> AllianceFlipUtil.apply(new Pose2d(3.25, 4.02, Rotation2d.kZero)), 
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
            .whileTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MID));
        
        mPilotController.povDown()
            .whileTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN));

        mPilotController.povRight()
            .whileTrue(mHoodSS.incrementAngleCmd());
        
        mPilotController.povLeft()
            .whileTrue(mHoodSS.decrementAngleCmd());

        mPilotController.leftBumper()
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        mPilotController.rightBumper()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE)))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE).alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE)));
          
        mPilotController.leftTrigger()
            .whileTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.TUNING_VELOCITY))
            .whileFalse(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE));

        mPilotController.y()
            .whileTrue(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.TUNING_VELOCITY))
            .whileTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.CLOSE_SHOT))
            .whileFalse(mFlywheelsSS.setFlywheelStateCmd(FlywheelState.IDLE))
            .whileFalse(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN));

        mPilotController.rightTrigger().whileTrue(
            new SequentialCommandGroup(
                // mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.UNJAM).withTimeout(0.4),
                // mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.INTAKE).alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.UNJAM)).until(()->mFuelPumpSS.isReadyToShoot()),
                mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.INTAKE).until(()->mFuelPumpSS.isReadyToShoot()),
                mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE)
            )
        )
        .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE).alongWith(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED)).alongWith(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE)));
    }

    public void initGunnerBindings() {

        mGunnerController.povUp()
            .whileTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MAX));
        
        mGunnerController.povDown()
            .whileTrue(mHoodSS.setGoalCmd(HoodClosedSetpoints.MIN));

        mGunnerController.povRight()
            .whileTrue(mHoodSS.incrementAngleCmd());
        
        mGunnerController.povLeft()
            .whileTrue(mHoodSS.decrementAngleCmd());;

        mGunnerController.a().whileTrue(mClimbSS.unHookClawsCmd());
        mGunnerController.b().whileTrue(mClimbSS.hookClawsCmd());

        mGunnerController.leftBumper().onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.STOWED));
        mGunnerController.rightBumper().onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));

        mGunnerController.leftTrigger()
            .whileTrue(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.OUTTAKE))
            .onFalse(mFuelPumpSS.setFuelPumpStateCmd(FuelPumpState.STOPPED));

        mGunnerController.rightTrigger()
            .whileTrue((mIntakeSS.trashCompact()))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));
    }
}
