package frc.robot.bindings;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.game.GameGoalPoseChooser;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.conveyor.ConveyorSS.ConveyorState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.PoseConstants;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelState;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS;

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

        mPilotController.a()
            .onTrue(mDriveSS.getDriveManager().setToLinearTest())
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.b()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> GameGoalPoseChooser.getHubPresetPose(
                    mDriveSS.getPoseEstimate(), 
                    2.25), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.x()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> GameGoalPoseChooser.getHubPresetPose(
                    mDriveSS.getPoseEstimate(), 
                    2.9), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.y()
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

        mPilotController.a()
            .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(() -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), mDriveSS.getDriveManager().getDefaultTurnPointFF()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.y()
            .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(() -> PoseConstants.kClimbPose, ConstraintType.LINEAR)) // TODO: TUNE ME
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.povUp()
        //     .whileTrue(mShooter.setHoodStateCmd(HoodState.MID));
        
        // mPilotController.povDown()
        //     .whileTrue(mShooter.setHoodStateCmd(HoodState.MIN));

        // mPilotController.povRight()
        //     .whileTrue(mShooter.incrementHoodCmd());
        
        // mPilotController.povLeft()
        //     .whileTrue(mShooter.decrementHoodCmd());

        mPilotController.leftBumper()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.COMPACT))
            .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));

        mPilotController.rightBumper()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.STOWED));

        // mPilotController.leftTrigger()
        //     .whileTrue(mIntakeSS.setPivotStateCmd(
        //         IntakePivotState.INTAKE)
        //             .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //             .alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.SLOW_OUTTAKE)))
        //     .onFalse(
        //         mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE)
        //             .alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE)));

        // mPilotController.rightTrigger()
        //     .whileTrue(
        //         mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE)
        //             .alongWith(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //             .alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE))
        //             .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE)))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE)
        //         .alongWith(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE)
        //         .alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE))
        //         .alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE))));
    }

    public void initGunnerBindings() {
        // mGunnerController.povUp()
        //     .whileTrue(mShooter.setHoodStateCmd(HoodState.TUNING));

        // mGunnerController.povUp()
        //     .whileTrue(mShooter.setHoodStateCmd(HoodState.MID));
        
        // mGunnerController.povDown()
        //     .whileTrue(mShooter.setHoodStateCmd(HoodState.MIN));

        // mGunnerController.povRight()
        //     .whileTrue(mShooter.incrementHoodCmd());
        
        // mGunnerController.povLeft()
        //     .whileTrue(mShooter.decrementHoodCmd());

        mGunnerController.a().whileTrue(mClimbSS.unHookClawsCmd());
        mGunnerController.b().whileTrue(mClimbSS.hookClawsCmd());

        // mGunnerController.leftBumper().whileTrue(mShooter.setFlywheelStateCmd(FlywheelState.STANDBY))
        //     .whileFalse(mShooter.setFlywheelsVoltsCmd(0));

        // mGunnerController.rightBumper().whileTrue(mShooter.setFlywheelStateCmd(FlywheelState.TUNING))
        //     .whileFalse(mShooter.setFlywheelsVoltsCmd(0));

        // mGunnerController.leftTrigger()
        //     .whileTrue(mConveyorSS.setConveyorStateCmd(ConveyorState.OUTTAKE).alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.OUTTAKE)))
        //     .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE).alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE)));

        // mGunnerController.rightTrigger()
        //     .whileTrue(
        //         new SequentialCommandGroup(
        //             mShooter.setFuelPumpStateCmd(FuelPumpState.UNJAM).withTimeout(0.4),
        //             mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE).alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.UNJAM)).until(mShooter.isFuelPumpAtSetpoint()),
        //             mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE)
        //         ).alongWith(mIntakeSS.trashCompact())
        //     )
        //     .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE).alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE)).alongWith(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE)));

        new Trigger(() -> (mGunnerController.getRightY() < -0.25)).onTrue(mIntakeSS.trashCompact()).onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));

        new Trigger(() -> (mGunnerController.getRightY() < -0.5)).whileTrue(mIntakeSS.setPivotVoltsCmd(-4)).onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));
    }
}
