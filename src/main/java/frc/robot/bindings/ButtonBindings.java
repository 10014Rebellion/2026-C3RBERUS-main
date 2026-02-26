package frc.robot.bindings;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.conveyor.ConveyorSS.ConveyorState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.shooter.Shooter;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelState;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS.HoodState;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final Shooter mShooter;
    private final Intake mIntakeSS;
    private final ClimbSS mClimbSS;
    private final ConveyorSS mConveyorSS;
    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final FlydigiApex4 mGunnerController = new FlydigiApex4(BindingsConstants.kGunnerControllerPort);

    public ButtonBindings(Drive pDriveSS, Shooter pShooter, Intake pIntake, ConveyorSS pConveyorSS, ClimbSS pClimbSS) {
        this.mDriveSS = pDriveSS;
        this.mShooter = pShooter;
        this.mIntakeSS = pIntake;
        this.mConveyorSS = pConveyorSS;
        this.mClimbSS = pClimbSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.getDriveManager().setToTeleop());
    }


    public void initBindings() {
        initPilotBindings();
        initGunnerBindings();
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.getDriveManager().setToTeleop());

        // mDriveSS.getDriveManager().acceptJoystickInputs(
        //         () -> -mPilotController.getLeftY(),
        //         () -> -mPilotController.getLeftX(),
        //         () -> -mPilotController.getRightX(),
        //         () -> mPilotController.getPOVAngle());

        // mPilotController.startButton().onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        // mPilotController.rightTrigger()
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));
        //     // .onFalse(mIntakeSS.setPivotState(IntakePivotState.IDL8));

        // mPilotController.rightBumper()
        //     .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.IDLE))
        //     .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));

        // mPilotController.y()
        //     .onTrue(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE))
        //     .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE));

        // mPilotController.a()
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        // mPilotController.b()
        //     .onTrue(mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE))
        //     .onFalse(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE));

    }

    public Command rumbleDriverController(){
        return Commands.startEnd(
            () -> mPilotController.setRumble(RumbleType.kBothRumble, 1.0), 
            () -> mPilotController.setRumble(RumbleType.kBothRumble, 0.5));
    }

    public void initPilotBindings() {

        // Rumble the driver controller if the conveyer is jammed //
        // new Trigger(mConveyorSS::isConveyerJammed)
        // .onTrue(rumbleDriverController().withTimeout(0.8));

        mDriveSS.getDriveManager().acceptJoystickInputs(
                () -> -mPilotController.getLeftY(),
                () -> -mPilotController.getLeftX(),
                () -> -mPilotController.getRightX(),
                () -> mPilotController.getPOVAngle());

        // UNCOMMENT THESE BINDINGS ONCE VISION IS IMPLEMENTED
        // mPilotController.a()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(() -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()), mDriveSS.getDriveManager().getDefaultTurnPointFF()))
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // mPilotController.y()
        //     .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(() -> PoseConstants.kClimbPose, ConstraintType.LINEAR)) // TODO: TUNE ME
        //     .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mPilotController.povUp()
            .whileTrue(mShooter.setHoodStateCmd(HoodState.MID));
        
        mPilotController.povDown()
            .whileTrue(mShooter.setHoodStateCmd(HoodState.MIN));

        mPilotController.povRight()
            .whileTrue(mShooter.incrementHoodCmd());
        
        mPilotController.povLeft()
            .whileTrue(mShooter.decrementHoodCmd());

        mPilotController.rightBumper()
            .onTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.COMPACT));

        mPilotController.leftBumper()
            .whileTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.STOWED));

        mPilotController.leftTrigger()
            .whileTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));
    }

    public void initGunnerBindings() {
        mGunnerController.povUp()
            .whileTrue(mShooter.setHoodStateCmd(HoodState.MID));
        
        mGunnerController.povDown()
            .whileTrue(mShooter.setHoodStateCmd(HoodState.MIN));

        mGunnerController.povRight()
            .whileTrue(mShooter.incrementHoodCmd());
        
        mGunnerController.povLeft()
            .whileTrue(mShooter.decrementHoodCmd());

        mGunnerController.a().whileTrue(mClimbSS.unHookClawsCmd());
        mGunnerController.b().whileTrue(mClimbSS.hookClawsCmd());

        // mGunnerController.leftBumper().whileTrue(mShooter.setFlywheelsRPSCmd(FlywheelState.SHOOT_FAR));

        mGunnerController.rightBumper().whileTrue(mShooter.setFlywheelStateCmd(FlywheelState.STANDBY))
            .whileFalse(mShooter.setFlywheelsVoltsCmd(0));

        mGunnerController.leftTrigger()
            .whileTrue(mConveyorSS.setConveyorStateCmd(ConveyorState.OUTTAKE).alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.OUTTAKE)))
            .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE).alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE)));

        mGunnerController.rightTrigger()
            .whileTrue(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE).alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE)))
            .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE).alongWith(mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE)));

        new Trigger(() -> (mGunnerController.getLeftY() > 0.5)).whileTrue(mIntakeSS.setPivotVoltsCmd(10)).onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));

        new Trigger(() -> (mGunnerController.getLeftY() < -0.5)).whileTrue(mIntakeSS.setPivotVoltsCmd(-4)).onFalse(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));
    }
}
