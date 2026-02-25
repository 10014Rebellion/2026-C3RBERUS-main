package frc.robot.bindings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.conveyor.ConveyorSS.ConveyorState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.Shooter;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS.HoodState;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final Shooter mShooter;
    private final Intake mIntakeSS;
    private final ConveyorSS mConveyorSS;
    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final FlydigiApex4 mGunnerController = new FlydigiApex4(BindingsConstants.kGunnerControllerPort);

    public ButtonBindings(Drive pDriveSS, Shooter pShooter, Intake pIntake, ConveyorSS pConveyorSS) {
        this.mDriveSS = pDriveSS;
        this.mShooter = pShooter;
        this.mIntakeSS = pIntake;
        this.mConveyorSS = pConveyorSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.getDriveManager().setToTeleop());
    }


    public void initBindings() {
        initPilotBindings();
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

    public void initPilotBindings() {
        mDriveSS.getDriveManager().acceptJoystickInputs(
                () -> -mPilotController.getLeftY(),
                () -> -mPilotController.getLeftX(),
                () -> -mPilotController.getRightX(),
                () -> mPilotController.getPOVAngle());
        
        mPilotController.povUp()
            .whileTrue(mShooter.setHoodStateCmd(HoodState.MID));
        
        mPilotController.povDown()
            .whileTrue(mShooter.setHoodStateCmd(HoodState.MIN));

        mPilotController.povRight()
            .whileTrue(mShooter.incrementHoodCmd());
        
        mPilotController.povLeft()
            .whileTrue(mShooter.decrementHoodCmd());

        mPilotController.rightBumper()
            .whileTrue(mIntakeSS.setPivotRotManualCmd());

        mPilotController.leftBumper()
            .whileTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.STOWED));

        mPilotController.leftTrigger()
            .whileTrue(mIntakeSS.setPivotStateCmd(IntakePivotState.INTAKE));
        
        

    }
}
