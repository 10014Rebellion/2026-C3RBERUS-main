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

public class ButtonBindings {
    private final Drive mDriveSS;
    private final Shooter mShooter;
    private final Intake mIntakeSS;
    private final ConveyorSS mConveyorSS;
    private final FlydigiApex4 mDriverController = new FlydigiApex4(BindingsConstants.kDriverControllerPort);

    public ButtonBindings(Drive pDriveSS, Shooter pShooter, Intake pIntake, ConveyorSS pConveyorSS) {
        this.mDriveSS = pDriveSS;
        this.mShooter = pShooter;
        this.mIntakeSS = pIntake;
        this.mConveyorSS = pConveyorSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.getDriveManager().setToTeleop());
    }


    public void initDriverButtonBindings() {
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.getDriveManager().setToTeleop());

        mDriveSS.getDriveManager().acceptJoystickInputs(
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.getPOVAngle());

        mDriverController.startButton().onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        mDriverController.rightTrigger()
            .onTrue(mIntakeSS.setPivotState(IntakePivotState.INTAKE));
            // .onFalse(mIntakeSS.setPivotState(IntakePivotState.IDL8));

        mDriverController.rightBumper()
            .onTrue(mIntakeSS.setPivotState(IntakePivotState.IDLE))
            .onFalse(mIntakeSS.setPivotState(IntakePivotState.INTAKE));

        mDriverController.y()
            .onTrue(mConveyorSS.setConveyorState(ConveyorState.INTAKE))
            .onFalse(mConveyorSS.setConveyorState(ConveyorState.IDLE));

        mDriverController.a()
            .onTrue(mIntakeSS.setRollerState(IntakeRollerState.INTAKE))
            .onFalse(mIntakeSS.setRollerState(IntakeRollerState.IDLE));

        mDriverController.b()
            .onTrue(mShooter.setFuelPumpsVoltsCmd(10.0))
            .onFalse(mShooter.setFuelPumpsVoltsCmd(0));

    }
}
