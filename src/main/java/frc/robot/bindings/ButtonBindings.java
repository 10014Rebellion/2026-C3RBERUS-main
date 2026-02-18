// REBELLION 10014

package frc.robot.bindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.shooter.Shooter;
import frc.robot.commands.DriveCharacterizationCommands;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final Shooter mShooter;
    private final Intake mIntake;
    private final ConveyorSS mConveyorSS;
    private final FlydigiApex4 mDriverController = new FlydigiApex4(BindingsConstants.kDriverControllerPort);

    public ButtonBindings(Drive pDriveSS, Shooter pShooter, Intake pIntake, ConveyorSS pConveyorSS) {
        this.mDriveSS = pDriveSS;
        this.mShooter = pShooter;
        this.mIntake = pIntake;
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

        mDriverController.y().onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        mDriverController.a()
            .onTrue(mConveyorSS.setConveyorVoltsCmd(9))
            .onFalse(mConveyorSS.setConveyorVoltsCmd(0));

        mDriverController.povUp()
            .onTrue(mShooter.setHoodRot(Rotation2d.fromDegrees(20)));

        mDriverController.povRight()
            .onTrue(mShooter.setHoodRot(Rotation2d.fromDegrees(10)));

        mDriverController.povDown()
            .onTrue(mShooter.setHoodRot(Rotation2d.kZero));

        mDriverController.rightTrigger()
            .onTrue(mShooter.setFuelPumpsVoltsCmd(9))
            .onFalse(mShooter.setFuelPumpsVoltsCmd(0.0));

        mDriverController.leftTrigger()
            .onTrue(mShooter.setFlywheelsRPSCmd(90))
            .onFalse(mShooter.setFlywheelsRPSCmd(0));

        mDriverController.leftBumper()
            .onTrue(mShooter.setFlywheelsVoltsCmd(12))
            .onFalse(mShooter.setFlywheelsVoltsCmd(0));

        // mDriverController.a()
        //     .onTrue(
        //         Commands.runOnce(() -> mDriveSS.setPose(new Pose2d()), mDriveSS)
        //             .andThen(mDriveSS.setToGenericAutoAlign(() -> new Pose2d(0.5, 0, Rotation2d.kCCW_Pi_2), ConstraintType.LINEAR)))
        //     .onFalse(mDriveSS.setToTeleop());

        mDriverController.a()
            .onTrue(
                mDriveSS.getDriveManager().setToGenericHeadingAlign(
                    () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()),
                    () -> GameGoalPoseChooser.getHub()))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());
            
       mDriverController.b()
            .onTrue(mDriveSS.getDriveManager().setToGenericLineAlign(
                () -> new Pose2d(3.0, 3.0, Rotation2d.kZero),
                () -> Rotation2d.fromDegrees(45),
                () -> 1.0,
                () -> false
            ))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());

        mDriverController.x()
            .onTrue(DriveCharacterizationCommands.testAzimuthsVoltage(mDriveSS, 0, 1, 2, 3))
            .onFalse(mDriveSS.getDriveManager().setToTeleop());
    }
}
