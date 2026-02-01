// REBELLION 10014

package frc.robot.bindings;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.shooter.flywheels.Flywheels;
import frc.robot.systems.shooter.indexers.Indexers;
import frc.robot.commands.DriveCharacterizationCommands;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final Flywheels mFlywheelsSS;
    private final Indexers mIndexers;
    private final FlydigiApex4 mDriverController = new FlydigiApex4(BindingsConstants.kDriverControllerPort);

    public ButtonBindings(Drive pDriveSS, Flywheels pFlywheelsSS, Indexers pIndexersSS) {
        this.mFlywheelsSS = pFlywheelsSS;
        this.mIndexers = pIndexersSS;
        this.mDriveSS = pDriveSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.setToTeleop());
    }

    public void initDriverButtonBindings() {
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.setToTeleop());

        mDriveSS.acceptJoystickInputs(
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.getPOVAngle());

        mDriverController.y().onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        // mDriverController.a()
        //     .onTrue(
        //         Commands.runOnce(() -> mDriveSS.setPose(new Pose2d()), mDriveSS)
        //             .andThen(mDriveSS.setToGenericAutoAlign(() -> new Pose2d(0.5, 0, Rotation2d.kCCW_Pi_2), ConstraintType.LINEAR)))
        //     .onFalse(mDriveSS.setToTeleop());

        mDriverController.a()
            .onTrue(
                mDriveSS.setToGenericHeadingAlign(() -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate())))
            .onFalse(mDriveSS.setToTeleop());
            
    //    mDriverController.b()
    //         .onTrue(mDriveSS.setToGenericLineAlign(
    //             () -> new Pose2d(3.0, 3.0, Rotation2d.kZero),
    //             () -> Rotation2d.fromDegrees(45),
    //             () -> 1.0,
    //             () -> false
    //         ))
    //         .onFalse(mDriveSS.setToTeleop());

        mDriverController.x()
            .onTrue(DriveCharacterizationCommands.testAzimuthsVoltage(mDriveSS, 0, 1, 2, 3))
            .onFalse(mDriveSS.setToTeleop());

        // mDriverController.a()
        //     .onTrue(Commands.runOnce(() -> mDriveSS.setPose(new Pose2d()), mDriveSS)
        //             .andThen(mDriveSS.setToGenericLineAlign(() -> new Pose2d(2.5, 2.5, Rotation2d.fromDegrees(45.0)), () -> Rotation2d.fromDegrees(45.0))));
    
        mDriverController.rightTrigger()
            .onTrue(new InstantCommand(() -> mFlywheelsSS.setFlywheelVolts(mDriverController.getRightTriggerAxis() * 12)))
            .onFalse(new InstantCommand(() -> mFlywheelsSS.setFlywheelVolts(0)));

        mDriverController.leftTrigger()
            .onTrue(new InstantCommand(() -> mIndexers.setIndexerVolts(12)))
            .onFalse(new InstantCommand(() -> mIndexers.setIndexerVolts(0)));

    }
}
