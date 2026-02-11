// REBELLION 10014

package frc.robot.bindings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.shooter.Shooter;

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
        this.mDriveSS.setDefaultCommand(mDriveSS.setToTeleop());
    }


    public void initDriverButtonBindings() {
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(mDriveSS.setToTeleop());

        mDriveSS.acceptJoystickInputs(
                () -> -mDriverController.getLeftY(), // TODO: interrogate Anshul on why theses are flipped and why on earth it works
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
            .onTrue(mShooter.setIndexersVoltsCmd(9))
            .onFalse(mShooter.setIndexersVoltsCmd(0.0));

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

        // mDriverController.a()
        //     .onTrue(
        //         mDriveSS.setToGenericHeadingAlign(() -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate())))
        //     .onFalse(mDriveSS.setToTeleop());
            
    //    mDriverController.b()
    //         .onTrue(mDriveSS.setToGenericLineAlign(
    //             () -> new Pose2d(3.0, 3.0, Rotation2d.kZero),
    //             () -> Rotation2d.fromDegrees(45),
    //             () -> 1.0,
    //             () -> false
    //         ))
    //         .onFalse(mDriveSS.setToTeleop());


        // mDriverController.a()
        //     .onTrue(Commands.runOnce(() -> mDriveSS.setPose(new Pose2d()), mDriveSS)
        //             .andThen(mDriveSS.setToGenericLineAlign(() -> new Pose2d(2.5, 2.5, Rotation2d.fromDegrees(45.0)), () -> Rotation2d.fromDegrees(45.0))));
    
        // mDriverController.a()
        //     .onTrue(new InstantCommand())
        //     .onFalse(new InstantCommand());


        // mDriverController.rightTrigger()
        //     .onTrue(mShooterSS.setIndexersVoltsCmd(12))
        //     .onFalse(mShooterSS.setIndexersVoltsCmd(0));

        // mDriverController.leftBumper()
        //     .onTrue(mShooterSS.setHoodVoltsCmd(1))
        //     .onFalse(mShooterSS.setHoodVoltsCmd(0));

        // mDriverController.rightBumper()
        //     .onTrue(mShooterSS.setHoodVoltsCmd(-1))
        //     .onFalse(mShooterSS.setHoodVoltsCmd(0));
    }
}
