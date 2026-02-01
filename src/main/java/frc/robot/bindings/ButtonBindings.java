// REBELLION 10014

package frc.robot.bindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.FlydigiApex4;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.shooter.flywheels.Flywheels;
import frc.robot.systems.shooter.indexers.Indexers;

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
        mDriveSS.acceptJoystickInputs(
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.getPOVAngle());

        mDriverController.y().onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        mDriverController.a()
            .onTrue(
                Commands.runOnce(() -> mDriveSS.setPose(new Pose2d()), mDriveSS)
                    .andThen(mDriveSS.setToGenericAutoAlign(() -> new Pose2d(0.5, 0, Rotation2d.kCCW_Pi_2), ConstraintType.LINEAR)))
            .onFalse(mDriveSS.setToTeleop());

        mDriverController.x()
            .onTrue(mDriveSS.characterizeAzimuths(0))
            .onFalse(mDriveSS.setToTeleop());

        mDriverController.rightTrigger()
            .onTrue(new InstantCommand(() -> mFlywheelsSS.setFlywheelVolts(mDriverController.getRightTriggerAxis() * 12)))
            .onFalse(new InstantCommand(() -> mFlywheelsSS.setFlywheelVolts(0)));

        mDriverController.leftTrigger()
            .onTrue(new InstantCommand(() -> mIndexers.setIndexerVolts(12)))
            .onFalse(new InstantCommand(() -> mIndexers.setIndexerVolts(0)));
    }
}
