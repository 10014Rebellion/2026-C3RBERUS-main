package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.tuning.SysIDCharacterization;
import frc.robot.systems.drive.Drive;

public class DriveCharacterizationCommands {
        /* LINEAR CHARACTERIZATION: The x-y movement of the drivetrain(basically drive motor feedforward) */
    public Command characterizeLinearMotion(Drive pDrive) {
        return pDrive.setToSysIDCharacterization()
            .andThen(SysIDCharacterization.runDriveSysIDTests(
                (voltage) -> {
                    runLinearCharacterization(voltage, pDrive);
                }, pDrive));
    }

    /* Runs the robot forward at a voltage */
    public void runLinearCharacterization(double volts, Drive pDrive) {
        pDrive.setToSysIDCharacterization().initialize();
        for (int i = 0; i < 4; i++) pDrive.getModules()[i].runCharacterization(volts);
    }

    public void setDriveAmperagesForAllModules(double amps, Drive pDrive) {
        for (int i = 0; i < 4; i++) {
            pDrive.getModules()[i].setDesiredVelocity(null);
            pDrive.getModules()[i].setDriveAmperage(amps);
            pDrive.getModules()[i].setDesiredRotation(Rotation2d.fromDegrees(0.0));
        }
    }

    /*
     * ANGULAR CHARACTERIZATION: The angular movement of the drivetrain(basically used to get drivebase MOI)
     * https://choreo.autos/usage/estimating-moi/
     */
    public Command characterizeAngularMotion(Drive pDrive) {
        return pDrive.setToSysIDCharacterization()
            .andThen(SysIDCharacterization.runDriveSysIDTests(
                (voltage) -> runAngularCharacterization(voltage, pDrive), pDrive));
    }

    /* Runs the rotate's robot at a voltage */
    public void runAngularCharacterization(double volts, Drive pDrive) {
        pDrive.setToSysIDCharacterization().initialize();
        pDrive.getModules()[0].runCharacterization(volts, Rotation2d.fromDegrees(-45.0));
        pDrive.getModules()[1].runCharacterization(-volts, Rotation2d.fromDegrees(45.0));
        pDrive.getModules()[2].runCharacterization(volts, Rotation2d.fromDegrees(45.0));
        pDrive.getModules()[3].runCharacterization(-volts, Rotation2d.fromDegrees(-45.0));
    }

    public Command characterizeAzimuths(int pModNumber, Drive pDrive) {
        return pDrive.setToSysIDCharacterization()
            .andThen(SysIDCharacterization.runDriveSysIDTests(
                (voltage) -> {
                pDrive.getModules()[pModNumber].setDesiredRotation(null);
                pDrive.getModules()[pModNumber].setDesiredVelocity(0.0);
                pDrive.getModules()[pModNumber].setAzimuthVoltage(voltage);
            }, pDrive));
    }

    public Command testAzimuths(int pModNumber, Drive pDrive) {
        return characterizeAzimuths(pModNumber, Drive.tAzimuthCharacterizationVoltage, pDrive);
    }

    public Command characterizeAzimuths(int pModNumber, DoubleSupplier voltage, Drive pDrive) {
        return new FunctionalCommand(
            () -> pDrive.setToSysIDCharacterization().initialize(), 
            () -> {
                pDrive.getModules()[pModNumber].setDesiredRotation(null);
                pDrive.getModules()[pModNumber].setDesiredVelocity(0.0);
                pDrive.getModules()[pModNumber].setAzimuthVoltage(voltage.getAsDouble());
            }, 
            (interrupted) -> {}, 
            () -> false, 
            pDrive);
    }
}
