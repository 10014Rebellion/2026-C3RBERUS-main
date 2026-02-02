package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.tuning.SysIDCharacterization;
import frc.robot.systems.drive.Drive;

public class DriveCharacterizationCommands {
        /* LINEAR CHARACTERIZATION: The x-y movement of the drivetrain(basically drive motor feedforward) */
    public static Command characterizeLinearMotion(Drive pDrive) {
        return pDrive.setToSysIDCharacterization()
            .andThen(SysIDCharacterization.runDriveSysIDTests(
                (voltage) -> {
                    runLinearCharacterization(voltage, pDrive);
                }, pDrive));
    }

    /* Runs the robot forward at a voltage */
    public static void runLinearCharacterization(double volts, Drive pDrive) {
        pDrive.setToSysIDCharacterization().initialize();
        for (int i = 0; i < 4; i++) pDrive.getModules()[i].runCharacterization(volts);
    }

    public static void setDriveAmperagesForAllModules(double amps, Drive pDrive) {
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
    public static Command characterizeAngularMotion(Drive pDrive) {
        return pDrive.setToSysIDCharacterization()
            .andThen(SysIDCharacterization.runDriveSysIDTests(
                (voltage) -> runAngularCharacterization(voltage, pDrive), pDrive));
    }

    /* Runs the rotate's robot at a voltage */
    public static void runAngularCharacterization(double volts, Drive pDrive) {
        pDrive.setToSysIDCharacterization().initialize();
        pDrive.getModules()[0].runCharacterization(volts, Rotation2d.fromDegrees(-45.0));
        pDrive.getModules()[1].runCharacterization(-volts, Rotation2d.fromDegrees(45.0));
        pDrive.getModules()[2].runCharacterization(volts, Rotation2d.fromDegrees(45.0));
        pDrive.getModules()[3].runCharacterization(-volts, Rotation2d.fromDegrees(-45.0));
    }

    public static Command characterizeAzimuthsVoltage(int pModNumber, Drive pDrive) {
        return pDrive.setToSysIDCharacterization()
            .andThen(SysIDCharacterization.runDriveSysIDTests(
                (voltage) -> {
                pDrive.getModules()[pModNumber].setDesiredRotation(null);
                pDrive.getModules()[pModNumber].setDesiredVelocity(0.0);
                pDrive.getModules()[pModNumber].setAzimuthVoltage(voltage);
            }, pDrive));
    }

    public static Command testAzimuthsVoltage(Drive pDrive, int... pModNumber) {
        return characterizeAzimuthsVoltage(Drive.tAzimuthCharacterizationVoltage, pDrive, pModNumber);
    }

    public static Command characterizeAzimuthsVoltage(DoubleSupplier voltage, Drive pDrive, int... pModNumbers) {
        return new FunctionalCommand(
            () -> pDrive.setToSysIDCharacterization().initialize(), 
            () -> {
                for(int moduleNumber : pModNumbers) {
                    pDrive.getModules()[moduleNumber].setDesiredRotation(null);
                    pDrive.getModules()[moduleNumber].setDesiredVelocity(0.0);
                    pDrive.getModules()[moduleNumber].setAzimuthVoltage(voltage.getAsDouble());
                }
            }, 
            (interrupted) -> {}, 
            () -> false, 
            pDrive);
    }

    public static Command testAzimuthsAmps(Drive pDrive, int... pModNumber) {
        return characterizeAzimuthsAmps(Drive.tAzimuthCharacterizationAmps, pDrive, pModNumber);
    }

    public static Command characterizeAzimuthsAmps(DoubleSupplier amps, Drive pDrive, int... pModNumbers) {
        return new FunctionalCommand(
            () -> pDrive.setToSysIDCharacterization().initialize(), 
            () -> {
                for(int moduleNumber : pModNumbers) {
                    pDrive.getModules()[moduleNumber].setDesiredRotation(null);
                    pDrive.getModules()[moduleNumber].setDesiredVelocity(0.0);
                    pDrive.getModules()[moduleNumber].setAzimuthAmps(amps.getAsDouble());
                }
            }, 
            (interrupted) -> {}, 
            () -> false, 
            pDrive);
    }
}
