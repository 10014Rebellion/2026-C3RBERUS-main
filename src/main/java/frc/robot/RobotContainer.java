// REBELLION 10014

package frc.robot;

import static frc.robot.systems.drive.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auton.AutonCommands;
import frc.robot.bindings.BindingsConstants;
import frc.robot.bindings.ButtonBindings;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroIOPigeon2;
import frc.robot.systems.drive.modules.Module;
import frc.robot.systems.drive.modules.ModuleIO;
import frc.robot.systems.drive.modules.ModuleIOKraken;
import frc.robot.systems.drive.modules.ModuleIOSim;
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.ShooterConstants.IndexerConstants;
import frc.robot.systems.shooter.flywheels.FlywheelIOKrakenx44;
import frc.robot.systems.shooter.flywheels.Flywheels;
import frc.robot.systems.shooter.indexers.IndexerIOKrakenx44;
import frc.robot.systems.shooter.indexers.Indexers;
import frc.robot.systems.apriltag.ATagCameraIO;
import frc.robot.systems.apriltag.ATagCameraIOPV;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVisionConstants;
import frc.robot.systems.conveyor.Conveyor;
import frc.robot.systems.conveyor.ConveyorConstants;
import frc.robot.systems.conveyor.ConveyorIOKrakenx44;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer {
    private final Drive mDrive;
    private final Flywheels mFlywheels;
    private final Indexers mIndexers;
    private final Conveyor mConveyor;
    private final LoggedDashboardChooser<Command> mDriverProfileChooser = new LoggedDashboardChooser<>("DriverProfile");
    private final ButtonBindings mButtonBindings;
    private final AutonCommands autos;

    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL:
                mDrive = new Drive(
                    new Module[] {
                        new Module("FL", new ModuleIOKraken(kFrontLeftHardware)),
                        new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                        new Module("BL", new ModuleIOKraken(kBackLeftHardware)),
                        new Module("BR", new ModuleIOKraken(kBackRightHardware))
                    },
                    new GyroIOPigeon2(),
                    new ATagVision(new ATagCameraIO[]{
                        new ATagCameraIOPV(ATagVisionConstants.kFLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kFRATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBRATagCamHardware)
                    }));
                break;

            case SIM:
                mDrive = new Drive(
                    new Module[] {
                        new Module("FL", new ModuleIOSim()),
                        new Module("FR", new ModuleIOSim()),
                        new Module("BL", new ModuleIOSim()),
                        new Module("BR", new ModuleIOSim())
                    },
                    new GyroIO() {},
                    new ATagVision(new ATagCameraIO[]{
                        new ATagCameraIOPV(ATagVisionConstants.kFLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kFRATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBRATagCamHardware)
                    }));
                break;

            default:
                mDrive = new Drive(
                    new Module[] {
                        new Module("FL", new ModuleIO() {}),
                        new Module("FR", new ModuleIO() {}),
                        new Module("BL", new ModuleIO() {}),
                        new Module("BR", new ModuleIO() {})
                    },
                    new GyroIO() {},
                    new ATagVision(new ATagCameraIO[] {
                        new ATagCameraIO() {}, 
                        new ATagCameraIO() {}, 
                        new ATagCameraIO() {}, 
                        new ATagCameraIO() {}
                    }));
                break;
        }
        
        FlywheelIOKrakenx44 flywheelLeader = new FlywheelIOKrakenx44(ShooterConstants.FlywheelConstants.kFlywheelLeaderConfig);
        mFlywheels = new Flywheels(
            flywheelLeader,
            new FlywheelIOKrakenx44(ShooterConstants.FlywheelConstants.kFlywheelFollowerConfig, flywheelLeader)
        );
        
        mConveyor = new Conveyor(new ConveyorIOKrakenx44(ConveyorConstants.kConveyorMotorConstants));

        mIndexers = new Indexers(
            new IndexerIOKrakenx44(IndexerConstants.kIndexerLeaderConfig), new IndexerIOKrakenx44(IndexerConstants.kIndexerFollowerConfig));

        mButtonBindings = new ButtonBindings(mDrive, mFlywheels, mIndexers, mConveyor);

        initBindings();

        mDriverProfileChooser.addDefaultOption(
                BindingsConstants.kDefaultProfile.key(), mDrive.setDriveProfile(BindingsConstants.kDefaultProfile));
        for (DriverProfiles profile : BindingsConstants.kProfiles)
            mDriverProfileChooser.addOption(profile.key(), mDrive.setDriveProfile(profile));

        autos = new AutonCommands(mDrive);
    }

    public Drive getDrivetrain() {
        return mDrive;
    }

    private void initBindings() {
        mButtonBindings.initDriverButtonBindings();
    }

    public Command getAutonomousCommand() {
        return autos.getAuto();
    }

    public Command getDriverProfileCommand() {
        return mDriverProfileChooser.get();
    }
}
