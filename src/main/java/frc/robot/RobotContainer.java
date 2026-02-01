// REBELLION 10014

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.bindings.BindingsConstants;
import frc.robot.bindings.ButtonBindings;
import frc.robot.game.StateTracker;
import frc.robot.systems.apriltag.AprilTag;
import frc.robot.systems.apriltag.AprilTagConstants;
import frc.robot.systems.apriltag.AprilTagIO;
import frc.robot.systems.apriltag.AprilTagIOPVTag;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.DriveConstants;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroIOPigeon2;
import frc.robot.systems.drive.modules.Module;
import frc.robot.systems.drive.modules.ModuleIO;
import frc.robot.systems.drive.modules.ModuleIOKraken;
import frc.robot.systems.drive.modules.ModuleIOSim;
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.ShooterConstants.IndexerConstants;
import frc.robot.systems.shooter.flywheels.Flywheels;
import frc.robot.systems.shooter.indexers.IndexerIOKrakenx44;
import frc.robot.systems.shooter.indexers.Indexers;
import frc.robot.systems.shooter.flywheels.FlywheelIOKrakenx44;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final Drive mDrive;
    private final Flywheels mFlywheels;
    private final Indexers mIndexers;


    private final LoggedDashboardChooser<Command> mDriverProfileChooser = new LoggedDashboardChooser<>("DriverProfile");
    private final ButtonBindings mButtonBindings;

    public RobotContainer() {
        new StateTracker();

        switch (Constants.kCurrentMode) {
            case REAL:
                mDrive = new Drive(
                        new Module[] {
                            new Module("FL", new ModuleIOKraken(DriveConstants.kFrontLeftHardware)),
                            new Module("FR", new ModuleIOKraken(DriveConstants.kFrontRightHardware)),
                            new Module("BL", new ModuleIOKraken(DriveConstants.kBackLeftHardware)),
                            new Module("BR", new ModuleIOKraken(DriveConstants.kBackRightHardware))
                        },
                        new GyroIOPigeon2(),
                        new AprilTag(new AprilTagIO[] {
                            new AprilTagIOPVTag(
                                AprilTagConstants.kRightCamName,
                                AprilTagConstants.kRightCamTransform,
                                AprilTagConstants.kRightCamOrientation
                            ),
                            new AprilTagIOPVTag(
                                AprilTagConstants.kLeftCamName,
                                AprilTagConstants.kLeftCamTransform,
                                AprilTagConstants.kLeftCamOrientation
                            )
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
                        new AprilTag(new AprilTagIO[] {
                            new AprilTagIOPVTag(
                                AprilTagConstants.kRightCamName,
                                AprilTagConstants.kRightCamTransform,
                                AprilTagConstants.kRightCamOrientation
                            ),
                            new AprilTagIOPVTag(
                                AprilTagConstants.kLeftCamName,
                                AprilTagConstants.kLeftCamTransform,
                                AprilTagConstants.kLeftCamOrientation
                            )
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
                        new AprilTag(new AprilTagIO[] {new AprilTagIO() {}, new AprilTagIO() {}}));
                break;
        }

        FlywheelIOKrakenx44 flywheelLeader = new FlywheelIOKrakenx44(ShooterConstants.FlywheelConstants.kFlywheelLeaderConfig);
        mFlywheels = new Flywheels(
            flywheelLeader,
            new FlywheelIOKrakenx44(ShooterConstants.FlywheelConstants.kFlywheelFollowerConfig, flywheelLeader)
        );

        mIndexers = new Indexers(
            new IndexerIOKrakenx44(IndexerConstants.kIndexerLeaderConfig), new IndexerIOKrakenx44(IndexerConstants.kIndexerFollowerConfig));

        mButtonBindings = new ButtonBindings(mDrive, mFlywheels, mIndexers);

        initBindings();

        mDriverProfileChooser.addDefaultOption(
                BindingsConstants.kDefaultProfile.key(), mDrive.setDriveProfile(BindingsConstants.kDefaultProfile));
        for (DriverProfiles profile : BindingsConstants.kProfiles)
            mDriverProfileChooser.addOption(profile.key(), mDrive.setDriveProfile(profile));
    }

    public Drive getDrivetrain() {
        return mDrive;
    }

    private void initBindings() {
        mButtonBindings.initDriverButtonBindings();
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public Command getDriverProfileCommand() {
        return mDriverProfileChooser.get();
    }
}
