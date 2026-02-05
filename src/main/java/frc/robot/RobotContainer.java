// REBELLION 10014

package frc.robot;

import static frc.robot.systems.drive.DriveConstants.*;

import java.util.function.Supplier;

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
import frc.robot.systems.apriltag.ATagCameraIO;
import frc.robot.systems.apriltag.ATagCameraIOPV;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVisionConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer {
    private final Drive mDrive;
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

        mButtonBindings = new ButtonBindings(mDrive);

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

    public Supplier<Command> getAutonomousCommand() {
        return autos.getAuto();
    }

    public Command getDriverProfileCommand() {
        return mDriverProfileChooser.get();
    }
}
